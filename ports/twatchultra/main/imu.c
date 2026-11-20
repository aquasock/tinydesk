/*
 * imu.c - the Bosch BHI260AP "smart sensor" (BHI260AP_SLAVE_ADDRESS_L, 0x28).
 *
 * WHAT THIS PART IS. Not an accelerometer with a register map: a Fuser core that
 * runs Bosch algorithms -- orientation, step counting, gestures, wake-on-motion
 * -- and reports results through a FIFO. Talking to it means uploading ~121 KB
 * of Bosch firmware into its RAM, booting it, then configuring virtual sensors
 * and draining samples. The protocol is Bosch's BHY2 SDK (components/bhy2,
 * BSD-3-Clause); this file is the port's side of it.
 *
 * THE RAIL WAS THE WHOLE REASON IT WAS INVISIBLE. ALDO4 feeds the sensor at
 * 1.8 V and nothing in this port had ever enabled it, so the part did not answer
 * a bus scan at all -- an unpowered I2C device is absent rather than faulty. See
 * board_imu_rail_on() in power.c. It is raised at bring-up, so the part is on
 * the bus from boot; what waits for first use is the firmware upload below.
 *
 * WHY THE FIRMWARE IS LAZY, AND WHERE IT LIVES. 121 KB is affordable in flash
 * but it is 12 percent of the app slot that was just freed to 1005 KB, and an
 * upload costs a second or two of start-up. So the image is NOT in the app: it
 * is a blob in the `moddata` partition at IMU_FW_OFFSET, streamed to the part
 * with esp_partition_read() when the IMU is first used. That keeps the app image
 * lean and makes the firmware independently updatable -- the same trade the
 * codec already makes in that partition.
 *
 * THE OFFSET MATTERS. The FLAC codec's .rodata is written at the very start of
 * `moddata` (1,976 bytes), so this takes 0x10000: `make flac` and `make imu` can
 * be flashed in either order and neither overwrites the other.
 *
 * SAMPLES ARRIVE THROUGH A CALLBACK, because that is how BHY2 is shaped:
 * bhy2_get_and_process_fifo() drains the part's FIFO and dispatches each packet
 * to whatever callback was registered for that sensor id. So the reading is
 * stashed by the callback and read out by imu_accel().
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "tdsh.h"

#include "bhy2.h"
#include "bhy2_defs.h"
#include "bhy2_hif.h"
#include "bhy2_klio.h"
#include "bhy2_klio_defs.h"
#include "tinydesk/td_sysinfo.h"
#include "imu.h"
#include "power.h"

static const char *TAG = "imu";

/* DEFINED BY klio.c, WHICH THIS FILE MUST NOT DEPEND ON.
 *
 * The hook exists because the part's RAM does not survive a boot: klio.c keeps
 * the learnt patterns in flash and puts them back here. The weak empty default
 * means a build without klio.c links and boots exactly as before.
 *
 * This is the same pattern ports/esp_idf/app/main.c uses for board_imu_ops(). */
__attribute__((weak)) void klio_after_boot(void) {}

/* ------------------------------------------------------------ the part */

#define IMU_ADDR       0x28     /* BHI260AP_SLAVE_ADDRESS_L: HSDO low */
#define IMU_HZ         400000   /* the vendor runs this bus at 400 kHz to 1 MHz */
#define IMU_CHIP_ID    0x70     /* BHI260_CHIP_ID, at BHY2_REG_CHIP_ID */

/* Where the firmware blob sits inside `moddata`. */
#define IMU_FW_LABEL   "moddata"
#define IMU_FW_OFFSET  0x10000
#define IMU_FW_MAGIC   0x494D5546u   /* "FUMI" little-endian */

/* The piece the firmware is read from flash and written to the part in. From
 * PSRAM, for the duration of the upload only; 4 KB is also the flash sector
 * size, so a read never straddles a page boundary in an interesting way. */
#define IMU_FW_CHUNK   4096

/* How much FIFO to drain at once, AND THIS BEING TOO SMALL IS A HANG.
 *
 * The SDK's drain loop is: while (remain_length) { read(buffer_size - read_length);
 * parse; }. When the part holds more than the buffer, `buffer_size - read_length`
 * reaches 0, the read returns 0 bytes, remain_length does not change, and the
 * loop never terminates -- with the shell task inside it, so Ctrl+C cannot
 * reach it either.
 *
 * 512 bytes is 13 bytes/sample x 39 samples: less than half a second at 100 Hz.
 * Any polling gap longer than that wedges the board. 2048 bytes is 1.5 s of
 * headroom, and the drain loop below empties the FIFO in bounded passes as a
 * second guard. */
#define IMU_FIFO_MAX   2048
#define IMU_DRAIN_PASSES 4

typedef struct
{
    uint32_t magic;
    uint32_t size;
    uint32_t check;      /* low 32 bits of the payload's SHA-256 */
} imu_fw_header_t;

/* ------------------------------------------------------------ the state */

static struct bhy2_dev s_bhy2;
static i2c_master_dev_handle_t s_dev;
static bool s_ready;                 /* firmware uploaded and booted */
static bool s_sensor_on;
static bool s_ori_on;
static volatile bool s_have_ori;
static float s_heading, s_pitch, s_roll;
static char s_why[112];
static char s_version[40];
static uint8_t s_fifo[IMU_FIFO_MAX];

/* The most recent acceleration, written by the callback and read by
 * imu_accel(). Single producer (this task) and single consumer, so a plain
 * volatile pair of flags is enough; there is no second task polling the part. */
/* The last raw payload the parser handed to on_accel, for instruments that need
 * to see the bytes rather than the interpretation. */
static uint8_t s_accel_raw[8];
static volatile uint8_t s_accel_raw_len;

/* The first few payloads verbatim, with their reported sizes, so an off-by-one
 * in the event size shows up as a sequence rather than a single sample. */
static uint8_t s_first[5][9];
/* Parsed timestamps from consecutive physical accelerometer samples, used to
 * derive the part's timestamp unit from a source that injection cannot touch. */
static uint64_t s_accel_ts[12];
static volatile uint8_t s_accel_ts_n;
static volatile uint8_t s_first_len[5];
static volatile int s_first_n;

static volatile bool s_have_accel;
static float s_ax, s_ay, s_az;
static uint32_t s_accel_count;

/* THE GYROSCOPE, RAW, so a gesture can be recorded and replayed.
 *
 * Until now nothing in this port kept gyro values: the algorithm got them
 * straight from the FIFO and only the accelerometer was decoded into floats for
 * the windows. Recording a gesture needs all six numbers, and it needs them as
 * the RAW int16 the part produced, because replay has to inject exactly what the
 * part reported rather than a value that has been through a float round trip. */
static volatile bool s_have_gyro;
static volatile int16_t s_gx_raw, s_gy_raw, s_gz_raw;
static volatile uint32_t s_gyro_count;
/* The accelerometer's raw triple, kept beside its float form because a recorded
 * gesture must replay the part's own integers, not a value that has been through
 * a float conversion and back. */
static volatile int16_t s_ax_raw, s_ay_raw, s_az_raw;


/* The sensor ids the loaded firmware carries, as a compact "3 61 63 ..." string.
 * Built once and kept, because it is the answer to "why did nothing happen" and
 * asking again costs 255 availability queries. */
static char s_present[160];
static int s_present_n = -1;

/* Declared here because the firmware bring-up below registers it, and its body
 * lives with the other FIFO callbacks further down. */
static void on_meta(const struct bhy2_fifo_parse_data_info *info, void *priv);
static void on_ori(const struct bhy2_fifo_parse_data_info *info, void *priv);

/* Fills "3 61 63 ..." with the virtual sensors the loaded firmware carries. */
static int imu_list_sensors(char *out, size_t cap);

/* ------------------------------------------------------ the BHY2 hooks */

/* BHY2 wants four things from the port: read, write, a microsecond delay, and a
 * length limit. Everything else in the SDK is protocol. */
static BHY2_INTF_RET_TYPE bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf)
{
    (void)intf;
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100) == ESP_OK
               ? BHY2_INTF_RET_SUCCESS
               : BHY2_E_IO;
}

/* THE WRITE BUFFER HAS TO BE AS LARGE AS THE LARGEST HIF WRITE, and that is
 * not a control register. The firmware upload ends up as
 * bhy2_hif_set_regs(BHY2_REG_CHAN_CMD, chunk, 4096) -- ONE 4,097-byte I2C
 * transaction -- so a small stack buffer refuses it, and the SDK turns any
 * non-zero return from here into BHY2_E_IO (-3), which reads as a bus fault
 * rather than as "your buffer is too small". That cost one cycle: the upload
 * succeeded for the first chunk (which goes through exec_cmd_generic) and then
 * failed at exactly 4096.
 *
 * So one scratch buffer lives for the session, from PSRAM because 4 KB of the
 * internal heap is not spare, and every write is assembled in it: the register
 * byte then the payload, as one transaction. */
static uint8_t *s_wbuf;
static size_t s_wbuf_len;

static BHY2_INTF_RET_TYPE bus_write(uint8_t reg, const uint8_t *data, uint32_t len,
                                    void *intf)
{
    (void)intf;
    if (!s_wbuf)
        return BHY2_E_INVALID_PARAM;
    if (len + 1 > s_wbuf_len)
    {
        ESP_LOGE(TAG, "write of %u exceeds the %u-byte buffer",
                 (unsigned)len, (unsigned)s_wbuf_len);
        return BHY2_E_INVALID_PARAM;
    }
    s_wbuf[0] = reg;
    if (len)
        memcpy(s_wbuf + 1, data, len);
    return i2c_master_transmit(s_dev, s_wbuf, len + 1, 200) == ESP_OK
               ? BHY2_INTF_RET_SUCCESS
               : BHY2_E_IO;
}

static void bus_delay_us(uint32_t us, void *intf)
{
    (void)intf;
    /* A BUSY-WAIT BELOW THE TICK, NOT A vTaskDelay. BHY2 asks for microsecond
     * delays inside its handshakes -- the FIFO read and the boot-status poll
     * both loop on one -- and rounding those up to a FreeRTOS tick turned a
     * millisecond-scale handshake into something that never finished: the shell
     * task blocked inside the first FIFO read and stopped answering Ctrl+C.
     * Above the tick, yield properly; below it, spin. */
    if (us >= 1000)
        vTaskDelay(pdMS_TO_TICKS(us / 1000));
    else if (us > 0)
        esp_rom_delay_us(us);
}

/* ------------------------------------------------------ firmware upload */

/* Locate and validate the blob. The firmware is NOT loaded into RAM: the SDK
 * takes a chunk at a time, so it is streamed from flash to the part. */
static bool fw_open(const esp_partition_t **part, size_t *size)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                        ESP_PARTITION_SUBTYPE_ANY,
                                                        IMU_FW_LABEL);
    if (!p)
    {
        snprintf(s_why, sizeof(s_why), "no '%s' partition", IMU_FW_LABEL);
        return false;
    }

    imu_fw_header_t h;
    if (esp_partition_read(p, IMU_FW_OFFSET, &h, sizeof(h)) != ESP_OK)
    {
        snprintf(s_why, sizeof(s_why), "cannot read the blob header");
        return false;
    }
    if (h.magic != IMU_FW_MAGIC)
    {
        /* The commonest cause by far is that `make imu` was never flashed: the
         * partition is mostly 0xFF and an erased word is not the magic. */
        snprintf(s_why, sizeof(s_why),
                 "no IMU firmware at %s+0x%x (magic %08lx) -- run `make imu`",
                 IMU_FW_LABEL, (unsigned)IMU_FW_OFFSET, (unsigned long)h.magic);
        return false;
    }
    if (h.size == 0 || h.size > p->size - IMU_FW_OFFSET - sizeof(h))
    {
        snprintf(s_why, sizeof(s_why), "blob size %lu is out of range",
                 (unsigned long)h.size);
        return false;
    }

    *part = p;
    *size = h.size;
    return true;
}

/* Upload the firmware and boot it.
 *
 * THE WHOLE IMAGE GOES THROUGH THE SDK'S OWN UPLOADER, and that was not the
 * first attempt. Hand-rolling a loop over
 * bhy2_upload_firmware_to_ram_partly() uploaded 123,696 bytes successfully and
 * then failed to boot with the part's HOST_FW_VERIFY_ERROR bit set -- boot status
 * 0x78 -- because the chunk offsets this side passed were not what the protocol
 * wanted. The SDK's whole-image call sequences the command and the data
 * correctly; the cost is that the image has to be resident, which on this board
 * means 121 KB of PSRAM for the duration of the upload. That is cheap: 7.7 MB is
 * free, and it is freed before the function returns.
 *
 * The vendor (SensorLib, BoschSensorBase::uploadFirmware) does exactly this, and
 * also sets the async-status-channel HIF control first, which is reproduced
 * below because the SDK's parser expects status packets on that channel. */
static bool fw_upload_and_boot(void)
{
    const esp_partition_t *part = NULL;
    size_t size = 0;

    if (!fw_open(&part, &size))
        return false;

    /* Resident for the upload only. PSRAM first; the internal heap could not
     * spare 121 KB on this board and would not be asked to. */
    uint8_t *img = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!img)
        img = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    if (!img)
    {
        snprintf(s_why, sizeof(s_why), "no %u-byte buffer for the firmware",
                 (unsigned)size);
        return false;
    }

    esp_err_t r = esp_partition_read(part, IMU_FW_OFFSET + sizeof(imu_fw_header_t),
                                     img, size);
    if (r != ESP_OK)
    {
        heap_caps_free(img);
        snprintf(s_why, sizeof(s_why), "flash read failed (%d)", (int)r);
        return false;
    }

    ESP_LOGI(TAG, "uploading %u bytes of firmware", (unsigned)size);
    const int64_t t0 = esp_timer_get_time();

    int8_t rc = bhy2_upload_firmware_to_ram(img, (uint32_t)size, NULL, NULL, &s_bhy2);
    heap_caps_free(img);

    if (rc != BHY2_OK)
    {
        snprintf(s_why, sizeof(s_why), "upload refused (%d)", rc);
        return false;
    }

    rc = bhy2_boot_from_ram(&s_bhy2);
    if (rc != BHY2_OK)
    {
        /* The part reports WHY in its boot status, and a timeout alone does not
         * say. 0x40 there is a verify error, i.e. the image arrived corrupt --
         * which is a different bug from 0x00, where it never started. */
        uint8_t st = 0;
        (void)bhy2_get_boot_status(&st, &s_bhy2);
        snprintf(s_why, sizeof(s_why), "boot_from_ram failed (%d), status %02x",
                 rc, st);
        return false;
    }

    ESP_LOGI(TAG, "booted in %d ms", (int)((esp_timer_get_time() - t0) / 1000));

    /* THE HIF CONTROL GOES HERE, BEFORE ANY VIRTUAL SENSOR -- and it has to be
     * AFTER the boot, which is where the vendor puts it
     * (SensorLib BoschSensorBase, immediately after uploadFirmware and before
     * getKernelVersion).
     *
     * WHY IT MATTERS: status packets are what BHY2's
     * bhy2_hif_wait_status_ready() polls for, and it retries
     * BHY2_QUERY_PARAM_STATUS_READY_MAX_RETRY (1000) times at 10 ms -- ten
     * seconds per command -- before giving up. Without the async status channel
     * selected, those status bits never arrive, so every command that waits for
     * one burns its full ten seconds. Setting this first is the difference
     * between a sensor configuration that takes milliseconds and one that
     * appears to hang.
     *
     * The vendor also reads the control back and complains when the bit did not
     * take, which is the check reproduced below. */
    rc = bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &s_bhy2);
    if (rc != BHY2_OK)
    {
        snprintf(s_why, sizeof(s_why), "set_host_intf_ctrl failed (%d)", rc);
        return false;
    }
    {
        uint8_t ctrl = 0;
        if (bhy2_get_host_intf_ctrl(&ctrl, &s_bhy2) == BHY2_OK)
        {
            ESP_LOGI(TAG, "host intf ctrl %02x (async status %s)", ctrl,
                     (ctrl & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL) ? "on" : "OFF");
            if (!(ctrl & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL))
                ESP_LOGW(TAG, "async status channel did not take");
        }
    }

    /* Listen for the part's own verdicts before asking it to do anything.
     *
     * TWO ids, because meta events arrive on either the normal (254) or the
     * wakeup (248) channel depending on the event, and the vendor registers
     * both -- SensorLib, BoschSensorBase, does exactly this pair. Registering
     * only the first is why a rejected configuration can look like silence. */
    (void)bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, on_meta, NULL,
                                            &s_bhy2);
    (void)bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, on_meta,
                                            NULL, &s_bhy2);

    /* POPULATE THE SENSOR BITMAP NOW, ONCE. bhy2_is_sensor_available() reads a
     * bitmap that bhy2_get_virt_sensor_list() fills; asking before that reports
     * "no sensors at all", which reads as a firmware that cannot do the job. It
     * cost a build cycle spent chasing the wrong thing -- the accelerometer was
     * there the whole time. */
    s_present_n = imu_list_sensors(s_present, sizeof(s_present));
    ESP_LOGI(TAG, "%d virtual sensors: %s", s_present_n, s_present);
    return true;
}

/* ---------------------------------------------------------- the callback */

/* BHY2 hands each FIFO packet here, already decoded far enough to say which
 * sensor it belongs to. An accelerometer packet is three little-endian int16 in
 * 1/4096 g, followed by a timestamp the SDK appends. */
static void on_accel(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 6)
        return;
    /* A CLEAN CLOCK SAMPLE: the physical accelerometer, no injection involved.
     * The part's timestamp unit is not documented in this SDK, and measuring it
     * during an injected run is worthless because the injected deltas are
     * themselves what is in question. Here the only contribution is the part's
     * own timekeeping, so consecutive deltas at a known sample rate give the
     * unit directly. */
    if (info->time_stamp && s_accel_ts_n < 12)
        s_accel_ts[s_accel_ts_n++] = *info->time_stamp;
    int16_t s[3];
    memcpy(s, info->data_ptr, 6);
    memcpy(s_accel_raw, info->data_ptr,
           info->data_size < sizeof(s_accel_raw) ? info->data_size
                                                 : sizeof(s_accel_raw));
    s_accel_raw_len = info->data_size < sizeof(s_accel_raw)
                          ? (uint8_t)info->data_size : (uint8_t)sizeof(s_accel_raw);
    if (s_first_n < 5)
    {
        const int k = s_first_n++;
        uint8_t n2 = info->data_size < 9 ? (uint8_t)info->data_size : 9;
        s_first_len[k] = n2;
        memcpy(s_first[k], info->data_ptr, n2);
    }
    s_ax = s[0] / 4096.0f;
    s_ay = s[1] / 4096.0f;
    s_az = s[2] / 4096.0f;
    s_accel_count++;
    s_have_accel = true;
    s_ax_raw = s[0];
    s_ay_raw = s[1];
    s_az_raw = s[2];
}

/* THE GYROSCOPE SINK, which nothing in this port had until a gesture needed to
 * be recorded. Raw int16 in 16.4 LSB per degree per second, exactly as the part
 * reports it, so a replay can inject the same numbers back. */
static void on_gyro(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 6)
        return;
    int16_t s[3];
    memcpy(s, info->data_ptr, 6);
    s_gx_raw = s[0];
    s_gy_raw = s[1];
    s_gz_raw = s[2];
    s_gyro_count++;
    s_have_gyro = true;
}

/* ORIENTATION, WHICH IS THE INTERESTING ONE ON A WRIST.
 *
 * BHY2_SENSOR_ID_DEVICE_ORI reports quaternion and Euler angles computed ON THE
 * PART by Bosch's fusion algorithm -- work this board cannot afford to do in
 * its 26 KB of internal RAM. The payload is quaternion (4 x int16, scaled by
 * 2^14), then heading, pitch and roll in degrees as int16 scaled by 2^8.
 *
 * Sensor 69 (DEVICE_ORI) only reports on change; 70 (DEVICE_ORI_WU) is the
 * wakeup variant. This subscribes to 69, which is what a window showing "which
 * way am I facing" wants. */
static void on_ori(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 14)      /* 8 quaternion + 6 euler */
        return;
    int16_t v[7];
    memcpy(v, info->data_ptr, 14);
    /* v[0..3] quaternion, v[4] heading, v[5] pitch, v[6] roll. */
    s_heading = v[4] / 256.0f;
    s_pitch = v[5] / 256.0f;
    s_roll = v[6] / 256.0f;
    s_have_ori = true;
}

/* META EVENTS ARE THE PART'S ONLY WAY OF SAYING NO. Every BHY2 command comes
 * back as "accepted"; whether the part then honours it arrives later as a meta
 * event on the same FIFO, and a rejected configuration looks exactly like an
 * accepted one unless something listens. Registering this is what turned "cfg
 * returned 0 and then nothing ever happened" into a readable answer. */
/* META EVENTS ARE PART OF THE MEASUREMENT, NOT JUST THE LOG.
 *
 * `meta 12 (fifo overflow)` and `meta 11 (SENSOR ERROR)` are the part's own
 * verdicts on whether the engine is being starved, and until now they went only
 * to ESP_LOGW -- which the display mirror garbles and which is compiled out on
 * the image being measured. Entry 73's diagnosis of a FIFO overflow was an
 * inference from a reason byte, not a count of overflows.
 *
 * These are counters, deliberately: no I/O from a FIFO callback, so instrumenting
 * the run cannot change it. The inject command writes them into its trace. */
static volatile uint32_t s_meta_total;
static volatile uint32_t s_meta_overflow;
static volatile uint32_t s_meta_sensor_err;
static volatile uint8_t  s_meta_last_ev;
static volatile uint8_t  s_meta_last_sensor;

/* WHAT THE FOUR THOUSAND META EVENTS ACTUALLY ARE.
 *
 * The first instrumented run counted 4135 meta events in 30 seconds and no
 * overflow, and recorded a last event code of 0 -- which is not in the SDK's
 * vocabulary at all (bhy2_defs.h defines 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13,
 * 14, 15, 16, 18, 19, 20). A total with no distribution is a number that invites
 * a guess, so the next run carries the distribution and the packet sizes.
 *
 * IN PSRAM, NOT .bss, AND THAT IS NOT FUSSINESS. The first version was a static
 * array and it cost `music` its I2S slot: `state error, 0 frames, 0 errors, 0
 * short writes: slot: ESP_ERR_NO_MEM`, on a module that loaded fine (magic
 * 464c4143, rate 44100) and decoded six times. A hundred and twenty-eight bytes
 * of .bss was enough to cross the line, which is EMU-018's rule arriving again.
 * The array is therefore allocated by the inject command that needs it and is
 * NULL the rest of the time, so on_meta tolerates NULL rather than requiring it. */
static volatile uint32_t *s_meta_hist;
static volatile uint8_t  s_meta_max_size;

/* THE FIRST FEW META PACKETS, VERBATIM.
 *
 * The histogram above reports codes 0 and 31 in the thousands and 31 is not in
 * bhy2_defs.h's vocabulary at all, with the sensor byte always 0. Either the part
 * emits meta packets this SDK does not describe, or this callback is being handed
 * something that is not a meta packet -- and the answer decides whether the
 * overflow count means anything. Recording the bytes is the only way to tell,
 * exactly as it was for the KLIO tap. */
static uint8_t s_meta_raw[4][12];
static volatile uint8_t s_meta_raw_len[4];
static volatile uint8_t s_meta_raw_n;

static void on_meta(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 1)
        return;
    const uint8_t ev = info->data_ptr[0];
    const uint8_t sensor = info->data_size > 1 ? info->data_ptr[1] : 0;

    s_meta_total++;
    s_meta_last_ev = ev;
    s_meta_last_sensor = sensor;
    if (s_meta_raw_n < 4)
    {
        const uint8_t k = s_meta_raw_n++;
        const uint8_t n = info->data_size < 12 ? info->data_size : 12;
        s_meta_raw_len[k] = n;
        memcpy(s_meta_raw[k], info->data_ptr, n);
    }
    if (s_meta_hist)
        s_meta_hist[ev < 32 ? ev : 31]++;
    if (info->data_size > s_meta_max_size)
        s_meta_max_size = info->data_size;
    if (ev == BHY2_META_EVENT_FIFO_OVERFLOW)
        s_meta_overflow++;
    else if (ev == BHY2_META_EVENT_SENSOR_ERROR)
        s_meta_sensor_err++;
    const char *name = "?";
    switch (ev)
    {
    case BHY2_META_EVENT_FLUSH_COMPLETE:        name = "flush complete"; break;
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:   name = "sample rate changed"; break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:    name = "power mode changed"; break;
    case BHY2_META_EVENT_SENSOR_STATUS:         name = "sensor status"; break;
    case BHY2_META_EVENT_SENSOR_ERROR:          name = "SENSOR ERROR"; break;
    case BHY2_META_EVENT_FIFO_OVERFLOW:         name = "fifo overflow"; break;
    case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED: name = "range changed"; break;
    case BHY2_META_EVENT_FIFO_WATERMARK:        name = "fifo watermark"; break;
    case BHY2_META_EVENT_INITIALIZED:           name = "initialised"; break;
    case BHY2_META_EVENT_RESET:                 name = "reset"; break;
    case BHY2_META_EVENT_ALGORITHM_EVENTS:      name = "algorithm event"; break;
    default: break;
    }
    /* Routine events (sample rate changed, power mode, the initial handshake)
     * are DEBUG: they happen once per configuration and then never again, so
     * putting them at INFO fills the console of anyone who runs the command.
     * The ones worth seeing by default are the failures. */
    const bool loud = ev == BHY2_META_EVENT_SENSOR_ERROR ||
                      ev == BHY2_META_EVENT_FIFO_OVERFLOW;
    if (loud)
        ESP_LOGW(TAG, "meta %u (%s) sensor %u", ev, name, sensor);
    else
        ESP_LOGD(TAG, "meta %u (%s) sensor %u", ev, name, sensor);
}

/* ---------------------------------------------------------- start/stop */

struct bhy2_dev *imu_dev(void)
{
    return s_ready ? &s_bhy2 : NULL;
}

const char *imu_why(void)
{
    return s_why;
}

bool imu_ready(void)
{
    return s_ready;
}

/* Fill "3 61 63 ..." with every virtual sensor id the firmware carries. */
static int imu_list_sensors(char *out, size_t cap)
{
    size_t pos = 0;
    int n = 0;

    if (out && cap)
        out[0] = '\0';

    /* UPDATE, NOT JUST GET -- and the difference is the whole reason nothing
     * ever arrived.
     *
     * bhy2_get_virt_sensor_list() fills the presence bitmap, which is all that
     * bhy2_is_sensor_available() needs. bhy2_update_virtual_sensor_list() does
     * that AND queries each sensor's info to fill dev->event_size[], which is
     * how the FIFO parser knows how long a packet for a given sensor id is.
     * Without those sizes it cannot frame anything, so it dispatches nothing:
     * the sensor is configured, the part streams, the drain succeeds, and zero
     * samples reach the callbacks -- which is exactly what was observed for two
     * build cycles.
     *
     * The vendor calls both, in this order (SensorLib BoschSensorBase: update,
     * then get). Reproduced here. */
    (void)bhy2_update_virtual_sensor_list(&s_bhy2);

    for (int id = 1; id < 255; id++)
    {
        if (!bhy2_is_sensor_available((uint8_t)id, &s_bhy2))
            continue;
        n++;
        if (out && pos + 6 < cap)
            pos += (size_t)snprintf(out + pos, cap - pos, "%s%d", pos ? " " : "", id);
    }
    return n;
}

/* Bring the rail, the bus and the SDK up, and stop BEFORE any firmware is
 * uploaded. Split out of imu_start() so the boot diagnostic below can inspect a
 * part whose firmware was refused -- which is exactly the case that cannot be
 * inspected while a successful start is a precondition for every command. */
static bool imu_hw_open(void)
{
    if (!board_imu_rail_on())
    {
        snprintf(s_why, sizeof(s_why), "the sensor rail did not come up");
        return false;
    }

    if (!s_dev)
    {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = IMU_ADDR,
            .scl_speed_hz = IMU_HZ,
        };
        if (!board_i2c())
        {
            snprintf(s_why, sizeof(s_why), "no I2C bus");
            return false;
        }
        if (i2c_master_bus_add_device(board_i2c(), &cfg, &s_dev) != ESP_OK)
        {
            snprintf(s_why, sizeof(s_why), "no device at 0x%02X", IMU_ADDR);
            s_dev = NULL;
            return false;
        }
    }

    /* Identify it before uploading anything to it. */
    uint8_t id = 0;
    if (bus_read(BHY2_REG_CHIP_ID, &id, 1, NULL) != BHY2_INTF_RET_SUCCESS ||
        id != IMU_CHIP_ID)
    {
        snprintf(s_why, sizeof(s_why), "chip id %02x is not a BHI260 (%02x)",
                 id, IMU_CHIP_ID);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return false;
    }

    if (!s_wbuf)
    {
        /* One byte for the register address on top of the largest data write. */
        s_wbuf_len = IMU_FW_CHUNK + 1;
        s_wbuf = heap_caps_malloc(s_wbuf_len, MALLOC_CAP_SPIRAM);
        if (!s_wbuf)
            s_wbuf = heap_caps_malloc(s_wbuf_len, MALLOC_CAP_8BIT);
        if (!s_wbuf)
        {
            snprintf(s_why, sizeof(s_why), "no %u-byte write buffer",
                     (unsigned)s_wbuf_len);
            return false;
        }
    }

    memset(&s_bhy2, 0, sizeof(s_bhy2));
    /* read_write_len = 64, and it is load-bearing in BOTH directions. Zero
     * makes the firmware upload fail with BHY2_E_INVALID_PARAM (-2): the
     * uploader chunks the image by this value, and 0 means "no chunking".
     * SensorLib passes half of Arduino's I2C buffer here with the note that
     * ESP32 "has an issue with reading large data"; 64 is the same order and
     * is what uploads cleanly here. */
    int8_t rc = bhy2_init(BHY2_I2C_INTERFACE, bus_read, bus_write, bus_delay_us,
                          64, NULL, &s_bhy2);
    if (rc != BHY2_OK)
    {
        snprintf(s_why, sizeof(s_why), "bhy2_init failed (%d)", rc);
        return false;
    }

    /* A soft reset first, as the vendor does. Without it a part left in a
     * half-finished state by an earlier attempt stays there: this was found with
     * a stale boot-attempt sequence, and the reset is cheap and idempotent. */
    rc = bhy2_soft_reset(&s_bhy2);
    if (rc != BHY2_OK)
    {
        snprintf(s_why, sizeof(s_why), "soft reset failed (%d)", rc);
        return false;
    }

    return true;
}

bool imu_start(void)
{
    if (s_ready)
        return true;
    s_why[0] = '\0';

    if (!imu_hw_open())
        return false;

    if (!fw_upload_and_boot())
        return false;

    s_ready = true;
    imu_version(s_version, sizeof(s_version));
    ESP_LOGI(TAG, "BHI260AP up: %s", s_version);

    /* PUT BACK WHAT DOES NOT SURVIVE A BOOT. The firmware above was just
     * uploaded into the part's RAM, which means the engine's pattern table is
     * empty however many gestures were learnt last session. klio.c restores its
     * stored patterns here, and is written so that a missing or corrupt store
     * leaves the sensor perfectly usable -- persistence may not be able to stop
     * the IMU from starting. */
    klio_after_boot();
    return true;
}

/* ------------------------------------------------------- boot diagnostic */

/* WHAT THE PART SAYS ABOUT ITS OWN BOOT, in words.
 *
 * The upload path reports only a timeout when the part never raises
 * HOST_FW_VERIFY_DONE, and a timeout does not distinguish "the image arrived and
 * failed verification" from "the image never got there". The boot status byte
 * does, and so does the error register.
 *
 * This exists because the question being asked is whether Bosch's DATA INJECTION
 * firmware -- which they ship for the BHI360 (chip id 0x7A) and not for our
 * BHI260 (0x70) -- will boot here. Chips can share a firmware container format
 * and still refuse the payload, and the refusal is recorded in these two bytes.
 */
static void boot_status_report(const char *when)
{
    uint8_t st = 0, err = 0, chip = 0;

    (void)bhy2_get_regs(BHY2_REG_CHIP_ID, &chip, 1, &s_bhy2);
    if (bhy2_get_boot_status(&st, &s_bhy2) != BHY2_OK)
    {
        printf("  %-18s boot status unreadable\n", when);
        return;
    }
    (void)bhy2_get_error_value(&err, &s_bhy2);

    printf("  %-18s chip %02x  boot %02x  err %02x  %s%s%s%s%s\n", when, chip, st,
           err,
           (st & BHY2_BST_HOST_INTERFACE_READY) ? "iface-ready " : "",
           (st & BHY2_BST_HOST_FW_VERIFY_DONE) ? "verify-done " : "",
           (st & BHY2_BST_HOST_FW_VERIFY_ERROR) ? "VERIFY-ERROR " : "",
           (st & BHY2_BST_NO_FLASH) ? "no-flash " : "",
           (st & BHY2_BST_HOST_FW_IDLE) ? "idle " : "");
}

/* ---------------------------------------------------------- sine feeder --- */

/* FEED THE PART A SINE THE HOST GENERATED, using Bosch's own injection
 * sequence rather than a guess.
 *
 * The protocol comes from their `examples/swim/swim_data_injection` in
 * BHI2xy_SensorAPI, and every step of it was missing from the earlier attempts:
 *
 *   1  bhy2_set_data_injection_mode(STEP_BY_STEP)
 *   2  bhy2_set_virt_sensor_cfg(SENSOR_ID, rate, latency)   <-- CONFIGURE FIRST
 *   3  drain the status FIFO
 *   4  bhy2_inject_data(timestamp, 6)      a full timestamp, its OWN call
 *   5  bhy2_inject_data(event, 7)          then one FIFO event
 *   6  drain the FIFO between injections
 *
 * STEP 2 IS THE ONE THAT MATTERED. Injection supplies data for a virtual sensor;
 * if no sensor is configured then nothing is listening and the bytes are
 * accepted and dropped, which is exactly what every earlier result looked like.
 * A 6-byte full timestamp also precedes the data as a separate injection --
 * the part will not place an event in time without it.
 *
 * THE TEST MEASURES THE ROUND TRIP. Every sample is generated from the sine,
 * injected, drained back through the part's own parser, and compared against
 * what the callback saw. If the returned range matches the injected amplitude
 * then the part really is consuming host data, which is the whole premise.
 */
static int cmd_imu_feed(int seconds, float hz, float amp, int mode, int ev_id)
{
    if (!s_ready && !imu_start())
    {
        printf("imu: %s\n", s_why);
        return 1;
    }

    printf("feeding a %.2f Hz sine, %.2f g amplitude, 50 samples/s for %d s,"
           " event id %d\n", (double)hz, (double)amp, seconds, ev_id);
    printf("injected values are round-tripped through the part's own parser.\n\n");

    /* 1. the injection mode, and confirm it really changed like Bosch does.
     * The two modes are not interchangeable and which one substitutes values is
     * an open question, so it is an argument rather than a constant. */
    const int inj = (mode == 1) ? BHY2_REAL_TIME_INJECTION
                                : BHY2_STEP_BY_STEP_INJECTION;
    if (!imu_inject_begin(inj))
    {
        printf("imu: the part refused injection mode %d\n", inj);
        return 1;
    }
    printf("  injection mode: %s\n",
           inj == BHY2_REAL_TIME_INJECTION ? "real-time" : "step-by-step");

    /* 2. CONFIGURE THE SENSOR. This is the step whose absence made injection
     * look dead: the part needs to know a virtual sensor exists before it will
     * accept data for one. */
    const int8_t cfg = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 50.0f, 20,
                                                &s_bhy2);
    printf("  configure accel @50 Hz: %d\n", cfg);
    printf("  event_size: acc_raw %u  acc %u  acc_wu %u  (id + payload)\n",
           (unsigned)s_bhy2.event_size[BHY2_SENSOR_ID_ACC_RAW],
           (unsigned)s_bhy2.event_size[BHY2_SENSOR_ID_ACC],
           (unsigned)s_bhy2.event_size[BHY2_SENSOR_ID_ACC_WU]);
    (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, on_accel, NULL,
                                            &s_bhy2);
    s_sensor_on = true;      /* so imu_poll() drains */

    /* 3. DRAIN, THEN FLUSH. The part boots with a backlog already queued (a
     * measured 251 bytes on the DataInject image), and leaving it there makes
     * the round trip below impossible to interpret -- a sample read back could
     * be a leftover rather than an injected one. Bosch's own example drains the
     * status FIFO before injecting for the same reason; flushing the data FIFO
     * is the stronger version and is what makes "0 samples arrived" mean
     * something. */
    for (int i = 0; i < 10; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)imu_poll();
    }
    const int8_t flushed = bhy2_clear_fifo(0xFF, &s_bhy2);
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)imu_poll();
    printf("  flush fifo: %d, %lu samples before\n", flushed,
           (unsigned long)s_accel_count);

    s_have_accel = false;
    const uint32_t before = s_accel_count;

    float lo = 99.0f, hi = -99.0f;
    uint32_t sent = 0, seen = 0;
    const int step_ms = 20;                    /* 50 samples a second */
    const int steps = seconds * 1000 / step_ms;
    uint32_t ts = 0;

    for (int i = 0; i < steps; i++)
    {
        const float t = (float)i * step_ms / 1000.0f;
        const float z = amp * sinf(2.0f * (float)M_PI * hz * t);
        const int16_t zq = (int16_t)(z * 4096.0f);
        const int16_t xq = 0, yq = 0;

        /* 4. the full timestamp, as its own injection. 0xFD is TS_FULL on the
         * non-wakeup channel; five bytes carry the 40-bit value. */
        uint8_t stamp[6];
        stamp[0] = BHY2_SYS_ID_TS_FULL;
        stamp[1] = (uint8_t)(ts & 0xFF);
        stamp[2] = (uint8_t)((ts >> 8) & 0xFF);
        stamp[3] = (uint8_t)((ts >> 16) & 0xFF);
        stamp[4] = (uint8_t)((ts >> 24) & 0xFF);
        stamp[5] = 0;
        ts += (uint32_t)(step_ms * 1000);      /* microseconds */

        /* 5. ONE EVENT, WITH THE LENGTH THE PART ITSELF DECLARES.
         *
         * A BHy2 accelerometer event is the sensor id plus three int16, which is
         * 7 bytes -- but on the DataInject image the part reports
         * `event_size[4] == 7`, i.e. id + SEVEN. Injecting id + 6 is one byte
         * short of the descriptor, and the part then emits events of its own
         * choosing rather than the ones supplied: 145 to 299 of them arrived
         * back for 150 to 300 injections, and every payload decoded as
         * `00 08 00 01 00 00 00`, which is not an acceleration at all.
         *
         * So the length comes from the part, not from what BHy2 usually means
         * by an accelerometer. If it says 7, seven bytes go in. */
        uint8_t ev[9] = {0};
        const uint8_t payload = s_bhy2.event_size[BHY2_SENSOR_ID_ACC]
                                    ? (uint8_t)(s_bhy2.event_size[BHY2_SENSOR_ID_ACC] - 1)
                                    : 6;
        /* THE EVENT ID IS AN ARGUMENT BECAUSE THERE ARE TWO CANDIDATES AND NO
         * DOCUMENTATION SETTLES IT. The virtual sensor is 4; Bosch's
         * dinject_defs.h names ACCEL_INJECT_ID 55 and ACC_COR_INJECT_DRIVER_ID
         * 76, and on the DataInject image those are plausibly what the handler
         * expects even though the OUTPUT is still reported against sensor 4. */
        ev[0] = (uint8_t)ev_id;
        memcpy(ev + 1, &xq, 2);
        memcpy(ev + 3, &yq, 2);
        memcpy(ev + 5, &zq, 2);
        const uint16_t evlen = (uint16_t)(1 + payload);

        if (imu_inject_frame(stamp, sizeof(stamp)) &&
            imu_inject_frame(ev, evlen))
            sent++;

        /* 6. drain so the parser sees it. */
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        (void)imu_poll();

        float ax, ay, az;
        uint32_t n;
        if (imu_accel(&ax, &ay, &az, &n))
        {
            seen = n - before;
            if (az < lo) lo = az;
            if (az > hi) hi = az;
        }
    }

    imu_inject_end();
    s_sensor_on = false;

    const uint32_t got = s_accel_count - before;
    printf("\n%lu samples injected, %lu arrived back\n", (unsigned long)sent,
           (unsigned long)got);
    if (got == 0)
    {
        printf("range: none\n");
        printf("\nNOTHING CAME BACK. The part accepted the packets and produced no\n"
               "events, so either the framing is still wrong or this firmware's\n"
               "DataInject paths want a different sensor id.\n");
        return 1;
    }

    printf("z range seen: %+.2f .. %+.2f g   (injected peak %+.2f)\n",
           (double)lo, (double)hi, (double)amp);
    for (int k = 0; k < s_first_n; k++)
    {
        printf("first[%d] %u B:", k, (unsigned)s_first_len[k]);
        for (uint8_t i = 0; i < s_first_len[k]; i++)
            printf(" %02x", s_first[k][i]);
        printf("\n");
    }
    printf("%s\n", (hi > 0.5f * amp && lo < -0.5f * amp)
                   ? "THE SINE ARRIVED. Host-generated data is being consumed by\n"
                     "the part and read back out of its own FIFO."
                   : "Events arrived but the range does not match the injections.");
    return 0;
}

static int cmd_imu_boot(bool upload)
{
    s_ready = false;
    s_why[0] = '\0';

    printf("part bring-up, no firmware:\n");
    if (!imu_hw_open())
    {
        printf("  failed: %s\n", s_why);
        return 1;
    }
    boot_status_report("after soft reset");

    if (!upload)
    {
        printf("\n  `imu boot upload` sends whatever is at %s+0x%x\n",
               IMU_FW_LABEL, (unsigned)IMU_FW_OFFSET);
        return 0;
    }

    printf("\nuploading %s+0x%x:\n", IMU_FW_LABEL, (unsigned)IMU_FW_OFFSET);
    const bool ok = fw_upload_and_boot();
    printf("  result: %s\n", ok ? "BOOTED" : s_why);
    boot_status_report(ok ? "after boot" : "after failure");

    if (!ok)
        printf("\n  VERIFY-ERROR means the image arrived and was rejected.\n"
               "  A clean status that never sets verify-done means the part\n"
               "  stopped before checking, i.e. wrong image for this silicon.\n");

    s_ready = ok;
    return ok ? 0 : 1;
}

void imu_stop(void)
{
    /* Deliberately does NOT power the rail down and does not unload the
     * firmware: the upload is the expensive part and the part idles at
     * microamps. Stopping only turns the sensor off. */
    if (s_ready && s_sensor_on)
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 0.0f, 0, &s_bhy2);
    s_sensor_on = false;
    s_have_accel = false;
}

/* ---------------------------------------------------------- the readings */

bool imu_version(char *out, size_t cap)
{
    uint16_t v = 0;

    if (!s_ready)
    {
        snprintf(out, cap, "not started");
        return false;
    }
    if (bhy2_get_kernel_version(&v, &s_bhy2) != BHY2_OK)
    {
        snprintf(out, cap, "kernel version unreadable");
        return false;
    }
    /* The SDK returns the kernel version packed into 16 bits, high byte major. */
    snprintf(out, cap, "BHI260AP fw kernel %u.%u", (unsigned)(v >> 8),
             (unsigned)(v & 0xFF));
    return true;
}

bool imu_sensor_on(void)
{
    return s_sensor_on;
}

/* Turn the accelerometer on at `hz` (0 turns it off). Idempotent. */
bool imu_accel_enable(float hz)
{
    if (!s_ready)
        return false;

    /* ASK THE FIRMWARE WHAT IT HAS BEFORE ASKING IT TO DO ANYTHING. This is the
     * check the vendor's own `configure()` makes first, and leaving it out cost
     * a cycle: bhy2_set_virt_sensor_cfg() returns BHY2_OK for a sensor the
     * uploaded firmware does not carry, and then simply never produces a
     * sample. The part's FIFO stays empty and nothing anywhere says why.
     *
     * It matters here specifically because the firmware image is a CHOICE. KLIO
     * (121 KB, what this port uploads, and what LilyGo's own watch firmware
     * uses) and the "shuttle" builds (680 KB, what SensorLib's examples use) do
     * not carry the same set of virtual sensors. */
    if (!bhy2_is_sensor_available(BHY2_SENSOR_ID_ACC, &s_bhy2))
    {
        /* WHICH FIRMWARE IS LOADED DECIDES THIS, and the answer is not a
         * detail: the KLIO image this port uploads (121 KB, and what LilyGo's
         * own watch firmware uses) does NOT carry the accelerometer virtual
         * sensor, while the "shuttle" builds SensorLib's examples use (680 KB)
         * do. So enumerate what IS there rather than guessing. */
        ESP_LOGE(TAG, "this firmware has no accelerometer virtual sensor");
        s_present_n = imu_list_sensors(s_present, sizeof(s_present));
        ESP_LOGI(TAG, "%d sensors: %s", s_present_n, s_present);
        snprintf(s_why, sizeof(s_why),
                 "the uploaded firmware carries %d virtual sensors, none of them"
                 " the accelerometer", s_present_n);
        return false;
    }

    /* TELL THE PART HOW OFTEN TO HAND OVER DATA, because the default is "when
     * the FIFO is nearly full". With a default watermark the accelerometer
     * delivers about 26 samples once a second and nothing in between, which is
     * useless for a window redrawing at 4 Hz and leaves the FIFO on the edge of
     * overflowing ("meta 12 fifo overflow"). Both knobs are set here:
     *
     *   report latency  20 ms -> the part flushes on its own clock, so samples
     *                            arrive steadily instead of in a second-long
     *                            lurch
     *   FIFO watermark  32 bytes -> interrupts early, so a poll has something to
     *                            read rather than waiting for a batch
     *
     * Reading the API docs: latency is the maximum time data may sit in the
     * part before being reported, and the watermark is the level that triggers
     * the interrupt. They are complementary, not alternatives -- latency alone
     * still leaves a full second of silence if the watermark is high.
     */
    (void)bhy2_set_fifo_wmark_wkup(32, &s_bhy2);

    /* Instrumented because this call is where a stall shows up, and a stall
     * here takes the shell task with it. The interrupt status is read first so
     * the log says whether the part was signalling before we asked it to do
     * anything -- BHY2_IST_MASK_STATUS is what the SDK's wait loop polls. */
    {
        uint8_t ist = 0;
        if (bhy2_get_interrupt_status(&ist, &s_bhy2) == BHY2_OK)
            ESP_LOGD(TAG, "before cfg: int status %02x", ist);
    }
    const int64_t tc = esp_timer_get_time();
    ESP_LOGD(TAG, "configuring accel at %.0f Hz", (double)hz);
    int8_t rc = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, hz, 20, &s_bhy2);
    ESP_LOGD(TAG, "cfg returned %d after %d ms", rc,
             (int)((esp_timer_get_time() - tc) / 1000));
    if (rc != BHY2_OK)
    {
        ESP_LOGW(TAG, "accelerometer config %s failed (%d)",
                 hz > 0 ? "on" : "off", rc);
        return false;
    }
    s_sensor_on = hz > 0;

    if (s_sensor_on)
    {
        /* Register once; a second registration would overwrite the table slot
         * and is harmless, but there is no reason to do it every poll. */
        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, on_accel, NULL,
                                                &s_bhy2);
    }
    return true;
}

/* Drain whatever the part has queued and hand it to the callbacks. Cheap when
 * nothing is waiting: one I2C status read. */
bool imu_poll(void)
{
    if (!s_ready || !s_sensor_on)
        return false;

    /* There used to be an interrupt-status gate here, returning early unless the
     * WAKEUP bits were set. It discarded data two ways: the accelerometer is
     * routed to the NON-wakeup FIFO, and the part interrupts on a watermark, so
     * trusting a stale bit let the FIFO overflow. The stall it was written for
     * was the missing async-status-channel control, fixed above. One bug does not
     * justify a filter that drops samples. */
    /* ONE DRAIN PER POLL, WHICH IS THE SHAPE THAT WAS VERIFIED. Two attempts to
     * be cleverer than this both wedged the shell task:
     *
     *   - reading the FIFO length first to gate the drain let a backlog build
     *     until the SDK's own loop could not make progress;
     *   - draining in bounded passes still handed the SDK a FIFO larger than
     *     the work buffer, and its loop reads into `buffer_size - read_length`,
     *     so at the end it issues a zero-length read, `remain_length` never
     *     changes, and it spins for ever.
     *
     * A single call with a 2 KB buffer covers 1.5 s of 100 Hz data, and the
     * sensor is turned OFF when a command or window stops reading, so a backlog
     * has nowhere to come from. Nothing here should be "improved" without
     * reproducing this test first: run a command twice in a row and then type
     * at the prompt. */
    return bhy2_get_and_process_fifo(s_fifo, sizeof(s_fifo), &s_bhy2) == BHY2_OK;
}

/* The most recent acceleration in g. `count` is how many samples have arrived
 * since start, which is how a caller tells a live sensor from a frozen one. */
bool imu_accel(float *x, float *y, float *z, uint32_t *count)
{
    if (!s_have_accel)
        return false;
    *x = s_ax;
    *y = s_ay;
    *z = s_az;
    if (count)
        *count = s_accel_count;
    return true;
}

/* ------------------------------------------------------- td_imu_ops_t */

/* Turn orientation on too, when the firmware offers it. Silent when it does
 * not: a board whose firmware lacks sensor 69 still shows acceleration. */
static bool ori_enable(float hz)
{
    bool any = false;

    /* BOTH ORIENTATION SENSORS, and that is not belt-and-braces. 69 (DEVICE_ORI)
     * reports ON CHANGE, so a watch lying still produces nothing and the window
     * shows "--" for ever -- which is what happened. 70 (DEVICE_ORI_WU) is the
     * wakeup variant and reports continuously. Subscribing to both means a
     * resting watch still has a heading, and a moving one gets fresh values
     * from whichever fires. */
    static const uint8_t ids[] = {BHY2_SENSOR_ID_DEVICE_ORI,
                                  BHY2_SENSOR_ID_DEVICE_ORI_WU};
    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++)
    {
        if (!bhy2_is_sensor_available(ids[i], &s_bhy2))
            continue;
        if (bhy2_set_virt_sensor_cfg(ids[i], hz, 20, &s_bhy2) != BHY2_OK)
            continue;
        (void)bhy2_register_fifo_parse_callback(ids[i], on_ori, NULL, &s_bhy2);
        any = true;
    }
    s_ori_on = any;
    return any;
}

/* START MEANS "STREAMING", NOT MERELY "BOOTED". The first version of this
 * returned as soon as the part was up, and the window then sat on "waiting for
 * the first sample" for ever: the firmware was running but neither sensor had
 * been enabled, so nothing ever arrived and no cell ever changed -- which is
 * indistinguishable from a dead link. Opening the window has to leave the
 * sensors producing data. */
static bool ops_start(const char **why)
{
    if (!imu_start())
    {
        if (why)
            *why = imu_why();
        return false;
    }
    /* 50 Hz is twice what a 4 Hz window needs and leaves room for a UI that
     * later wants to watch a tilt more closely. */
    if (!imu_stream_on(50.0f))
    {
        if (why)
            *why = "the part is up but its sensors would not start";
        return false;
    }
    return true;
}

static void ops_stop(void)
{
    if (s_ori_on)
    {
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_DEVICE_ORI, 0.0f, 0, &s_bhy2);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_DEVICE_ORI_WU, 0.0f, 0, &s_bhy2);
        s_ori_on = false;
    }
    imu_stop();
    s_have_ori = false;
}

static bool ops_latest(td_imu_t *out)
{
    if (!out)
        return false;
    memset(out, 0, sizeof(*out));

    /* LATEST MEANS "DRAIN, THEN TELL ME" -- the window has no poll call, and
     * the first version of this forgot to drain at all. The result was a window
     * that sat on "waiting for the first sample" while 50 Hz of data piled up in
     * the part's FIFO: the sensor was running, the readings were there, and
     * nothing ever asked for them. The contract in td_sysinfo.h says the port
     * does the work and a window may call this every tick, so the drain belongs
     * here. */
    (void)imu_poll();

    out->accel_valid = imu_accel(&out->ax, &out->ay, &out->az, &out->accel_samples);

    /* TILT FROM GRAVITY. When the watch is still the accelerometer vector IS
     * "up", so the angle of each axis off vertical falls straight out of the
     * components -- no fusion needed, and no magnetometer, which matters because
     * a magnetometer cannot see heading indoors next to a speaker either.
     *
     *   pitch = atan2(-x, sqrt(y^2 + z^2))   nose up/down
     *   roll  = atan2( y, sqrt(x^2 + z^2))   left/right lean
     *
     * The signs are chosen so that tipping the watch away from you reads
     * positive pitch, which is the convention a pilot would expect. */
    if (out->accel_valid)
    {
        const float rad2deg = 57.2957795f;
        out->tilt_pitch = atan2f(-out->ax, sqrtf(out->ay * out->ay + out->az * out->az)) * rad2deg;
        out->tilt_roll = atan2f(out->ay, sqrtf(out->ax * out->ax + out->az * out->az)) * rad2deg;
        out->tilt_valid = true;
    }
    snprintf(out->posture, sizeof(out->posture), "%s", imu_posture(out));
    out->ori_valid = s_have_ori;
    out->heading = s_heading;
    out->pitch = s_pitch;
    out->roll = s_roll;
    snprintf(out->part, sizeof(out->part), "BHI260AP");
    snprintf(out->version, sizeof(out->version), "%s", s_version);
    return out->accel_valid || out->ori_valid;
}

static const td_imu_ops_t s_imu_ops = {
    .start = ops_start,
    .stop = ops_stop,
    .latest = ops_latest,
};

const td_imu_ops_t *board_imu_ops(void)
{
    return &s_imu_ops;
}

/* Turn both sensors on at the rate the window wants. Called by the window's
 * tick, so it must be cheap when they are already on. */
bool imu_stream_on(float hz)
{
    if (!s_ready)
        return false;
    if (s_sensor_on && s_ori_on)
        return true;
    if (!s_sensor_on && !imu_accel_enable(hz))
        return false;
    (void)ori_enable(hz);
    return true;
}

bool imu_orientation(float *heading, float *pitch, float *roll)
{
    if (!s_have_ori)
        return false;
    if (heading) *heading = s_heading;
    if (pitch)   *pitch = s_pitch;
    if (roll)    *roll = s_roll;
    return true;
}

/* ------------------------------------------------------- status, at last ---- */

/* WHAT DID THE PART ACTUALLY SAY?
 *
 * This is the hole the entry-51 injection test left, and it is structural rather
 * than a mistake in that test. EVERY BHY2 command on this SDK's command path is
 * fire-and-forget: bhy2_hif_exec_cmd() writes the packet to CHAN_CMD and returns
 * BHY2_OK as soon as the I2C write completes. It never waits for the status
 * response and never reads it. So "accepted" in every message this driver has
 * printed has meant "the bytes left the ESP32" -- and the verdict that injection
 * is inert was reached without once asking the part what it thought of the
 * payload.
 *
 * The part answers on a separate channel, and bhy2_defs.h names what it can say:
 *
 *   0x04  INJECT_SENSOR_CONF_REQ   "tell me which sensor this data is for"
 *   0x0f  CMD_ERR                  rejected outright
 *
 * Two answers, two completely different futures: 0x04 means injection is alive
 * and the handshake was simply never implemented on this side; 0x0f means the
 * handler is compiled out and the only route is our own firmware for the Fuser
 * core.
 *
 * THE SEQUENCE IS BORROWED FROM BOSCH, not invented. bhy2_hif_get_parameter()
 * is the one path in this SDK that reads a response: it clears the async status
 * bit, execs the command, waits for BHY2_IST_MASK_STATUS, then pulls the status
 * FIFO. That is what happens here, and the async bit has to be cleared BEFORE
 * the command goes out -- with it set, the response is delivered asynchronously
 * into the data FIFO and a synchronous read finds nothing.
 *
 * THE TIMEOUT IS OURS. bhy2_read_status() waits 1000 x 10 ms = ten seconds for a
 * status that may never come, inside the shell's own task, and a call that
 * cannot be interrupted takes the terminal with it. So the interrupt-status
 * register is polled here first, and read_status() is only called once it says
 * something is waiting.
 */

/* Name a status code. bhy2_defs.h's list, and the two that matter here. */
static const char *status_name(uint16_t code)
{
    switch (code)
    {
    case BHY2_STATUS_INITIALIZED:            return "initialised";
    case BHY2_STATUS_DEBUG_OUTPUT:           return "debug output";
    case BHY2_STATUS_CRASH_DUMP:             return "CRASH DUMP";
    case BHY2_STATUS_INJECT_SENSOR_CONF_REQ: return "INJECT SENSOR CONF REQ";
    case BHY2_STATUS_SW_PASS_THRU_RES:       return "passthru result";
    case BHY2_STATUS_SELF_TEST_RES:          return "self-test result";
    case BHY2_STATUS_FOC_RES:                return "foc result";
    case BHY2_STATUS_SYSTEM_ERROR:           return "SYSTEM ERROR";
    case BHY2_STATUS_SENSOR_ERROR:           return "SENSOR ERROR";
    case BHY2_STATUS_FLASH_ERASE_COMPLETE:   return "flash erase done";
    case BHY2_STATUS_FLASH_WRITE_COMPLETE:   return "flash write done";
    case BHY2_STATUS_FLASH_CONTINUE_UPLOAD:  return "flash upload cont";
    case BHY2_STATUS_HOST_EV_TIMESTAMP:      return "host timestamp";
    case BHY2_STATUS_DUT_TEST_RES:           return "dut test result";
    case BHY2_STATUS_CMD_ERR:                return "CMD ERR";
    default: break;
    }
    if (BHY2_IS_STATUS_GET_PARAM_OUTPUT(code))
        return "parameter output";
    return "unrecognised";
}

/* What the sweep saw, counted rather than printed line by line: the terminal is
 * 26 rows and a per-exchange transcript scrolls the answer off the top. One line
 * per shape plus a summary is what actually gets read. */
#define IMU_STATUS_PAYLOAD 16

static uint16_t s_status_code[8];
static int      s_status_hits[8];
static int      s_status_n;
static int      s_status_total;
static int      s_status_silent;
static uint16_t s_last_code;
static uint8_t  s_last_payload[IMU_STATUS_PAYLOAD];
static uint32_t s_last_len;
static bool     s_last_have;

static void status_reset(void)
{
    s_status_n = 0;
    s_status_total = 0;
    s_status_silent = 0;
    s_last_have = false;
    s_last_len = 0;
}

static void status_tally(uint16_t code)
{
    s_status_total++;
    for (int i = 0; i < s_status_n; i++)
    {
        if (s_status_code[i] == code)
        {
            s_status_hits[i]++;
            return;
        }
    }
    if (s_status_n < 8)
    {
        s_status_code[s_status_n] = code;
        s_status_hits[s_status_n] = 1;
        s_status_n++;
    }
}

/* Switch the part to synchronous status and back. */
typedef struct
{
    uint8_t saved_ctrl;
    bool    was_async;
} imu_status_ctx_t;

static void status_sync_begin(imu_status_ctx_t *ctx)
{
    ctx->saved_ctrl = 0;
    ctx->was_async = false;

    if (bhy2_get_host_intf_ctrl(&ctx->saved_ctrl, &s_bhy2) != BHY2_OK)
        return;

    if (ctx->saved_ctrl & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL)
    {
        ctx->was_async = true;
        (void)bhy2_set_host_intf_ctrl(
            (uint8_t)(ctx->saved_ctrl & ~BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL), &s_bhy2);
    }
}

static void status_sync_end(const imu_status_ctx_t *ctx)
{
    if (ctx->was_async)
        (void)bhy2_set_host_intf_ctrl(ctx->saved_ctrl, &s_bhy2);
}

/* Wait up to `budget_ms` for one status response. Records it; prints nothing.
 * The caller decides how to say it, because the answer belongs on one line. */
static bool status_read_one(int budget_ms)
{
    const int64_t t0 = esp_timer_get_time();
    bool ready = false;

    while (esp_timer_get_time() - t0 < (int64_t)budget_ms * 1000)
    {
        uint8_t ist = 0;
        if (bhy2_get_interrupt_status(&ist, &s_bhy2) == BHY2_OK &&
            (ist & BHY2_IST_MASK_STATUS))
        {
            ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (!ready)
    {
        s_status_silent++;
        return false;
    }

    uint16_t code = 0;
    uint32_t got = 0;

    if (bhy2_read_status(&code, s_last_payload, IMU_STATUS_PAYLOAD, &got,
                         &s_bhy2) != BHY2_OK)
    {
        s_status_silent++;
        return false;
    }

    s_last_code = code;
    s_last_len = got > IMU_STATUS_PAYLOAD ? IMU_STATUS_PAYLOAD : got;
    s_last_have = true;
    status_tally(code);
    return true;
}

/* One shape, one line.
 *
 * The previous version printed six lines per shape -- mode rc, three inject rc's,
 * three status waits -- which is 40 lines of transcript and the terminal holds
 * 26, so the summary that answers the question scrolled off the top. What is
 * actually needed is: did the part say anything, and did the value move.
 *
 * `status` collapses to "none" when the part said nothing at all, and names the
 * code (with its payload) when it did. Those two are different answers and the
 * whole experiment turns on which one this firmware gives. */
static bool try_inject(const char *name, const uint8_t *payload, uint16_t len,
                       enum bhy2_data_inj_mode mode, float bx, float by, float *bz)
{
    (void)bx;
    (void)by;

    imu_status_ctx_t ctx;
    status_sync_begin(&ctx);

    int8_t rc = bhy2_set_data_injection_mode(mode, imu_dev());
    if (rc != BHY2_OK)
    {
        status_sync_end(&ctx);
        printf("  %-22s mode REFUSED rc %d\n", name, rc);
        return false;
    }
    (void)status_read_one(120);

    int sent = 0;
    for (int i = 0; i < 3; i++)
    {
        rc = bhy2_inject_data(payload, len, imu_dev());
        if (rc != BHY2_OK)
            break;
        sent++;
        (void)status_read_one(120);
        vTaskDelay(pdMS_TO_TICKS(8));
    }

    /* Back to async before touching the FIFO: with the bit clear, data events and
     * status responses share a channel and nothing parses. */
    status_sync_end(&ctx);

    float x = 0, y = 0, z = 0;
    uint32_t n = 0;
    for (int i = 0; i < 30; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        imu_poll();
    }
    const bool got = imu_accel(&x, &y, &z, &n);

    (void)bhy2_set_data_injection_mode(BHY2_NORMAL_MODE, imu_dev());

    const bool landed = got && fabsf(z - 2.50f) < 0.12f;

    printf("  %-22s %ux %s z %+5.2f%s\n", name, sent,
           s_last_have ? "said" : "silent", got ? (double)z : (double)*bz,
           landed ? "  <-- LANDED" : "");

    if (s_last_have)
    {
        printf("      -> 0x%04x %s", (unsigned)s_last_code, status_name(s_last_code));
        for (uint32_t i = 0; i < s_last_len; i++)
            printf(" %02x", s_last_payload[i]);
        printf("\n");
        s_last_have = false;
    }

    if (landed)
        *bz = z;
    return landed;
}

/* ------------------------------------------------------- injection, exported */

void imu_drain_hard(void)
{
    if (!s_ready)
        return;
    /* A finite number of passes, never a loop on "is it empty" -- the SDK's own
     * drain spins for ever if the FIFO outgrows the buffer, and this function
     * exists to avoid that class of bug rather than recreate it. */
    for (int i = 0; i < 12; i++)
        (void)bhy2_get_and_process_fifo(s_fifo, sizeof(s_fifo), &s_bhy2);
}

void imu_quiesce(void)
{
    if (!s_ready)
        return;

    /* Drain first, so the stop commands are not queued behind sample data. */
    imu_drain_hard();

    (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 0.0f, 0, &s_bhy2);
    (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 0.0f, 0, &s_bhy2);
    (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 0.0f, 0, &s_bhy2);

    s_sensor_on = false;
    s_ori_on = false;

    /* And again, for whatever was already in flight when they stopped. */
    imu_drain_hard();
    vTaskDelay(pdMS_TO_TICKS(20));
    imu_drain_hard();

    /* A flush is the belt to that braces: it discards anything the part still
     * holds, so the next operation starts from a known-empty FIFO. */
    (void)bhy2_clear_fifo(0xFF, &s_bhy2);
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_drain_hard();
}

bool imu_inject_begin(int mode)
{
    if (!s_ready)
        return false;

    const int8_t rc = bhy2_set_data_injection_mode((enum bhy2_data_inj_mode)mode,
                                                   &s_bhy2);

    /* READ THE STATUS ONCE AFTER SWITCHING, which is what Bosch's own host tool
     * does and comments as "Fix for mode switching not occurring cleanly"
     * (BHy2CLI, callbacks/bhy2cli_callbacks.c, dmode_callback). The mode command
     * is fire-and-forget like every other one, so this is the only confirmation
     * that the part actually changed mode before data starts arriving. */
    imu_status_ctx_t ctx;
    status_sync_begin(&ctx);
    (void)status_read_one(200);
    status_sync_end(&ctx);

    /* The helper behind that call clears the async status bit and never puts it
     * back -- every call leaves the part with asynchronous status reporting off
     * for the rest of the session. Restored here because the caller needs the
     * normal data FIFO to keep parsing while it injects. */
    (void)bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &s_bhy2);

    return rc == BHY2_OK;
}

bool imu_inject_frame(const uint8_t *payload, uint16_t len)
{
    if (!s_ready || len == 0 || len > 56)
        return false;

    uint8_t pkt[60] = {0};
    pkt[0] = (uint8_t)(BHY2_CMD_INJECT_DATA & 0xFF);
    pkt[1] = (uint8_t)((BHY2_CMD_INJECT_DATA >> 8) & 0xFF);
    pkt[2] = (uint8_t)(len & 0xFF);
    pkt[3] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(pkt + 4, payload, len);

    /* Written exactly as declared. The SDK's own path rounds the length up to a
     * word and declares the rounded value; for a FIFO-format frame that adds a
     * byte the part did not send, which the parser then walks into. */
    return bhy2_set_regs(BHY2_REG_CHAN_CMD, pkt, (uint16_t)(4 + len), &s_bhy2)
           == BHY2_OK;
}

void imu_inject_end(void)
{
    if (!s_ready)
        return;
    (void)bhy2_set_data_injection_mode(BHY2_NORMAL_MODE, &s_bhy2);
    (void)bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &s_bhy2);
}

/* --------------------------------------------------- KLIO data injection --- */

/* A KLIO frame sink that lives here rather than in klio.c, so this command does
 * not depend on that file's private state. The frame is 11 bytes: d[1] learn
 * index (-1 = none), d[2] progress, d[5] recognition index (0xFF = none). */
static volatile uint32_t s_tap_frames;
static volatile int      s_tap_progress;
static volatile int      s_tap_index = -1;

/* THE FOURTH BYTE IS WHY, AND NOTHING WAS RECORDING IT.
 *
 * The operator watched learning reach 40 percent and drop back to 0, and the
 * captured frames show the mechanism in that byte: `progress 20, reason 0` then
 * `progress 0, reason 1`. The engine's own vocabulary for the field is 0 = ok,
 * 1 = interrupted, 2 = no movement. Entry 73 concluded "interrupted" from two
 * frames and could not say why, because the reason was never kept -- the tap
 * stored index and progress only.
 *
 * Counting each reason costs four counters and no I/O, so this cannot perturb
 * the run it measures. `s_tap_reasons[3]` collects anything outside the
 * documented vocabulary rather than dropping it. */
static volatile uint32_t s_tap_reasons[4];
static volatile uint8_t  s_tap_last_reason;

/* THE FIRST FEW FRAMES, VERBATIM, WITH THEIR SIZE.
 *
 * The decode below (d[1] = index, d[2] = progress) comes from the extended-port
 * notes, not from Bosch, and this session has produced progress values of 246 --
 * which a 0..100 field cannot be. Recording the raw bytes is the only way to
 * tell a bad decode from a bad engine. */
static uint8_t s_tap_raw[6][14];
static uint8_t s_tap_raw_len[6];
static volatile uint8_t s_tap_raw_n;

/* WHAT THE PART'S OWN CLOCK SAYS, because the delta field we inject is
 * undocumented in units.
 *
 * bhy2.c adds a TS_LARGE_DELTA's 16-bit value to a running counter, so the
 * field is a DELTA. This command writes an absolute counter into it -- 10000,
 * 20000, ... reset to 0 once it passes 65000, i.e. every seventh block -- which
 * makes the engine's notion of time both wrong in scale and sawtoothed with a
 * 70 ms period. The fix needs the unit, and the unit is not in this SDK.
 *
 * It can be measured instead: the SDK's parsed 64-bit timestamp is handed to
 * every callback, so recording it at the first and last KLIO frame of a run and
 * dividing by the run's real elapsed time gives units per second directly, with
 * no assumption about the part's convention. */
static volatile uint64_t s_tap_ts_first;
static volatile uint64_t s_tap_ts_last;
static volatile bool     s_tap_ts_have;
static uint64_t          s_tap_ts_raw[6];

static void klio_tap_cb(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 1)
        return;

    if (s_tap_raw_n < 6)
    {
        const uint8_t k = s_tap_raw_n++;
        const uint8_t n = info->data_size < 14 ? info->data_size : 14;
        s_tap_raw_len[k] = n;
        memcpy(s_tap_raw[k], info->data_ptr, n);
    }
    /* A KLIO FRAME IS EXACTLY 11 BYTES, AND ANYTHING ELSE IS NOT ONE.
     *
     * The raw dump proved the tap was decoding accelerometer events as KLIO:
     *
     *   11: 00 ff 14 00 00 ff 00 00 00 00 fd   KLIO, d[2] = 0x14 = progress 20
     *    7: 43 03 01 07 11 0e 0d               accel x/y/z = 0.20/0.44/0.88 g
     *
     * A 7-byte accel frame passed a `>= 3` check and its data bytes were read as
     * progress and index, which is where the impossible values came from -- 246
     * among them. The count of "frames" was likewise inflated with samples that
     * were never KLIO output.
     *
     * 11 is not a guess: it is the size recorded when this engine was first made
     * to work (entry 52), confirmed above against a frame whose d[2] is a legal
     * progress value. */
    if (info->data_size != 11)
        return;
    const uint8_t *d = info->data_ptr;
    s_tap_index = (int8_t)d[1];
    s_tap_progress = d[2];
    s_tap_last_reason = d[3];
    /* The parsed timestamp the SDK just computed for this frame. */
    if (info->time_stamp)
    {
        const uint64_t v = *info->time_stamp;
        if (!s_tap_ts_have)
        {
            s_tap_ts_first = v;
            s_tap_ts_have = true;
        }
        s_tap_ts_last = v;
        if (s_tap_frames < 6)
            s_tap_ts_raw[s_tap_frames] = v;
    }
    /* 0/1/2 are the documented reasons; anything else lands in slot 3 so an
     * unexpected value is visible rather than silently folded in with "ok". */
    s_tap_reasons[d[3] < 3 ? d[3] : 3]++;
    s_tap_frames++;
}

static void klio_tap_reset(void)
{
    s_tap_raw_n = 0;
    s_tap_frames = 0;
    s_tap_progress = 0;
    s_tap_index = -1;
    s_tap_last_reason = 0;
    s_tap_ts_have = false;
    s_tap_ts_first = 0;
    s_tap_ts_last = 0;
    for (int i = 0; i < 6; i++)
        s_tap_ts_raw[i] = 0;
    for (int i = 0; i < 4; i++)
        s_tap_reasons[i] = 0;
}

/* SUCCESS REQUIRES MORE THAN A FIELD THAT LOOKS HIGH.
 *
 * The progress byte in a KLIO frame is NOT confined to 0..100 -- a run measured
 * `peak progress 246`, which is a wrapped or reinterpreted value, and a check of
 * `progress >= 100` fires on it and announces a pattern that does not exist.
 * That is exactly the class of false positive this project has been bitten by
 * before (entry 50: `learn_index >= 0` read a zero-initialised struct as a
 * learnt pattern).
 *
 * So success requires BOTH: progress EXACTLY 100, which is the value the engine
 * actually reports when a pattern completes (entry 50 saw 0/20/40/60/80/100),
 * and a read_pattern that returns a full 157-byte blob afterwards. A claim this
 * important gets two independent confirmations. */
static bool klio_tap_learnt(void)
{
    /* Informational only -- the real gate is a 157-byte pattern read back from
     * the engine. Progress has been seen at 1, 35, 96 and 246, so neither
     * `>= 100` nor `== 100` is trustworthy on its own. */
    return s_tap_frames > 0;
}

static int klio_tap_index(void) { return s_tap_index; }
static uint32_t klio_tap_frames(void) { return s_tap_frames; }

/* Report progress without filling the screen: every two seconds, and only when
 * it changed. The console is 26 rows and a 30-second run prints 30 lines. */
static void peer_progress(int *peak, int i, int steps, int step_ms)
{
    if (s_tap_progress > *peak && s_tap_progress <= 100)
        *peak = s_tap_progress;
    if ((i % 100) == 99)
        /* THE ELAPSED SECONDS COME FROM THE ACTUAL STEP, NOT A CONSTANT.
         *
         * This printed `(i + 1) * 20 / 1000`. Twenty was the block interval when
         * injection ran at 50 Hz; the interval became 10 ms and the label did
         * not, so a 30-second run reported `60 s` and every console timeline in
         * this project has been double its true length since entry 73. The
         * operator's own account of a run (0, then a climb to 60, then back to
         * 0) is what exposed it -- the file trace and the screen disagreed about
         * when things happened. */
        printf("  %2d s  progress %3d%%  frames %lu\n",
               (i + 1) * step_ms / 1000, s_tap_progress,
               (unsigned long)s_tap_frames);
    (void)steps;
}


/* THE ALIGNMENT RULE, WHICH COST THIS PROJECT FIVE ENTRIES.
 *
 * The injection handler rejects any payload whose length is not a multiple of
 * four, with error 0x7c. That code is undocumented in the BHI2xy SDK this port
 * vendors and IS documented in the BHI360 SDK:
 *
 *   BHI360_ERR_INPUT_CHANNEL_NOT_WORD_ALIGNED      0x7C
 *   BHI360_ERR_SENSOR_DATA_INJECTION               0xE2
 *
 * Every payload sent before entry 67 was 6, 7, 13 or 17 bytes -- none divisible
 * by four -- so every one was rejected before reaching the handler, and the
 * rejection read as "the part ignores injection".
 *
 * So: round the length up to a multiple of four and pad with zeros. The handler
 * parses the block by walking event ids and lengths, so trailing zeros are read
 * as filler (0xFF is BHY2_SYS_ID_FILLER) and ignored. */
static uint16_t inject_pad(uint8_t *pkt, uint16_t len)
{
    /* ZERO, NOT 0xFF, AND THE REASON IS IN BOSCH'S OWN SDK.
     *
     * bhy2_hif_exec_cmd_generic() rounds the declared length up to a multiple of
     * four and then memsets the tail of its command buffer to zero. So when
     * Bosch's own tool calls bhy2_inject_data(log, 17) -- an UNALIGNED length --
     * what reaches the part is a declared length of 20 with three ZERO bytes of
     * padding. This function originally used 0xFF on the theory that
     * BHY2_SYS_ID_FILLER is the right filler; 0xFF is a filler ID the parser
     * knows, but the injection handler is not necessarily the FIFO parser, and
     * the SDK's behaviour is the observable one. */
    const uint16_t padded = (uint16_t)((len + 3u) & ~3u);
    for (uint16_t i = len; i < padded; i++)
        pkt[i] = 0x00;
    return padded;
}

/* Send one block, word-aligned, and report the part's verdict.
 *
 * The error register is CLEARED before each attempt by a soft status read,
 * because it is sticky: without that a new failure is indistinguishable from an
 * old one, which is exactly how 0x7c hid the truth for five entries. */
static int8_t inject_block(uint8_t *pkt, uint16_t len)
{
    const uint16_t padded = inject_pad(pkt, len);

    if (!imu_inject_frame(pkt, padded))
        return -1;

    uint8_t err = 0;
    (void)bhy2_get_error_value(&err, &s_bhy2);
    return (int8_t)err;
}

/* FEED KLIO SYNTHETIC MOTION AND SEE WHETHER IT LEARNS.
 *
 * This is the operator's whole idea, reduced to one command: put a known,
 * repeatable waveform into the engine instead of a hand, and watch whether the
 * classifier treats it as a gesture.
 *
 * THE BLOCK FORMAT. An injected block is one FIFO sample: an accelerometer
 * event (id + 3 x int16, 7 bytes), a gyroscope event (7 bytes) and a
 * large-delta timestamp (3 bytes) -- 17 bytes, padded to 20 for the alignment
 * rule. Bosch's own field-trial logs contain exactly this.
 *
 * WHY A GESTURE AND NOT A SINE. KLIO segments motion into repetitions and
 * counts them; progress runs 0/20/40/60/80/100. An endless sine has no
 * repetition boundary, so it cannot be learned however clean it is. The default
 * waveform is therefore discrete: a swing and a settle, repeated, with a rest
 * between. `wave 1` keeps the sine as a control, because a control that fails
 * is how this was established.
 */
static int cmd_imu_inject(int seconds, int shape, int target, int phys)
{
    if (!s_ready && !imu_start())
    {
        printf("imu: %s\n", s_why);
        return 1;
    }

    /* SHAPE 3: ONE CYCLE COMPUTED ONCE, THEN REPLAYED INDEX FOR INDEX.
     *
     * Every other shape recomputes the waveform per block from
     * `fmodf(t, period)` where t is `i * 0.01` as a float. Ten milliseconds is
     * not exactly representable in binary, so the phase at the start of cycle n
     * is not the phase at the start of cycle 0, and the INT16 sample values
     * differ between cycles that are supposed to be identical. KLIO's whole job
     * is deciding whether consecutive movements are the same movement, so a
     * waveform whose cycles are not bit-identical is testing the wrong thing.
     *
     * This shape builds the cycle once into a table -- phase driven by an integer
     * index, so cycle k and cycle k+1 use numerically identical inputs -- and the
     * loop reads table[i % cycle_blocks]. If the engine still resets at the same
     * point, cycle-to-cycle sameness is not the problem and the waveform's SHAPE
     * is, which is the next thing to vary.
     *
     * The same 0.9 s swing and 0.6 s rest as shape 0, so the only difference from
     * the baseline is that the cycles are exactly equal. */
    const int cycle_ms = 1500;                       /* 0.9 swing + 0.6 rest */
    const int cycle_blocks = cycle_ms / 10;          /* 150 at the 10 ms rate */
    static int16_t *s_cycle_tab;                     /* 3 int16 per block */
    if (shape == 3 && !s_cycle_tab)
    {
        /* PSRAM, not .bss: EMU-018 records that a few hundred bytes of static
         * internal DRAM is enough to starve a peripheral, and 900 bytes here
         * would repeat exactly that mistake. Allocated once per boot because the
         * table is a fixed property of the waveform, not of the run. */
        s_cycle_tab = heap_caps_malloc((size_t)cycle_blocks * 3 * sizeof(int16_t),
                                       MALLOC_CAP_SPIRAM);
        if (!s_cycle_tab)
        {
            printf("imu: no PSRAM for the cycle table -- shape 3 unavailable\n");
            return 1;
        }
        const float swing_frac = 0.6f;
        for (int k = 0; k < cycle_blocks; k++)
        {
            /* Integer index in, exact phase out: the same k always produces the
             * same three numbers, which is the entire point. */
            const float phase = (float)k / (float)cycle_blocks;
            float theta = 0.0f, dtheta = 0.0f;
            if (phase < swing_frac)
            {
                const float u = phase / swing_frac;
                const float s = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * u);
                theta = s;
                dtheta = (float)M_PI / swing_frac * sinf(2.0f * (float)M_PI * u)
                         / ((float)cycle_ms / 1000.0f);
            }
            s_cycle_tab[k * 3 + 0] = (int16_t)(sinf(theta) * 4096.0f);
            s_cycle_tab[k * 3 + 1] = (int16_t)(cosf(theta) * 4096.0f);
            s_cycle_tab[k * 3 + 2] =
                (int16_t)(dtheta * 180.0f / (float)M_PI * 16.0f);
        }
    }

    /* SHAPE 5: REPLAY A GESTURE RECORDED FROM A REAL HAND.
     *
     * This is the experiment five entries of waveform synthesis were working
     * towards and could not answer. A gesture performed by a person DOES learn --
     * `klio learn` reaches 100 percent and `klio detect` then reads back a
     * 157-byte pattern -- so a recording of that same gesture is a known-good
     * input. Injecting it settles the question the synthetic waveforms could not:
     * if a real, learnable recording also fails to produce a pattern, injection
     * is not equivalent to the sensor path and no waveform design will fix it;
     * if it succeeds, the synthesis was wrong all along.
     *
     * The file is six int16 per sample -- ax, ay, az, gx, gy, gz -- written by
     * `imu record` at 100 Hz, which is the rate this command injects at, so one
     * recorded sample becomes one injected block and the motion is replayed at
     * the speed it was performed. It loops if the run outlasts the recording.
     *
     * The values are the part's own integers and are injected unchanged: an
     * acceleration that was 0.512 g when recorded is 2097 in both the file and the
     * block, with no float round trip to move it. */
    static int16_t *s_replay;
    static uint32_t s_replay_n;
    if (shape == 5 && !s_replay)
    {
        FILE *rf = fopen("/fs/root/gesture.bin", "rb");
        if (!rf)
        {
            printf("imu: no /root/gesture.bin -- run `imu record <seconds>` first\n");
            return 1;
        }
        uint8_t hdr[8];
        if (fread(hdr, 1, sizeof(hdr), rf) != sizeof(hdr) ||
            memcmp(hdr, "GEST", 4) != 0)
        {
            printf("imu: gesture.bin is not a recording (bad header)\n");
            fclose(rf);
            return 1;
        }
        /* Sample count is in the header so a truncated file is refused rather
         * than read past. */
        s_replay_n = (uint32_t)hdr[4] | ((uint32_t)hdr[5] << 8) |
                     ((uint32_t)hdr[6] << 16) | ((uint32_t)hdr[7] << 24);
        if (s_replay_n == 0 || s_replay_n > 12000u)
        {
            printf("imu: gesture.bin has an implausible length (%lu)\n",
                   (unsigned long)s_replay_n);
            fclose(rf);
            return 1;
        }
        s_replay = heap_caps_malloc((size_t)s_replay_n * 6 * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM);
        if (!s_replay)
        {
            printf("imu: no PSRAM for the replay buffer\n");
            fclose(rf);
            return 1;
        }
        const size_t want = (size_t)s_replay_n * 6 * sizeof(int16_t);
        const size_t got = fread(s_replay, 1, want, rf);
        fclose(rf);
        if (got != want)
        {
            printf("imu: gesture.bin short read (%u of %u bytes)\n",
                   (unsigned)got, (unsigned)want);
            s_replay = NULL;
            return 1;
        }
        printf("replaying %lu samples (%.1f s at 100 Hz)\n",
               (unsigned long)s_replay_n, (double)s_replay_n / 100.0);
    }

    imu_status_ctx_t sctx;
    status_sync_begin(&sctx);

    /* 1. step-by-step, and read the status once afterwards as Bosch does. */
    if (!imu_inject_begin(BHY2_STEP_BY_STEP_INJECTION))
    {
        printf("imu: injection mode refused\n");
        status_sync_end(&sctx);
        return 1;
    }
    (void)status_read_one(150);
    printf("mode: step-by-step\n");

    /* 2. CONFIGURE THE SENSORS. Injection supplies data FOR a sensor; with none
     * configured nothing is listening. Accel and gyro at the rate the samples
     * are fed at, and KLIO itself subscribed so its frames are parsed. */
    /* DO NOT CONFIGURE THE PHYSICAL SENSORS, AND THAT IS BOSCH'S OWN ANSWER.
     *
     * Their CLI flow is `actse 114:100 dmode s dinject ...` -- it activates the
     * ALGORITHM and nothing else. This command additionally enabled the
     * accelerometer and gyroscope at 50 Hz, so the real sensors streamed their
     * own data into the same FIFO the injected blocks had to share. That is what
     * produced `meta 12 fifo overflow` on nearly every block and starved the
     * engine: 3 frames in 30 seconds, progress peaking at 96% and never
     * finishing.
     *
     * Injection replaces the physical sensors' contribution. Turning them on as
     * well gives the engine two sources at once. */
    /* ACCEL AND GYRO ARE CONFIGURED, AND THAT IS MEASURED, NOT REASONED.
     *
     * Removing them on the theory that Bosch's CLI configures only the
     * algorithm made the result worse: progress fell from 96% to 25% in the
     * same wall-clock time. The 96% run had both physical sensors at 50 Hz and
     * the algorithm at 100 Hz, so that is the configuration that stays. */
    /* DRAIN, DON'T WAIT. `status_read_one(300)` blocks for up to 300 ms on the
     * STATUS channel and drains nothing from the DATA channel -- so each
     * configuration call left up to 300 ms of accelerometer and gyroscope
     * samples queued, and the next config filled the FIFO past capacity. Every
     * `meta 12 fifo overflow` in this command landed at t=41 s, before learning
     * had even begun.
     *
     * imu_drain_hard() empties the data FIFO in bounded passes and cannot spin,
     * which is the property that matters here. */
    s_sensor_on = true;                 /* so the drain is permitted */
    /* THE PHYSICAL SENSORS BECOME A VARIABLE, because their contribution was
     * never measured cleanly.
     *
     * `phys 0` leaves the accelerometer and gyroscope off and injects into the
     * algorithm alone -- which is what Bosch's own CLI does (`actse 114:100
     * dmode s dinject`). The earlier comparison in this command's history ran
     * while the tap was decoding accelerometer frames as KLIO, so its verdict
     * ("removing them made it worse: 96% to 25%") is void, and `phys 1` keeps
     * that configuration as the default so an unadorned run is unchanged.
     *
     * The instrumented counters are what make this worth re-running: they report
     * FIFO overflows attributed to SENSOR 4, the accelerometer, during injection.
     * If those are real, `phys 0` removes the producer that is overflowing the
     * FIFO instead of trying to drain faster than it fills. */
    if (phys)
    {
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 50.0f, 20, &s_bhy2);
        (void)status_read_one(300);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 50.0f, 20, &s_bhy2);
        (void)status_read_one(300);
    }
    else
    {
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, 0.0f, 0, &s_bhy2);
        (void)status_read_one(300);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 0.0f, 0, &s_bhy2);
        (void)status_read_one(300);
    }

    /* AND THE ALGORITHM ITSELF, AT 100 Hz, WHICH IS THE STEP THAT WAS MISSING.
     *
     * Bosch's own CLI documents the flow as:
     *
     *   actse 114:100   dmode s   dinject field_log.txt   dmode n   actse 114:0
     *
     * `actse 114:100` ACTIVATES SENSOR 114 AT 100 Hz -- the algorithm, not the
     * accelerometer. Every attempt in this port configured the physical sensors
     * and never the consumer, and injection supplies data FOR a sensor that is
     * running. Sensor 112 is KLIO, 114 is SWIM; whatever the image carries, the
     * caller passes it in. */
    /* THE RATE MUST MATCH WHAT IS INJECTED. Configuring the algorithm for
     * 100 Hz while feeding it 50 blocks a second leaves it asking for twice
     * what it gets, which showed up as `meta 12 fifo overflow` plus
     * `meta 11 SENSOR ERROR` and interrupted learning mid-progress. */
    /* 100 Hz, MEASURED NOT REASONED. Matching the configured rate to the
     * injected block rate (both 50 Hz) made learning collapse -- progress 1%,
     * against 96% when the algorithm was configured at 100 Hz and fed 50
     * blocks a second. The engine wants to run faster than it is fed. */
    (void)bhy2_set_virt_sensor_cfg((uint8_t)target, 100.0f, 20, &s_bhy2);
    (void)status_read_one(400);
    printf("configured accel+gyro %s, sensor %d @100 Hz\n",
           phys ? "@50 Hz" : "OFF", target);

    /* 3. drain, then flush, so a leftover sample cannot be mistaken for an
     * injected one. */
    for (int i = 0; i < 10; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)imu_poll();
    }
    (void)bhy2_clear_fifo(0xFF, &s_bhy2);
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)imu_poll();

    /* 4. learning on, and reset first so progress is comparable between runs. */
    bhy2_klio_sensor_state_t learn = {
        .learning_enabled = 1, .learning_reset = 1,
        .recognition_enabled = 0, .recognition_reset = 0,
    };
    (void)bhy2_klio_set_state(&learn, &s_bhy2);
    (void)imu_poll();
    /* Register the tap on the TARGET sensor, whatever it is -- 112 is KLIO and
     * 114 is SWIM, and watching the wrong one reported "0 frames" whether or
     * not the algorithm was producing anything. */
    (void)bhy2_register_fifo_parse_callback((uint8_t)target,
                                            klio_tap_cb, NULL, &s_bhy2);
    klio_tap_reset();

    /* SNAPSHOT THE PART'S OWN VERDICTS BEFORE THE RUN STARTS.
     *
     * Configuration itself can provoke a FIFO overflow, and this command has
     * been read as though every overflow happened during learning when some
     * may have happened before it. Taking a baseline separates the two, so
     * "overflowed while injecting" is a fact rather than an inference. */
    const uint32_t meta_base_total = s_meta_total;
    const uint32_t meta_base_overflow = s_meta_overflow;
    const uint32_t meta_base_sensor_err = s_meta_sensor_err;
    /* The distribution lives in PSRAM; see the note on s_meta_hist. */
    uint32_t *meta_base_hist = heap_caps_malloc(32 * sizeof(uint32_t),
                                                MALLOC_CAP_SPIRAM);
    s_meta_hist = meta_base_hist;
    if (meta_base_hist)
        for (int i = 0; i < 32; i++)
            meta_base_hist[i] = 0;
    printf("meta before run: %lu total, %lu overflow, %lu sensor error\n",
           (unsigned long)meta_base_total, (unsigned long)meta_base_overflow,
           (unsigned long)meta_base_sensor_err);

    /* OPEN THE DRAIN GATE. imu_poll() returns immediately unless s_sensor_on is
     * set -- a guard that exists so a one-shot command leaves the part quiet --
     * and this command never set it, so THE FIFO WAS NEVER DRAINED DURING
     * INJECTION AT ALL. 500 blocks went in, nothing came out, the FIFO overflowed
     * and the final error was 0xc0 (CMD_ERROR) rather than the 0x00 that one
     * injected block returns on its own. */
    s_sensor_on = true;

    printf("learning on, %d s\n\n", seconds);

    /* TO A FILE, NOT THE SCREEN. The display mirror repaints only changed cells
     * and this console capture applies them to an empty grid, so a run that
     * prints every two seconds comes back interleaved and unreadable -- measured
     * as "progress I20%APframesH626accel" earlier in this project.
     *
     * ONE FILE PER SHAPE, and that is a correction earned the hard way: with a
     * single fixed name, running the gesture and then the sine control destroyed
     * the gesture's evidence before it could be read off the chip, and the two
     * runs are the whole comparison. The name now carries the shape. */
    /* THREE RUNS OF THE SAME CONFIGURATION MUST NOT DESTROY EACH OTHER.
     *
     * Entry 74 found the peak is not reproducible -- 80 percent in one run of
     * the gesture and 20 in an identical second run -- so a single run is not
     * evidence about convergence and every configuration now has to be repeated.
     * Naming the file for shape and phys alone meant the second run overwrote the
     * first, which is how that variance was nearly missed. A run counter is the
     * smallest thing that makes repetition possible. */
    static uint32_t s_run_no;
    char trace_path[52];
    snprintf(trace_path, sizeof(trace_path), "/fs/root/inject-%d-p%d-r%lu.txt",
             shape, phys, (unsigned long)++s_run_no);
    FILE *tr = fopen(trace_path, "w");
    if (tr)
        fprintf(tr, "inject %d s, shape %d, target %d, phys %d, run %lu\n",
                seconds, shape, target, phys, (unsigned long)s_run_no);

    /* 10 ms, TO MATCH THE 100 Hz THE ALGORITHM IS CONFIGURED FOR.
     *
     * This is the single-variable test entry 72 prescribed. Everything else --
     * the block layout, the accel and gyro configuration, the drain pattern,
     * the waveform -- is unchanged from the previous run. The engine asks for
     * 100 Hz and was being fed 50. */
    const int step_ms = 10;                  /* 100 blocks a second */
    const int steps = seconds * 1000 / step_ms;
    uint32_t sent = 0, ts = 0;
    int8_t last_err = -2;
    int peak = 0;
    /* Wall clock around the run, so the part's parsed timestamps can be turned
     * into units per second without assuming the unit. */
    const int64_t run_t0_us = esp_timer_get_time();

    for (int i = 0; i < steps; i++)
    {
        const float t = (float)i * step_ms / 1000.0f;

        /* THE SWING IS 0.9 s IN EVERY GESTURE SHAPE; WHAT CHANGES IS THE REST.
         *
         * Entry 74 left exactly one thing to vary: with the FIFO quiet and no
         * errors at all, KLIO counts repetitions to 80 percent and then reports
         * change_reason 1 -- documented in bhy2_klio_defs.h as "interrupted by a
         * non-repetitive activity" -- and never once reports reason 2, "no
         * significant movement". So the engine is judging the motion, not the
         * transport, and the quantity that judgement is about is the boundary
         * between one repetition and the next.
         *
         * The rest phase is already as still as injection can make it: theta is
         * exactly 0, so the accelerometer sits at a constant 1 g on z and the
         * gyroscope at exactly zero. Stillness therefore cannot be sharpened;
         * its DURATION can be lengthened, and that is the one variable here.
         *
         *   shape 0   0.9 s swing + 0.6 s rest  = 1.5 s cycle  (the baseline)
         *   shape 2   0.9 s swing + 1.5 s rest  = 2.4 s cycle  (this test)
         *   shape 1   the 1 Hz sine control, unchanged
         *
         * One variable per flash, so nothing else differs: same amplitude of
         * 1.0 radian, same 100 Hz injection, same sensors-off configuration. */
        const float swing_s = 0.9f;
        const float rest_s  = (shape == 2) ? 1.5f : 0.6f;
        const float period  = swing_s + rest_s;
        const float swing_frac = swing_s / period;
        const float phase = fmodf(t, period) / period;
        float theta = 0.0f, dtheta = 0.0f;

        if (shape == 1)
        {
            /* GENTLE SINE, 1 Hz, 0.3 RADIAN SWING.
             *
             * At 0.3 rad the lateral component peaks at sin(0.3) = 0.30 g and
             * gravity's share stays near 0.95 g -- inside what a wrist can
             * physically produce, where the previous 1.0 rad gesture reached a
             * full g of lateral swing. That matters because the first KLIO run
             * with injected data emitted a handful of frames and then began
             * returning 0xe2, and an implausible acceleration is one of the
             * three candidate reasons. */
            const float amp_rad = 0.3f;
            const float w = 2.0f * (float)M_PI * 1.0f;   /* 1 Hz */
            theta = amp_rad * sinf(w * t);
            dtheta = amp_rad * w * cosf(w * t);
        }
        else if (phase < swing_frac)
        {
            const float u = phase / swing_frac;       /* 0..1 through the swing */
            const float s = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * u);
            theta = 1.0f * s;
            dtheta = 1.0f * (float)M_PI / swing_frac * sinf(2.0f * (float)M_PI * u)
                     / period;
        }

        int16_t ax = 0;
        int16_t ay, az, gy;
        int16_t gx = 0;
        int16_t gz = 0;
        if (shape == 5 && s_replay)
        {
            /* One recorded sample per block, looping if the run is longer than
             * the recording. ALL SIX values go in exactly as the part reported
             * them -- including ax and gz, which the synthetic shapes always
             * left at zero. Dropping them would replay a different motion from
             * the one that was recorded, which is the whole point of this test. */
            const int k = (int)((uint32_t)i % s_replay_n);
            ax = s_replay[k * 6 + 0];
            ay = s_replay[k * 6 + 1];
            az = s_replay[k * 6 + 2];
            gx = s_replay[k * 6 + 3];
            gy = s_replay[k * 6 + 4];
            gz = s_replay[k * 6 + 5];
        }
        else if (shape == 3 && s_cycle_tab)
        {
            /* Index modulo the cycle length: no float arithmetic in this path at
             * all, so cycle k and cycle 0 are byte-for-byte identical inputs. */
            const int k = i % cycle_blocks;
            ay = s_cycle_tab[k * 3 + 0];
            az = s_cycle_tab[k * 3 + 1];
            gy = s_cycle_tab[k * 3 + 2];
        }
        else
        {
            ay = (int16_t)(sinf(theta) * 4096.0f);
            az = (int16_t)(cosf(theta) * 4096.0f);
            gy = (int16_t)(dtheta * 180.0f / (float)M_PI * 16.0f);
        }
        /* THE GYRO AXIS, WHICH WAS WRONG AND IS A REAL PHYSICAL DEFECT.
         *
         * The acceleration this command injects is gravity rotated by theta in
         * the y-z plane: ay = sin(theta), az = cos(theta). The rotation that
         * carries +z toward +y is a rotation ABOUT X, so its angular velocity is
         * (dtheta, 0, 0). The code has always written that rate into GY -- about
         * the y axis -- which describes a different motion entirely.
         *
         * So every injected block has said two contradictory things at once: the
         * accelerometer reporting a tilt about x and the gyroscope reporting a
         * spin about y. No wrist can do that, and an algorithm that decides
         * whether two movements are the same movement is entitled to reject it.
         *
         * `shape 4` moves the rate to the axis the acceleration implies and
         * changes nothing else -- same swing, same rest, same amplitude, same
         * rate -- so shape 0 stays as the baseline it has been for every entry
         * since 73 and the comparison stays single-variable. */
        if (shape == 4)
        {
            gx = gy;
            gy = 0;
        }

        /* THE TIMESTAMP IS A DELTA, AND ITS UNIT IS MEASURED, NOT ASSUMED.
         *
         * bhy2.c's parser does `*time_stamp += BHY2_LE2U16(buffer + pos + 1)` for
         * a TS_LARGE_DELTA -- it ADDS the 16-bit value to a running counter. This
         * command was writing an absolute millisecond counter into that field and
         * resetting it to 0 whenever it passed 65000, which is wrong twice over:
         * the value was an absolute time in a field the part adds, and it
         * sawtoothed back to zero every seventh block.
         *
         * The unit is in no header this port vendors, so `imu clock` measured it
         * from the physical accelerometer alone -- outside any injection -- and
         * this constant comes from that measurement: consecutive 50 Hz samples
         * advanced the parsed timestamp by 1282-1283 units, so 20 ms is 1283
         * units and 10 ms is 641. (64130 units/s, i.e. one unit = 15.59 us; the
         * part's clock is ~64 kHz.)
         *
         * The old value was 10000 units per block, which is 15.6x too large.
         * A segmentation algorithm that measures how long a gesture took cannot
         * be expected to segment when its clock runs 15.6x fast and resets. */
        const uint32_t ts_delta_per_block =
            (uint32_t)((step_ms * 64130u) / 1000u);   /* 10 ms -> 641 units */
        ts += ts_delta_per_block;
        /* NO RESET. Only the low 16 bits are written into the field (see the
         * block assembly below), so a free-running counter wraps modulo 2^16 on
         * its own -- which is exactly how the part's own delta arithmetic
         * behaves. The previous code zeroed it at 65000, inserting a deliberate
         * backwards jump into a clock the engine was trying to read. */

        /* ONE ID, THEN SIXTEEN BYTES -- not three id-prefixed sub-events.
         *
         * The log format is derivable from Bosch's own arithmetic: the first
         * line is 19 characters for 6 timestamp values (6 x 3 + newline) and
         * every data line is 52 characters for SENSOR_DATA_SIZE 17 values
         * (17 x 3 + newline). So the file is plain two-hex-digit bytes, one
         * space-separated, and an injected block is 17 BYTES, not a stream of
         * FIFO events.
         *
         * 17 = one sensor id + 16. Sixteen decomposes as accel xyz (6) + gyro
         * xyz (6) + a four-byte timestamp, which is the only arrangement that
         * uses all sixteen without waste. The earlier block put three separate
         * ids in and therefore carried only twelve bytes of sample data. */
        /* THE EVENT IDS ARE 55 AND 58, NOT THE SENSOR IDS.
         *
         * This is the answer to the whole injection problem, and it is in
         * Bosch's `dinject_defs.h` under names that look like sensor ids and
         * are not:
         *
         *   #define ACCEL_INJECT_ID          55
         *   #define GYRO_INJECT_ID           58
         *   #define ACCEL_INJECT_EVENT_SIZE   7
         *   #define GYRO_INJECT_EVENT_SIZE    7
         *
         * Every attempt in this port used a SENSOR id -- 4 (ACC), 13 (GYRO),
         * 3 (ACC_RAW), 12 (GYRO_RAW), or the driver ids 76/79 -- and every one
         * returned 0xe2. The injection handler does not take sensor ids; it
         * takes these two dedicated inject ids, and the block is built from
         * them exactly as a FIFO log line is.
         *
         * Verified on the glass: a block built this way returns error 0x00,
         * six times in a row, where the same block with id 4 returns 0xe2. */
        uint8_t blk[20] = {0};
        uint16_t n = 0;
        blk[n++] = 55;                       /* ACCEL_INJECT_ID */
        memcpy(blk + n, &ax, 2); n += 2;
        memcpy(blk + n, &ay, 2); n += 2;
        memcpy(blk + n, &az, 2); n += 2;
        blk[n++] = 58;                       /* GYRO_INJECT_ID */
        memcpy(blk + n, &gx, 2); n += 2;
        memcpy(blk + n, &gy, 2); n += 2;
        memcpy(blk + n, &gz, 2); n += 2;
        blk[n++] = BHY2_SYS_ID_TS_LARGE_DELTA;
        blk[n++] = (uint8_t)(ts & 0xFF);
        blk[n++] = (uint8_t)((ts >> 8) & 0xFF);

        last_err = inject_block(blk, n);
        /* COUNT THE WRITE, NOT THE VERDICT. inject_block returns -1 only when
         * the I2C write itself failed; any other value is the part's error
         * register after a block it did receive. Counting only clean writes
         * made a full run report "0 blocks sent", which read as though nothing
         * was transmitted when in fact every block went out and the handler
         * complained about each one. */
        if (last_err != -1)
            sent++;

        /* DRAIN REPEATEDLY, NOT ONCE. One imu_poll() per 20 ms could not keep
         * up: the accelerometer and gyroscope were streaming at 50 Hz each and
         * the engine emits an output frame per injected block, so roughly 1.5 KB
         * a second arrived against a 2 KB buffer drained fifty times a second
         * WHILE each drain is itself an I2C transaction. `meta 12 fifo overflow`
         * appeared on nearly every block and `meta 11 SENSOR ERROR` with it, and
         * learning was being interrupted mid-progress -- 96%, then 43%, then 20%
         * across runs as the timing drifted.
         *
         * Eight drains across the interval, not one at the end of it. */
        /* CORRECTED: IT IS ONE DRAIN PER BLOCK, NOT EIGHT, AND THE OVERFLOW
         * CLAIM ABOVE IS NOT WHAT THE INSTRUMENTED RUN MEASURED.
         *
         * Entry 74 counted meta events during a 30-second run at 100 blocks a
         * second: 4135 meta events, and ZERO FIFO overflows and ZERO sensor
         * errors. So "overflow appeared on nearly every block" was a reading of
         * the ESP_LOGW stream at a time the tap was also decoding accelerometer
         * frames as KLIO, and it does not survive being counted.
         *
         * The code below is unchanged and is what the count describes: one
         * delay of the full block interval, then one drain. A single
         * bhy2_get_and_process_fifo() with a 2 KB buffer covers well over a
         * second of 100 Hz data, so one drain per 10 ms block cannot starve the
         * FIFO. Do not "improve" this into a loop: imu_poll()'s own comment
         * records two attempts at that which wedged the shell task. */
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        (void)imu_poll();

        peer_progress(&peak, i, steps, step_ms);
        if (tr && (i % 50) == 49)
            /* Tenths of a second, because the trace samples every 500 ms and
             * whole-second labels printed each sample twice under the same
             * second, which read as a stalled counter rather than a 2 Hz one. */
            fprintf(tr, "  %3d.%d s  progress %3d  frames %lu  err %02x  reason %u\n",
                    (i + 1) * step_ms / 1000, ((i + 1) * step_ms % 1000) / 100,
                    s_tap_progress,
                    (unsigned long)s_tap_frames, (uint8_t)(last_err < 0 ? 0 : last_err),
                    (unsigned)s_tap_last_reason);
        /* NO EARLY EXIT. klio_tap_learnt() now means "a frame arrived", which is
         * true after the first one, and breaking on it cut a 30-second run down
         * to 97 blocks. The loop runs its full duration and the verdict comes
         * from read_pattern afterwards. */
    }

    /* GRAB THE PATTERN BEFORE THE STATE CHANGES. Learning leaves the fingerprint
     * inside the algorithm, and anything that resets learning can clear it --
     * `klio save` run afterwards reported `read_pattern failed: invalid param`
     * on a pattern that had just been announced. Reading it here, at the moment
     * of success, is the only reliable point. */
    uint16_t plen = 0;
    bool have_pattern = false;
    if (klio_tap_learnt())
    {
        uint8_t pat[244];
        plen = sizeof(pat);
        const int8_t rp = bhy2_klio_read_pattern(0, pat, &plen, &s_bhy2);
        printf("read_pattern: rc %d, %u bytes\n", rp, (unsigned)plen);
        have_pattern = (rp == BHY2_OK && plen == 157);
    }

    bhy2_klio_sensor_state_t off = {0};
    (void)bhy2_klio_set_state(&off, &s_bhy2);
    imu_inject_end();
    status_sync_end(&sctx);

    printf("\n%lu blocks sent, last error 0x%02x\n", (unsigned long)sent,
           (uint8_t)last_err);
    printf("sensor %d frames %lu, peak progress %d%%\n", target,
           (unsigned long)klio_tap_frames(), peak);
    printf("reasons ok %lu interrupted %lu no-move %lu other %lu\n",
           (unsigned long)s_tap_reasons[0], (unsigned long)s_tap_reasons[1],
           (unsigned long)s_tap_reasons[2], (unsigned long)s_tap_reasons[3]);
    printf("meta overflow %lu sensor-error %lu\n",
           (unsigned long)(s_meta_overflow - meta_base_overflow),
           (unsigned long)(s_meta_sensor_err - meta_base_sensor_err));

    if (tr)
    {
        fprintf(tr, "totals: %lu blocks, %u frames, peak %d, read_pattern %u B\n",
                (unsigned long)sent, (unsigned)s_tap_frames, peak, (unsigned)plen);
        /* WHY THE ENGINE STOPPED, IN THE ENGINE'S OWN WORDS. reason 0 = ok,
         * 1 = interrupted, 2 = no movement, 3 = unexplained value. */
        fprintf(tr, "reasons: ok %lu, interrupted %lu, no-move %lu, other %lu"
                    "  (last %u)\n",
                (unsigned long)s_tap_reasons[0], (unsigned long)s_tap_reasons[1],
                (unsigned long)s_tap_reasons[2], (unsigned long)s_tap_reasons[3],
                (unsigned)s_tap_last_reason);
        fprintf(tr, "meta during run: overflow %lu, sensor-error %lu, all %lu"
                    "  (last ev %u sensor %u, max size %u)\n",
                (unsigned long)(s_meta_overflow - meta_base_overflow),
                (unsigned long)(s_meta_sensor_err - meta_base_sensor_err),
                (unsigned long)(s_meta_total - meta_base_total),
                (unsigned)s_meta_last_ev, (unsigned)s_meta_last_sensor,
                (unsigned)s_meta_max_size);
        /* The distribution, because a total alone invites a guess. */
        if (s_meta_hist)
        {
            fprintf(tr, "meta codes:");
            for (int i = 0; i < 32; i++)
                if (s_meta_hist[i])
                    fprintf(tr, " %d=%lu", i, (unsigned long)s_meta_hist[i]);
            fprintf(tr, "\n");
        }
        else
            fprintf(tr, "meta codes: (no histogram -- PSRAM allocation failed)\n");
        /* THE PART'S CLOCK, MEASURED RATHER THAN ASSUMED. The delta field this
         * command injects is documented nowhere in this SDK's headers, and it is
         * being fed an absolute counter that resets every seventh block. The
         * units per second below come from the SDK's own parsed timestamps at
         * the first and last frame divided by the run's real duration. */
        {
            const int64_t el_us = esp_timer_get_time() - run_t0_us;
            fprintf(tr, "part clock: first %llu last %llu span %llu units"
                        " over %lld us\n",
                    (unsigned long long)s_tap_ts_first,
                    (unsigned long long)s_tap_ts_last,
                    (unsigned long long)(s_tap_ts_last - s_tap_ts_first),
                    (long long)el_us);
            if (s_tap_ts_have && s_tap_ts_last > s_tap_ts_first && el_us > 0)
                fprintf(tr, "part clock: %.1f units per second"
                            " (=> 10 ms = %.2f units)\n",
                        (double)(s_tap_ts_last - s_tap_ts_first) * 1e6 / (double)el_us,
                        (double)(s_tap_ts_last - s_tap_ts_first) * 1e6 / (double)el_us * 0.01);
            else
                fprintf(tr, "part clock: no usable span (%u frames)\n",
                        (unsigned)s_tap_frames);
            fprintf(tr, "parsed ts per frame:");
            for (int k = 0; k < 6; k++)
                fprintf(tr, " %llu", (unsigned long long)s_tap_ts_raw[k]);
            fprintf(tr, "\n");
        }
        fprintf(tr, "\nfirst frames (len:hex):\n");
        for (uint8_t k = 0; k < s_tap_raw_n; k++)
        {
            fprintf(tr, "  %u:", (unsigned)s_tap_raw_len[k]);
            for (uint8_t b = 0; b < s_tap_raw_len[k]; b++)
                fprintf(tr, " %02x", s_tap_raw[k][b]);
            fprintf(tr, "\n");
        }
        /* Verbatim, because the histogram alone cannot say whether these are
         * meta packets at all. */
        fprintf(tr, "first meta packets (len:hex):\n");
        for (uint8_t k = 0; k < s_meta_raw_n && k < 4; k++)
        {
            fprintf(tr, "  %u:", (unsigned)s_meta_raw_len[k]);
            for (uint8_t b = 0; b < s_meta_raw_len[k]; b++)
                fprintf(tr, " %02x", s_meta_raw[k][b]);
            fprintf(tr, "\n");
        }
        fclose(tr);
        printf("\ntrace: cat %s\n", trace_path);
    }

    /* The histogram is PSRAM, so hand it back before returning: this command is
     * meant to be run repeatedly and a leak here would outlive the measurement. */
    s_meta_hist = NULL;
    free(meta_base_hist);

    if (klio_tap_learnt() && have_pattern)
        printf("\nLEARNT pattern %d FROM INJECTED DATA: %u-byte fingerprint read"
               " back.\n", klio_tap_index(), (unsigned)plen);
    else if (klio_tap_learnt())
        printf("\nProgress reached 100 but read_pattern returned %u bytes, so no\n"
               "usable pattern exists. Treat this as NOT learnt.\n",
               (unsigned)plen);
    else if (last_err != 0)
        printf("\nThe handler rejected the blocks with 0x%02x -- framing, not\n"
               "the waveform. 0x7c is alignment, 0xe2 is content.\n",
               (uint8_t)last_err);
    else if (klio_tap_frames() == 0)
        printf("\nBlocks accepted, no KLIO frames -- the engine never saw them.\n");
    else
        printf("\nFrames arrived, progress did not reach 100. The engine saw the\n"
               "motion and did not find it repetitive enough.\n");

    return (klio_tap_learnt() && have_pattern) ? 0 : 1;
}

static int cmd_imu_inject(int seconds, int shape, int target, int phys);
static int cmd_imu(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    /* strcmp, not a first letter: "stop" and "scan" both start with 's', and
     * a first-letter dispatch silently sent `imu scan` to the status branch
     * for one build cycle. */
    const char *sub = argc >= 2 ? argv[1] : "";
    const bool stop = strcmp(sub, "stop") == 0;

    if (stop)
    {
        imu_stop();
        printf("imu: accelerometer off (firmware stays loaded)\n");
        return 0;
    }

    /* BEFORE THE s_ready GATE, because the whole point is to inspect a part
     * whose firmware did not load. */
    /* RECORD A GESTURE FROM A REAL HAND, so it can be replayed through injection.
     *
     * The accelerometer and gyroscope are configured at 100 Hz -- the rate
     * `klio learn` uses and the rate `imu inject` feeds at -- and the latest raw
     * triple from each is sampled every 10 ms into a PSRAM buffer. Sampling
     * rather than queueing costs fidelity if a sensor bursts, but both sensors
     * are at exactly the sampling rate, so each tick has a fresh value; queueing
     * would instead need the two streams merged by timestamp and would add a
     * failure mode to a recording whose whole value is being faithful.
     *
     * Six int16 per sample: ax, ay, az, gx, gy, gz -- the part's own integers.
     * 30 seconds is 18 KB, which is why this is PSRAM and not .bss (EMU-018). */
    if (strcmp(sub, "record") == 0)
    {
        const int secs = argc >= 3 ? atoi(argv[2]) : 20;
        const int n_secs = secs < 3 ? 3 : (secs > 120 ? 120 : secs);
        const uint32_t max_n = (uint32_t)n_secs * 100u;
        if (!imu_start())
        {
            printf("imu: %s\n", s_why);
            return 1;
        }
        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, on_accel, NULL,
                                                &s_bhy2);
        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, on_gyro, NULL,
                                                &s_bhy2);
        /* BOTH AT 100 Hz, which is what the algorithm expects. */
        (void)imu_accel_enable(100.0f);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 100.0f, 20, &s_bhy2);
        s_sensor_on = true;
        s_have_gyro = false;
        /* Let the streams establish before the first sample is taken, so the
         * recording does not open with a stale or absent gyro triple. */
        for (int i = 0; i < 30; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            (void)imu_poll();
        }
        if (!s_have_accel || !s_have_gyro)
        {
            printf("imu: accel %s, gyro %s -- both must stream to record\n",
                   s_have_accel ? "ok" : "SILENT", s_have_gyro ? "ok" : "SILENT");
            return 1;
        }

        int16_t *buf = heap_caps_malloc((size_t)max_n * 6 * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM);
        if (!buf)
        {
            printf("imu: no PSRAM for %lu samples\n", (unsigned long)max_n);
            return 1;
        }
        const uint32_t a0 = s_accel_count, g0 = s_gyro_count;
        printf("recording %d s at 100 Hz -- perform the gesture repeatedly NOW\n",
               n_secs);
        for (uint32_t i = 0; i < max_n; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            (void)imu_poll();
            buf[i * 6 + 0] = s_ax_raw;
            buf[i * 6 + 1] = s_ay_raw;
            buf[i * 6 + 2] = s_az_raw;
            buf[i * 6 + 3] = s_gx_raw;
            buf[i * 6 + 4] = s_gy_raw;
            buf[i * 6 + 5] = s_gz_raw;
        }
        imu_stop();

        const uint32_t got_a = s_accel_count - a0, got_g = s_gyro_count - g0;
        printf("\n%lu accel and %lu gyro samples arrived for %lu ticks\n",
               (unsigned long)got_a, (unsigned long)got_g,
               (unsigned long)max_n);

        FILE *wf = fopen("/fs/root/gesture.bin", "wb");
        if (!wf)
        {
            printf("imu: cannot write /root/gesture.bin\n");
            free(buf);
            return 1;
        }
        const uint8_t hdr[8] = {'G', 'E', 'S', 'T',
                                (uint8_t)(max_n & 0xFF),
                                (uint8_t)((max_n >> 8) & 0xFF),
                                (uint8_t)((max_n >> 16) & 0xFF),
                                (uint8_t)((max_n >> 24) & 0xFF)};
        const size_t body = (size_t)max_n * 6 * sizeof(int16_t);
        const size_t wrote_h = fwrite(hdr, 1, sizeof(hdr), wf);
        const size_t wrote_b = fwrite(buf, 1, body, wf);
        fclose(wf);
        free(buf);
        if (wrote_h != sizeof(hdr) || wrote_b != body)
        {
            printf("imu: short write (%u + %u of %u + %u bytes)\n",
                   (unsigned)wrote_h, (unsigned)wrote_b,
                   (unsigned)sizeof(hdr), (unsigned)body);
            return 1;
        }
        printf("wrote /root/gesture.bin: %u samples, %u bytes\n",
               (unsigned)max_n, (unsigned)(sizeof(hdr) + body));
        if (got_a < max_n / 2 || got_g < max_n / 2)
            printf("\nWARNING: fewer than half the expected samples arrived -- the\n"
                   "  streams were not running at 100 Hz and this recording is\n"
                   "  mostly repeated values. Re-record before replaying.\n");
        printf("\nNow: imu inject 30 5 112 0   (replays this recording)\n");
        return 0;
    }

    if (strcmp(sub, "boot") == 0)
        return cmd_imu_boot(argc >= 3 && strcmp(argv[2], "upload") == 0);

    /* WHAT UNIT IS THE PART'S CLOCK IN?
     *
     * The injected timestamp is a DELTA (bhy2.c adds it to a running counter),
     * and this command has been writing an absolute counter into it -- 10000,
     * 20000, ... reset once it passes 65000, so it sawtooths every seventh
     * block. Fixing that needs the unit, the unit is in no header this port
     * vendors, and measuring it inside an injected run is circular because the
     * injected deltas are the thing in question.
     *
     * So measure it from the physical accelerometer alone: run it at a known
     * rate, take the SDK's parsed timestamp for consecutive samples, and the
     * unit falls out of (mean delta per sample) against (20 ms per sample). */
    if (strcmp(sub, "clock") == 0)
    {
        if (!imu_start())
        {
            printf("imu: %s\n", s_why);
            return 1;
        }
        s_accel_ts_n = 0;
        if (!imu_accel_enable(50.0f))
        {
            printf("imu: accelerometer would not start\n");
            return 1;
        }
        for (int i = 0; i < 400 && s_accel_ts_n < 12; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            (void)imu_poll();
        }
        imu_stop();
        const uint8_t n = s_accel_ts_n;
        printf("accel at 50 Hz, %u timestamps\n", (unsigned)n);
        if (n < 3)
        {
            printf("imu: not enough samples; is the accelerometer streaming?\n");
            return 1;
        }
        uint64_t prev = s_accel_ts[1];
        printf("parsed ts:");
        for (uint8_t i = 0; i < n; i++)
            printf(" %llu", (unsigned long long)s_accel_ts[i]);
        printf("\n");
        /* Consecutive deltas, skipping any that are zero or wrap. */
        uint64_t sum = 0;
        uint32_t cnt = 0;
        for (uint8_t i = 2; i < n; i++)
        {
            const uint64_t d = s_accel_ts[i] - s_accel_ts[i - 1];
            if (d > 0 && d < 100000000ull)
            {
                sum += d;
                cnt++;
            }
        }
        (void)prev;
        if (!cnt)
        {
            printf("imu: every consecutive delta was zero or wild\n");
            return 1;
        }
        const double mean = (double)sum / (double)cnt;
        printf("mean delta %llu units over %lu intervals\n",
               (unsigned long long)(sum / cnt), (unsigned long)cnt);
        /* The configured rate is 50 Hz, so consecutive samples are 20 ms apart. */
        printf("20 ms = %.0f units  =>  %.0f units per second"
               "  =>  1 unit = %.3f us\n",
               mean, mean * 50.0, 20000.0 / mean);
        printf("\nFor a 10 ms injected block the delta to write is %.0f units.\n",
               mean / 2.0);
        return 0;
    }

    /* The sine feeder. Before the s_ready gate for symmetry with `boot`; it
     * starts the part itself if needed. */
    /* Data injection into KLIO. Before the s_ready gate, and it starts the part
     * itself, because the point is a controlled experiment from a cold board. */
    if (strcmp(sub, "inject") == 0 && argc >= 2)
    {
        const int secs   = argc >= 3 ? atoi(argv[2]) : 30;
        const int shp    = argc >= 4 ? atoi(argv[3]) : 0;
        const int target = argc >= 5 ? atoi(argv[4]) : BHY2_SENSOR_ID_KLIO;
        const int phys   = argc >= 6 ? atoi(argv[5]) : 1;
        /* SIX SHAPES BECAME THREE, AND THE OLD `shp == 1 ? 1 : 0` SILENTLY
         * TURNED EVERY OTHER VALUE INTO THE BASELINE. Adding shape 2 would have
         * run the 1.5 s cycle again and reported it as a longer rest, which is
         * the exact class of quiet substitution this project keeps paying for.
         * Only 0, 1 and 2 exist; anything else is refused rather than mapped. */
        if (shp < 0 || shp > 5)
        {
            printf("imu inject <seconds> [shape] [sensor] [phys]\n"
                   "  shape 0 = gesture, 0.9 s swing + 0.6 s rest (baseline)\n"
                   "  shape 1 = 1 Hz sine control\n"
                   "  shape 2 = gesture, 0.9 s swing + 1.5 s rest\n"
                   "  shape 3 = shape 0 replayed from a table, cycles bit-identical\n"
                   "  shape 4 = shape 0 with the gyro on X, the axis the accel implies\n"
                   "  shape 5 = replay /root/gesture.bin from `imu record`\n"
                   "  phys  1 = accel+gyro at 50 Hz, 0 = both off (use 0)\n");
            return 1;
        }
        return cmd_imu_inject(secs < 5 ? 5 : (secs > 120 ? 120 : secs),
                              shp, target, phys ? 1 : 0);
    }

    if (strcmp(sub, "feed") == 0)
    {
        const int secs = argc >= 3 ? atoi(argv[2]) : 10;
        const float fhz = argc >= 4 ? (float)atof(argv[3]) : 1.0f;
        const float amp = argc >= 5 ? (float)atof(argv[4]) : 0.5f;
        const int md = argc >= 6 ? atoi(argv[5]) : 2;
        const int eid = argc >= 7 ? atoi(argv[6]) : 4;
        return cmd_imu_feed(secs < 1 ? 1 : (secs > 120 ? 120 : secs),
                            fhz <= 0.0f ? 1.0f : fhz,
                            amp <= 0.0f ? 0.5f : amp,
                            md == 1 ? 1 : 2, eid < 0 ? 4 : eid);
    }

    if (!s_ready)
    {
        if (!imu_start())
        {
            printf("imu: %s\n", imu_why());
            return 1;
        }
        printf("imu: %s\n", s_version);
        printf("     firmware streamed lazily from %s+0x%x\n",
               IMU_FW_LABEL, (unsigned)IMU_FW_OFFSET);
    }

    /* STATUS IS THE DEFAULT; `imu accel` asks for samples.
     *
     * This split is not cosmetic. On the current state of the driver the
     * sensor-configuration call blocks, and a blocking call inside the shell's
     * own task takes the terminal with it -- Ctrl+C does not reach a command
     * that never returns. So the default path touches nothing that can hang,
     * and the one that can is asked for by name. */
    if (strcmp(sub, "rate") == 0)
    {
        /* WHAT RATE DOES IT ACTUALLY DELIVER? A burst command cannot answer
         * that -- it stops at the first sample -- and "it returned a number" is
         * not the same as "it keeps returning numbers". This polls hard for a
         * fixed window and reports the count, which is the figure a window
         * redrawing at 4 Hz needs to know. */
        if (!s_ready && !imu_start())
        {
            printf("imu: %s\n", imu_why());
            return 1;
        }
        if (!s_sensor_on && !imu_accel_enable(100.0f))
        {
            printf("imu: could not start the accelerometer\n");
            return 1;
        }
        const uint32_t before = s_accel_count;
        const int64_t t0 = esp_timer_get_time();
        int polls = 0;
        while (esp_timer_get_time() - t0 < 2000000)
        {
            imu_poll();
            polls++;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        const int64_t dt = esp_timer_get_time() - t0;
        const uint32_t got = s_accel_count - before;
        printf("%.1f Hz  (%lu samples / %d ms, %d polls)\n",
               got * 1000000.0 / (double)dt, (unsigned long)got,
               (int)(dt / 1000), polls);

        /* THE RATIO THAT SETTLES IT. An accelerometer packet is 6 bytes of
         * payload plus a sensor id and a timestamp, so samples can never exceed
         * bytes/7. If the sample count is larger than the bytes the part said
         * were waiting, the parser is re-reading the same bytes -- which also
         * explains identical values repeated many times in a row. */

        /* STOP THE SENSOR WHEN NOTHING IS READING IT. A part left streaming into
         * a FIFO nobody drains builds a backlog at 13 bytes per sample -- a
         * minute of that is 78 KB, far past any buffer, and the next command to
         * touch it is the one that wedges. The window polls continuously while
         * it is open; a one-shot command must leave the part quiet. */
        (void)imu_accel_enable(0.0f);
        return 0;
    }

    /* ------------------------------------------------------- inject ---- */

    /* CAN ARBITRARY IMU DATA BE FED TO THE PART, OR ONLY THE SENSOR'S OWN?
     *
     * This is the question behind treating the BHI260AP as a general-purpose
     * accelerator rather than a gesture toy: if the host can push a stream of
     * samples at it, then a classifier on the Fuser core can be trained from a
     * FILE -- recorded motion, or synthesised motion that never happened --
     * instead of only from a hand.
     *
     * THE SDK EXPOSES THE COMMANDS AND DOCUMENTS NO PAYLOAD. bhy2_defs.h has
     * BHY2_CMD_SET_INJECT_MODE and BHY2_CMD_INJECT_DATA, three modes, and no
     * struct describing what to send. An earlier attempt sent 8 bytes (a uint16
     * sensor id and three int16 in 1/4096 g) and the part ACCEPTED the command
     * and ignored the data -- which is what a payload of the wrong length looks
     * like when the firmware validates loosely rather than refusing.
     *
     * SO THIS SWEEPS THE SHAPES THAT ARE PLAUSIBLE, in one run rather than one
     * per flash: the 8-byte short form that failed before, the 28-byte FLOAT
     * form the algorithm's own log frame uses (timestamp, accel[3], gyro[3]),
     * the same with a sensor-id prefix, a timestamped short form, and the
     * wakeup accelerometer in case KLIO consumes that flavour.
     *
     * THE TEST VALUE IS +2.50 g ON Z, which gravity cannot physically produce,
     * so a match cannot be the real sensor agreeing by accident. */
    if (strcmp(sub, "inject") == 0)
    {
        if (!s_ready)
        {
            printf("imu: sensor not started\n");
            return 1;
        }
        /* `imu inject <n>` runs ONE shape. With six shapes in a sweep, a change
         * in the error register cannot be attributed to any of them; one shape
         * per run is what makes 0x2e evidence rather than aftermath. */
        int want = 0;
        if (argc >= 3)
        {
            want = atoi(argv[2]);
            if (want < 1 || want > 6)
            {
                printf("imu inject [1..6]\n");
                return 1;
            }
        }

        (void)imu_accel_enable(50.0f);
        status_reset();

        float bx = 0, by = 0, bz = 0;
        uint32_t bn = 0;
        for (int i = 0; i < 40 && !imu_accel(&bx, &by, &bz, &bn); i++)
        {
            vTaskDelay(pdMS_TO_TICKS(25));
            imu_poll();
        }
        printf("baseline z %+.2f g; injecting +2.50 g\n\n", (double)bz);

        int landed = 0;
        int tried = 0;

        /* ---- the 28-byte float frame, exactly as the algorithm sees it ---- */
        {
            float tmp[7] = {0.0f, 0.0f, 0.0f, 2.50f, 0.0f, 0.0f, 0.0f};
            uint8_t pay[28];
            memcpy(pay, tmp, sizeof(pay));

            if (!want || want == 1)
            {
                tried++;
                if (try_inject("28B float frame", pay, sizeof(pay),
                               BHY2_STEP_BY_STEP_INJECTION, bx, by, &bz))
                    landed++;
            }

            /* The same 28 bytes behind a uint16 sensor id: KLIO_LOG. */
            uint8_t pay30[30];
            pay30[0] = (uint8_t)(BHY2_SENSOR_ID_KLIO_LOG & 0xFF);
            pay30[1] = 0;
            memcpy(pay30 + 2, pay, sizeof(pay));
            if (!want || want == 2)
            {
                tried++;
                if (try_inject("30B id+float frame", pay30, sizeof(pay30),
                               BHY2_STEP_BY_STEP_INJECTION, bx, by, &bz))
                    landed++;
            }
        }

        /* ---- the short forms ---- */
        {
            uint8_t p8[8] = {0};
            uint16_t id = BHY2_SENSOR_ID_ACC;
            int16_t v[3] = {0, 0, (int16_t)(2.50f * 4096.0f)};
            memcpy(p8, &id, 2);
            memcpy(p8 + 2, v, 6);
            if (!want || want == 3)
            {
                tried++;
                if (try_inject("8B id+i16 (recheck)", p8, sizeof(p8),
                               BHY2_STEP_BY_STEP_INJECTION, bx, by, &bz))
                    landed++;
            }

            uint8_t p10[10];
            uint16_t ts = 0;
            memcpy(p10, &id, 2);
            memcpy(p10 + 2, &ts, 2);
            memcpy(p10 + 4, v, 6);
            if (!want || want == 4)
            {
                tried++;
                if (try_inject("10B id+ts+i16", p10, sizeof(p10),
                               BHY2_STEP_BY_STEP_INJECTION, bx, by, &bz))
                    landed++;
            }

            uint8_t p8w[8];
            id = BHY2_SENSOR_ID_ACC_WU;
            memcpy(p8w, &id, 2);
            memcpy(p8w + 2, v, 6);
            if (!want || want == 5)
            {
                tried++;
                if (try_inject("8B ACC_WU+i16", p8w, sizeof(p8w),
                               BHY2_STEP_BY_STEP_INJECTION, bx, by, &bz))
                    landed++;
            }
        }

        /* ---- and the plausible winner again, in real-time mode ---- */
        {
            float tmp[7] = {0.0f, 0.0f, 0.0f, 2.50f, 0.0f, 0.0f, 0.0f};
            uint8_t pay[28];
            memcpy(pay, tmp, sizeof(pay));
            if (!want || want == 6)
            {
                tried++;
                if (try_inject("28B float, real-time", pay, sizeof(pay),
                               BHY2_REAL_TIME_INJECTION, bx, by, &bz))
                    landed++;
            }
        }

        printf("\n%d of %d shapes landed.\n", landed, tried);

        /* THE ANSWER THIS EXPERIMENT EXISTS FOR. Entry 51 concluded from the
         * line above alone, and that line only ever measured whether the value
         * moved -- it could not tell "refused" from "accepted and ignored". The
         * status codes separate those, and `s_status_silent` bounds the
         * conclusion: if the part said nothing at all to six exchanges, then the
         * commands are not reaching a handler that answers, whatever else is
         * true. */
        printf("status channel: %d responses, %d exchanges with no response\n",
               s_status_total, s_status_silent);

        for (int i = 0; i < s_status_n; i++)
            printf("  0x%04x %-24s x%d\n", (unsigned)s_status_code[i],
                   status_name(s_status_code[i]), s_status_hits[i]);

        bool saw_conf_req = false, saw_cmd_err = false;
        for (int i = 0; i < s_status_n; i++)
        {
            if (s_status_code[i] == BHY2_STATUS_INJECT_SENSOR_CONF_REQ)
                saw_conf_req = true;
            if (s_status_code[i] == BHY2_STATUS_CMD_ERR)
                saw_cmd_err = true;
        }

        if (landed)
            printf("\nARBITRARY DATA REACHES THE PART -- a classifier here can be\n"
                   "trained from a file, not only from a hand.\n");
        else if (saw_conf_req)
            printf("\nTHE PART ASKED FOR A SENSOR CONFIGURATION (0x0004). Injection\n"
                   "is ALIVE in this firmware and waiting for a handshake this\n"
                   "driver never implemented -- it wants to be told which sensor's\n"
                   "data is being supplied. That is the next thing to build.\n");
        else if (saw_cmd_err)
            printf("\nTHE PART ANSWERED CMD ERR (0x000f). Injection is refused, not\n"
                   "merely ignored, on this firmware. The route to feeding it a\n"
                   "file is our own code on the Fuser core.\n");
        else if (s_status_total == 0)
            printf("\nSILENCE. Six exchanges, no status response at all, and the\n"
                   "value never moved. The commands are written to CHAN_CMD and\n"
                   "nothing answers them -- consistent with a firmware built\n"
                   "without the injection handler, and NOT the same evidence as\n"
                   "a refusal.\n");
        else
            printf("\nNone landed. The statuses above are the whole answer the part\n"
                   "gave; `meta 11 sensor error` in the log would mean a shape was\n"
                   "parsed and rejected.\n");

        /* PUT THE INTERFACE BACK. bhy2_hif_set_inject_data_mode() -- the SDK's
         * own helper behind bhy2_set_data_injection_mode() -- reads the host
         * interface control register, CLEARS the async status bit, sends the
         * mode command, and never restores it. Every call to it, including the
         * NORMAL_MODE one that ends each attempt above, leaves the part with
         * asynchronous status reporting switched off for the rest of the
         * session. Measured: reg 0x06 read 0x00 after a sweep, where boot leaves
         * it at 0x80. That is an SDK defect, not this driver's, and it is
         * invisible until something depends on async status. */
        (void)bhy2_set_data_injection_mode(BHY2_NORMAL_MODE, &s_bhy2);
        (void)bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &s_bhy2);
        (void)imu_accel_enable(0.0f);
        return landed ? 0 : 1;
    }

    /* Read the part's own registers. "Dig around in this chip" needs a window
     * that is not the API: everything above goes through the SDK, and the SDK
     * is exactly what is in question. These are the documented registers, one
     * byte each, read straight off the I2C bus. */
    if (strcmp(sub, "reg") == 0)
    {
        if (argc >= 3)
        {
            char *end = NULL;
            const unsigned long a = strtoul(argv[2], &end, 16);
            if (!end || *end || a > 0x3F)
            {
                printf("imu reg <hex 00..3f>\n");
                return 1;
            }
            uint8_t v = 0;
            const int8_t rc = bhy2_get_regs((uint8_t)a, &v, 1, &s_bhy2);
            printf("reg 0x%02lx = 0x%02x (%u) rc %d\n", a, v, v, rc);
            return 0;
        }

        static const struct { uint8_t addr; const char *name; } regs[] = {
            {0x1C, "product id"},   {0x1D, "revision id"},  {0x1E, "rom version"},
            {0x20, "kernel ver"},   {0x22, "user version"}, {0x24, "feature status"},
            {0x25, "boot status"},  {0x2B, "chip id"},      {0x2D, "int status"},
            {0x2E, "error value"},  {0x2F, "error aux"},    {0x30, "debug value"},
            {0x31, "debug state"},  {0x06, "hif ctrl"},     {0x17, "host status"},
        };
        for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
        {
            uint8_t v = 0;
            if (bhy2_get_regs(regs[i].addr, &v, 1, &s_bhy2) != BHY2_OK)
            {
                printf("  0x%02x %-14s <read failed>\n", regs[i].addr, regs[i].name);
                continue;
            }
            printf("  0x%02x %-14s 0x%02x  %3u\n", regs[i].addr, regs[i].name, v, v);
        }
        return 0;
    }

    /* POSITIVE CONTROL, and the reason it is not optional.
     *
     * "The status channel said nothing to 24 commands" is only evidence if the
     * status channel can be shown to say something at all. bhy2_get_parameter()
     * is the one call in this SDK that waits for and reads a status response --
     * exactly the plumbing status_read_one() borrows -- so a parameter read that
     * returns its payload proves the reading is right, and a parameter read that
     * hangs or returns nothing would mean the instrument is broken and the
     * silence above means nothing.
     *
     * `imu param 0900` is KLIO's algorithm version string (page 9, id 0), which
     * is a value that has to come from the part: there is no local copy. */
    if (strcmp(sub, "param") == 0)
    {
        char *end = NULL;
        const unsigned long id = strtoul(argc >= 3 ? argv[2] : "900", &end, 16);
        if (!end || *end || id > 0xFFFF)
        {
            printf("imu param <hex id>, e.g. 0900 = KLIO algorithm version\n");
            return 1;
        }

        uint8_t buf[128] = {0};
        uint32_t got = 0;
        const int8_t rc = bhy2_get_parameter((uint16_t)id, buf, sizeof(buf), &got,
                                             &s_bhy2);
        printf("param 0x%04lx rc %d, %lu B\n", id, rc, (unsigned long)got);
        if (rc == BHY2_OK && got)
        {
            printf("  hex:");
            for (uint32_t i = 0; i < got && i < 24; i++)
                printf(" %02x", buf[i]);
            printf("\n  str: ");
            for (uint32_t i = 0; i < got && i < 48; i++)
                putchar(buf[i] >= 32 && buf[i] < 127 ? buf[i] : '.');
            printf("\n");
        }
        return rc == BHY2_OK ? 0 : 1;
    }

    /* DOES INJECTED DATA EVER APPEAR ON THE SENSOR'S OWN OUTPUT?
     *
     * The cheapest question that decides the whole idea, and the one that does
     * not depend on guessing a framing correctly: inject a value gravity cannot
     * produce -- +8 g on Z, four times a hand's weight -- and watch the
     * accelerometer's virtual-sensor output while it is being injected.
     *
     * The test is a RANGE, not a reading. The part is asked for a new sample
     * many times a second and each one is recorded; if any sample anywhere in
     * the window comes near 8 g then injected bytes reached the virtual sensor,
     * and the earlier "it does not work" results were a framing problem. If the
     * range across thousands of samples stays inside the physical sensor's own
     * envelope, injection does not reach the sensor path at all.
     *
     * WHY THIS IS BETTER THAN WATCHING KLIO. KLIO reports progress, which is a
     * summary: it could stay at zero for a dozen reasons -- rate, threshold,
     * segmentation. The accelerometer reports the sample itself, so a negative
     * result here is about the data path and nothing else.
     *
     * THE SENSOR RUNS THROUGHOUT, on purpose. For injection to replace a value
     * the part has to have a virtual sensor to replace it in, so stopping the
     * accelerometer first would test something else. What is being asked is
     * whether injection OVERRIDES a configured sensor's output.
     */
    if (strcmp(sub, "watch") == 0)
    {
        int seconds = argc >= 3 ? atoi(argv[2]) : 15;
        const int shape = argc >= 4 ? atoi(argv[3]) : 4;
        if (seconds < 3 || seconds > 120)
            seconds = 15;

        if (!imu_accel_enable(50.0f))
        {
            printf("imu: could not start the accelerometer\n");
            return 1;
        }

        /* Baseline first, with nothing injected, so the physical sensor's own
         * envelope is measured rather than assumed. */
        float bmin = 99, bmax = -99;
        const uint32_t n0 = s_accel_count;
        for (int i = 0; i < 50; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            (void)imu_poll();
            float x, y, z;
            uint32_t n;
            if (imu_accel(&x, &y, &z, &n))
            {
                if (z < bmin) bmin = z;
                if (z > bmax) bmax = z;
            }
        }
        printf("baseline: z %.2f .. %.2f g over %lu samples\n", (double)bmin,
               (double)bmax, (unsigned long)(s_accel_count - n0));

        const int mode = (shape == 2 || shape == 4) ? BHY2_STEP_BY_STEP_INJECTION
                                                   : BHY2_REAL_TIME_INJECTION;
        if (!imu_inject_begin(mode))
        {
            printf("imu: the part refused injection mode\n");
            (void)imu_accel_enable(0.0f);
            return 1;
        }

        /* +8 g on Z: 8 * 4096 = 32768, which is exactly int16 range, so 30000
         * (7.32 g) is used to stay clear of the clamp. */
        const uint8_t acc_id = (shape == 4) ? 76 : (shape == 2 ? 55 : 4);
        int16_t big[3] = {0, 0, 30000};
        uint8_t pkt[14];
        uint16_t plen;

        if (shape == 1)
        {
            float f[7] = {0.0f, 0.0f, 7.32f, 0.0f, 0.0f, 0.0f, 0.0f};
            memcpy(pkt, f, sizeof(f));
            plen = (uint16_t)sizeof(f);
        }
        else if (shape == 2 || shape == 4)
        {
            pkt[0] = acc_id;
            memcpy(pkt + 1, big, 6);
            plen = 7;
        }
        else
        {
            pkt[0] = 4;
            pkt[1] = 0;
            plen = 2;
        }

        float imin = 99, imax = -99;
        uint32_t sent = 0, reads = 0;
        int near = 0;
        const uint32_t n1 = s_accel_count;
        const int64_t t0 = esp_timer_get_time();

        while (esp_timer_get_time() - t0 < (int64_t)seconds * 1000000)
        {
            if (imu_inject_frame(pkt, plen))
                sent++;

            /* Poll fast while injecting. The point is to catch the injected
             * sample in the output, and a slow poll would let it pass between
             * two reads. */
            for (int k = 0; k < 3; k++)
            {
                vTaskDelay(pdMS_TO_TICKS(5));
                (void)imu_poll();
                float x, y, z;
                uint32_t n;
                if (imu_accel(&x, &y, &z, &n))
                {
                    reads++;
                    if (z < imin) imin = z;
                    if (z > imax) imax = z;
                    if (z > 4.0f) near++;
                }
            }
        }

        imu_inject_end();

        const uint32_t got = s_accel_count - n1;
        printf("injecting %.2f g on Z for %d s: %lu frames sent\n", 7.32, seconds,
               (unsigned long)sent);
        printf("  sensor samples during: %lu\n", (unsigned long)got);
        printf("  z during injection:    %.2f .. %.2f g  (%lu reads)\n",
               (double)imin, (double)imax, (unsigned long)reads);
        printf("  samples above 4 g:     %d\n", near);

        if (near > 0)
            printf("\nINJECTION REACHES THE VIRTUAL SENSOR OUTPUT. Real.\n");
        else if (got == 0)
            printf("\nNo sensor samples either: injection suppressed the sensor\n"
                   "without replacing it, so the value never appears anywhere.\n");
        else
            printf("\nNo sample reached 4 g in %lu frames of +7.32 g injection.\n"
                   "Injection does not override or feed the virtual sensor.\n",
                   (unsigned long)got);

        (void)imu_accel_enable(0.0f);
        return 0;
    }

    /* Send a bare command id, with an optional payload, and then look at the
     * error register and the status channel.
     *
     * WHY. "The part ignores the inject command" and "the part does not know
     * the inject command" are different claims, and the second one is testable
     * without knowing anything about injection: send a command id that
     * certainly does not exist and compare what the part records. If a made-up
     * id and BHY2_CMD_INJECT_DATA leave the same error, the command is simply
     * not in this firmware. If they differ, injection is a real command being
     * rejected for a stated reason, and the reason is worth chasing.
     *
     *   imu cmd ffee            a command id that cannot exist
     *   imu cmd 0008            BHY2_CMD_INJECT_DATA, no payload
     *   imu cmd 0008 00000000   the same with a four-byte payload
     */
    if (strcmp(sub, "cmd") == 0)
    {
        char *end = NULL;
        const unsigned long id = strtoul(argc >= 3 ? argv[2] : "ffee", &end, 16);
        if (!end || *end || id > 0xFFFF)
        {
            printf("imu cmd <hex id> [payload hex]\n");
            return 1;
        }

        uint8_t pkt[64] = {0};
        uint16_t plen = 0;

        if (argc >= 4)
        {
            const char *h = argv[3];
            size_t n = strlen(h);
            if (n % 2 || n / 2 > 60)
            {
                printf("payload must be an even number of hex digits, <= 60\n");
                return 1;
            }
            for (size_t i = 0; i < n; i += 2)
            {
                char b[3] = {h[i], h[i + 1], 0};
                pkt[4 + plen++] = (uint8_t)strtoul(b, NULL, 16);
            }
        }

        pkt[0] = (uint8_t)(id & 0xFF);
        pkt[1] = (uint8_t)((id >> 8) & 0xFF);
        pkt[2] = (uint8_t)(plen & 0xFF);
        pkt[3] = (uint8_t)((plen >> 8) & 0xFF);

        uint8_t err_before = 0, err_after = 0, hif = 0;
        (void)bhy2_get_regs(BHY2_REG_ERROR_VALUE, &err_before, 1, &s_bhy2);

        /* Async status OFF for the exchange, exactly as the sweep does, so a
         * status response lands somewhere it can be read. */
        imu_status_ctx_t ctx;
        status_reset();
        status_sync_begin(&ctx);

        const int8_t rc = bhy2_set_regs(BHY2_REG_CHAN_CMD, pkt, 4 + plen, &s_bhy2);
        const bool said = status_read_one(200);
        status_sync_end(&ctx);

        (void)bhy2_get_regs(BHY2_REG_ERROR_VALUE, &err_after, 1, &s_bhy2);
        (void)bhy2_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &hif, 1, &s_bhy2);

        printf("cmd 0x%04lx +%u B  write rc %d\n", id, (unsigned)plen, rc);
        printf("  error 0x%02x -> 0x%02x   status %s", err_before, err_after,
               said ? "said" : "silent");
        if (said)
        {
            printf(" 0x%04x %s", (unsigned)s_last_code, status_name(s_last_code));
            for (uint32_t i = 0; i < s_last_len; i++)
                printf(" %02x", s_last_payload[i]);
        }
        printf("\nhif ctrl 0x%02x%s\n", hif,
               (hif & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL) ? " (async restored)"
                                                          : " (ASYNC OFF!)");
        return 0;
    }

    /* Raw bytes off an I2C register, including the FIFO channels. "Dig around in
     * this chip" needs one window that goes under the SDK entirely: if the API
     * says nothing is there, this says whether the bytes are really absent or
     * whether the layer above is swallowing them.
     *
     *   0x03 status FIFO   0x02 non-wakeup FIFO   0x01 wakeup FIFO
     *   0x2D int status    0x2E error value       0x31 debug state */
    if (strcmp(sub, "raw") == 0)
    {
        char *end = NULL;
        const unsigned long a = strtoul(argc >= 3 ? argv[2] : "2d", &end, 16);
        if (!end || *end || a > 0x3F)
        {
            printf("imu raw <hex reg> [len], e.g. 2d = int status\n");
            return 1;
        }
        int len = 16;
        if (argc >= 4)
        {
            len = atoi(argv[3]);
            if (len < 1 || len > 255)
                len = 250;
        }

        /*
         * THE FIFO CHANNELS ARE NOT PLAIN REGISTERS AND A BLIND READ WEDGES THE
         * BUS. Measured the expensive way: `imu raw 02 16` against an empty
         * non-wakeup FIFO hung the shell task and the board needed a hard reset.
         *
         * The protocol is Bosch's, from bhy2_hif_get_status_fifo(): read FOUR
         * bytes and they are a header -- a 16-bit status code (or FIFO byte
         * count, depending on the channel) and a 16-bit count of the bytes that
         * remain. Only then is a second read of exactly that length legal. A
         * read of 16 bytes when the part has 0 to give leaves the transaction
         * unfinished.
         *
         * So channels 0x01, 0x02 and 0x03 are read as header-then-payload, and
         * everything else as a plain register.
         */
        const bool is_fifo = (a == BHY2_REG_CHAN_FIFO_W ||
                              a == BHY2_REG_CHAN_FIFO_NW ||
                              a == BHY2_REG_CHAN_STATUS);

        if (is_fifo)
        {
            /* THE TWO KINDS OF FIFO HAVE DIFFERENT HEADERS and reading four
             * bytes from a data FIFO takes two payload bytes as if they were a
             * length -- which is exactly how a 0-byte FIFO came to be reported
             * as 251 bytes waiting, and how a dump of it came back all zeros.
             *
             *   status channel 0x03: 4 bytes -- 16-bit status code, then length
             *   wakeup 0x01 (and 0x02):  2 bytes -- 16-bit byte count only
             *
             * bhy2_hif_get_fifo() reads two; bhy2_hif_get_status_fifo() reads
             * four. This follows each. */
            const bool is_status_chan = (a == BHY2_REG_CHAN_STATUS);
            uint8_t hdr[4] = {0};
            const BHY2_INTF_RET_TYPE hrc =
                bus_read((uint8_t)a, hdr, is_status_chan ? 4 : 2, s_bhy2.hif.intf_ptr);

            uint16_t word = 0;
            uint32_t remain = 0;
            if (is_status_chan)
            {
                word = (uint16_t)(hdr[0] | (hdr[1] << 8));
                remain = (uint32_t)(hdr[2] | (hdr[3] << 8));
            }
            else
            {
                remain = (uint32_t)(hdr[0] | (hdr[1] << 8));
            }

            printf("fifo 0x%02lx hdr rc %d: %s%lu B waiting\n", a, (int)hrc,
                   is_status_chan ? "" : "", (unsigned long)remain);

            if (remain == 0)
            {
                printf("  empty -- nothing queued\n");
                return 0;
            }

            /* NEVER READ A PARTIAL FIFO. The count in the header is a promise
             * about how many bytes the next read will consume; taking fewer
             * leaves the rest queued and every later read starts mid-packet.
             * Measured the hard way: after one capped read the header went on to
             * report 64,768 bytes waiting, which is nonsense and was this. So a
             * FIFO larger than the buffer is reported and NOT read. */
            if (remain > (uint32_t)len)
            {
                printf("  %lu B waiting, more than the %d-byte buffer -- not read\n"
                       "  (a partial read desynchronises the stream)\n",
                       (unsigned long)remain, len);
                return 0;
            }

            static uint8_t buf[256];
            memset(buf, 0, sizeof(buf));
            const BHY2_INTF_RET_TYPE rc = bus_read((uint8_t)a, buf, remain,
                                                   s_bhy2.hif.intf_ptr);
            printf("  read rc %d\n", (int)rc);
            for (uint32_t i = 0; i < remain; i++)
            {
                if (i % 16 == 0)
                    printf("  %02x:", (unsigned)i);
                printf(" %02x", buf[i]);
                if (i % 16 == 15)
                    printf("\n");
            }
            printf("\n");
            return 0;
        }

        uint8_t buf[60] = {0};
        const BHY2_INTF_RET_TYPE rc = bus_read((uint8_t)a, buf, (uint32_t)len,
                                               s_bhy2.hif.intf_ptr);
        printf("raw reg 0x%02lx x%d rc %d\n", a, len, (int)rc);
        for (int i = 0; i < len; i++)
        {
            if (i % 16 == 0)
                printf("  %02x:", i);
            printf(" %02x", buf[i]);
            if (i % 16 == 15)
                printf("\n");
        }
        printf("\n");
        return 0;
    }

    /* Listen to the status channel with nothing else going on. The same
     * synchronous read the injection probe uses, available on its own so a
     * question about the part does not have to be wrapped in an experiment. */
    if (strcmp(sub, "status") == 0)
    {
        imu_status_ctx_t ctx;
        status_reset();
        status_sync_begin(&ctx);
        const bool got = status_read_one(400);
        status_sync_end(&ctx);
        if (!got)
        {
            printf("nothing waiting. A quiet part is the normal state; a code here\n"
                   "means something in the firmware wants the host's attention.\n");
            return 1;
        }
        printf("0x%04x %s", (unsigned)s_last_code, status_name(s_last_code));
        for (uint32_t i = 0; i < s_last_len; i++)
            printf(" %02x", s_last_payload[i]);
        printf("\n");
        return 0;
    }

    if (strcmp(sub, "scan") == 0)
    {
        if (s_present_n < 0)
            s_present_n = imu_list_sensors(s_present, sizeof(s_present));
        printf("firmware carries %d virtual sensors:\n", s_present_n);
        printf("  %s\n", s_present);
        printf("names: 61 pickup-gesture 63 ar 67 wrist-tilt-gesture\n");
        printf("       69 device-orientation 70 device-ori-wakeup\n");
        printf("       75 stationary-detect 77 motion-detect\n");
        return 0;
    }

    if (strcmp(sub, "accel") != 0)
    {
        printf("accel: %s (use `imu accel` for samples)\n",
               s_sensor_on ? "on" : "off");
        return 0;
    }

    if (!s_sensor_on && !imu_accel_enable(100.0f))
    {
        printf("imu: could not start the accelerometer\n");
        return 1;
    }

    /* Give it a few sample periods, then drain until something arrives. The
     * first packet after enabling is never immediate. */
    float x = 0, y = 0, z = 0;
    uint32_t n = 0;
    bool have = false;
    int polls = 0, fired = 0;
    for (int i = 0; i < 40 && !have; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(25));
        polls++;
        if (imu_poll())
            fired++;
        have = imu_accel(&x, &y, &z, &n);
    }
    ESP_LOGI(TAG, "%d polls, %d drained, %lu samples", polls, fired,
             (unsigned long)n);

    if (!have)
    {
        printf("imu: no samples yet -- the accelerometer is still filling\n");
        return 1;
    }

    /* A BURST, NOT A SNAPSHOT. One reading cannot be distinguished from a
     * frozen register; five taken as they arrive show that the numbers are
     * moving, which is the only evidence that means anything from a sensor. */
    /* THE SAME WORD THE WINDOW SHOWS, from the same function, so the two cannot
     * disagree about what the watch is doing. */
    {
        td_imu_t post;
        memset(&post, 0, sizeof(post));
        post.accel_valid = true;
        post.ax = x;
        post.ay = y;
        post.az = z;
        printf("posture %s\n", imu_posture(&post));
    }
    printf("accel, in g (a watch lying flat reads about +1.00 on one axis):\n");
    printf("  x %+6.2f   y %+6.2f   z %+6.2f\n", x, y, z);
    for (int i = 0; i < 4; i++)
    {
        /* `n` MUST BE REFRESHED INSIDE THIS LOOP. The first version compared the
         * sample count without ever updating it, so the wait always timed out and
         * the burst printed one line -- while `imu rate` on the same sensor
         * measured 97.5 Hz. A stale loop variable is not evidence about the
         * device. */
        const int64_t until = esp_timer_get_time() + 400000;
        const uint32_t before = n;
        while (esp_timer_get_time() < until)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            imu_poll();
            imu_accel(&x, &y, &z, &n);
            if (n != before)
                break;
        }
        if (n == before)
            break;          /* nothing new in 400 ms: stop rather than pad */
        printf("  x %+6.2f   y %+6.2f   z %+6.2f\n", x, y, z);
    }
    printf("%lu samples so far\n", (unsigned long)n);
    (void)imu_accel_enable(0.0f);      /* leave the part quiet; see `imu rate` */
    return 0;
}

static const tdsh_command_t s_imu_cmd = {
    "imu",
    "imu [accel|rate|scan|inject|stop]",
    "The BHI260AP: uploads its firmware on first use, then shows acceleration",
    cmd_imu,
    TDSH_CMD_INTERACTIVE,
};

bool imu_register_command(void)
{
    return tdsh_register_command(&s_imu_cmd) == 0;
}
