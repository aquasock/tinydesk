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
#include "tinydesk/td_sysinfo.h"
#include "imu.h"
#include "power.h"

static const char *TAG = "imu";

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
static volatile bool s_have_accel;
static float s_ax, s_ay, s_az;
static uint32_t s_accel_count;


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
    int16_t s[3];
    memcpy(s, info->data_ptr, 6);
    s_ax = s[0] / 4096.0f;
    s_ay = s[1] / 4096.0f;
    s_az = s[2] / 4096.0f;
    s_accel_count++;
    s_have_accel = true;
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
static void on_meta(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;
    if (info->data_size < 1)
        return;
    const uint8_t ev = info->data_ptr[0];
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
        ESP_LOGW(TAG, "meta %u (%s) sensor %u", ev, name,
                 info->data_size > 1 ? info->data_ptr[1] : 0);
    else
        ESP_LOGD(TAG, "meta %u (%s) sensor %u", ev, name,
                 info->data_size > 1 ? info->data_ptr[1] : 0);
}

/* ---------------------------------------------------------- start/stop */

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

bool imu_start(void)
{
    if (s_ready)
        return true;
    s_why[0] = '\0';

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

    if (!fw_upload_and_boot())
        return false;

    s_ready = true;
    imu_version(s_version, sizeof(s_version));
    ESP_LOGI(TAG, "BHI260AP up: %s", s_version);
    return true;
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

/* ------------------------------------------------------------- the command */

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
    "imu [accel|rate|scan|stop]",
    "The BHI260AP: uploads its firmware on first use, then shows acceleration",
    cmd_imu,
    TDSH_CMD_INTERACTIVE,
};

bool imu_register_command(void)
{
    return tdsh_register_command(&s_imu_cmd) == 0;
}
