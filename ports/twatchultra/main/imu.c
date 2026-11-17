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

/* How much FIFO to drain per poll. The part's FIFO is a few hundred bytes and
 * 100 Hz on one sensor fills it slowly. */
#define IMU_FIFO_MAX   512

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
static char s_why[112];
static char s_version[40];
static uint8_t s_fifo[IMU_FIFO_MAX];

/* The most recent acceleration, written by the callback and read by
 * imu_accel(). Single producer (this task) and single consumer, so a plain
 * volatile pair of flags is enough; there is no second task polling the part. */
static volatile bool s_have_accel;
static float s_ax, s_ay, s_az;
static uint32_t s_accel_count;

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

    /* Status packets on the async channel; the SDK's FIFO parser expects it. */
    (void)bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, &s_bhy2);

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

/* ---------------------------------------------------------- start/stop */

const char *imu_why(void)
{
    return s_why;
}

bool imu_ready(void)
{
    return s_ready;
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

    int8_t rc = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, hz, 0, &s_bhy2);
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

/* ------------------------------------------------------------- the command */

static int cmd_imu(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    const bool stop = argc >= 2 && argv[1][0] == 'x';

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
    if (!(argc >= 2 && argv[1][0] == 'a'))
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
    for (int i = 0; i < 40 && !have; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(25));
        imu_poll();
        have = imu_accel(&x, &y, &z, &n);
    }

    if (!have)
    {
        printf("imu: no samples yet -- the accelerometer is still filling\n");
        return 1;
    }

    printf("accel  x %+6.2f  y %+6.2f  z %+6.2f g   (%lu samples)\n",
           x, y, z, (unsigned long)n);
    printf("       flat on a table, one axis reads about +-1.00\n");
    return 0;
}

static const tdsh_command_t s_imu_cmd = {
    "imu",
    "imu [accel|stop]",
    "The BHI260AP: uploads its firmware on first use, then shows acceleration",
    cmd_imu,
    TDSH_CMD_INTERACTIVE,
};

bool imu_register_command(void)
{
    return tdsh_register_command(&s_imu_cmd) == 0;
}
