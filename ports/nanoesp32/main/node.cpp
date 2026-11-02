/*
 * node.cpp - WiFi and Reticulum, running beside the desktop.
 *
 * The split matters. microReticulum's UDPInterface brings up WiFi itself
 * through the Arduino WiFi library, but this firmware is ESP-IDF, so `#ifdef
 * ARDUINO` is false and that block is skipped: the interface falls through to
 * its POSIX socket path, which lwIP serves. WiFi is therefore brought up here,
 * before the interface starts, and the interface is left to do only sockets.
 *
 * Everything Reticulum does runs in its own task. The desktop redraws the
 * panel at 30 frames a second and must never wait on an announce, and
 * Reticulum must not be starved by a window drag.
 *
 * Credentials live in /fs/etc/wifi.conf so they can be written from the
 * desktop's Editor or the Terminal's shell -- there is no setup wizard in
 * this firmware, and a board that needs a cable to be provisioned is a board
 * that needs a cable.
 */
#include "node.h"

#include "lora.h"
#include "tang_link.h"
#include "sensors.h"

#include <stdio.h>
#include "tdsh.h"

/* tdsh_bridge_esp.h declares this without an extern "C" guard, and this file
 * is C++, so including it yields a mangled name that never links. Declared
 * here instead of editing a header shared with the C sources. */
extern "C" bool tdsh_bridge_break_requested(void);

/* True once the console is writing to USB. Declared here rather than pulled
 * from link.h. */
extern "C" bool link_usb_is_console(void);

/* Any key typed during a long command: 0 none, 1 Ctrl+C, 2 other. */
extern "C" int link_any_key(void);

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <math.h>
#include <stdlib.h>
#include <memory>
#include <string>
#include <time.h>

#include <microReticulum.h>
#include <microReticulum/Transport.h>
#include <LXMF/LXMRouter.h>
#include <LXMF/LXMessage.h>
#include <microReticulum/Utilities/OS.h>

/* microStore, for the prefixed filesystem adapter below. */
#include <microStore/FileSystem.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "UDPInterface.h"

#include "esp_random.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include <sys/stat.h>

static const char *TAG = "node";

#define WIFI_CONF     "/fs/etc/wifi.conf"
/* Paths handed to microStore are relative to the mount: the MountedFileSystem
 * prefixes "/fs", so "/identity" lands on /fs/identity. An absolute "/fs/..."
 * would become "/fs/fs/..." and silently fail to write. */
#define IDENTITY_FILE "/identity"
#define RETICULUM_DIR "/reticulum"

/* TinyNode announces on a random 90-300 minute schedule. Same policy here:
 * a fixed interval makes every node in earshot announce together. */
#define ANNOUNCE_MIN_S (90u * 60u)
#define ANNOUNCE_MAX_S (300u * 60u)

/* The LXMF router: what gives the node an address peers can message. */
static LXMF::LXMRouter *s_lxmf;
static bool s_lxmf_up;
static uint32_t s_messages_rx;

static volatile node_state_t s_state = NODE_OFF;
static char s_identity_hex[33];
static char s_ip[16];
static struct node_stats s_stats;
static volatile bool s_persist_requested;
static volatile bool s_announce_requested;

/* Whether a telemetry reply carries LOCATION and MAGNETIC_FIELD as well as the
 * environment triad. Set by `node telemetry full|compact`. Default true: see
 * lxmf_send_telemetry() for why the bigger map is the default. */
static volatile bool s_telemetry_full = true;

static void node_log(const char *fmt, ...);

/* ---- WiFi ---------------------------------------------------------- */

/* Read "ssid = value" and "password = value" from the config file. Returns
 * false if either is missing, which is what an unprovisioned board looks
 * like. */
static bool read_wifi_conf(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    FILE *fp = fopen(WIFI_CONF, "r");
    if (!fp)
    {
        /* Build-time fallback, for a bench image that has no way to write a
         * file yet: -DTD_WIFI_SSID=... -DTD_WIFI_PASSWORD=...
         * Unset in a normal build, where the file is the only source. */
#if defined(TD_WIFI_SSID)
        snprintf(ssid, ssid_len, "%s", TD_WIFI_SSID);
#ifdef TD_WIFI_PASSWORD
        snprintf(pass, pass_len, "%s", TD_WIFI_PASSWORD);
#else
        pass[0] = '\0';
#endif
        node_log("wifi from build-time default (%s)", WIFI_CONF);
        return true;
#else
        ESP_LOGW(TAG, "no %s", WIFI_CONF);
        return false;
#endif
    }
    ssid[0] = pass[0] = '\0';
    char line[128];
    while (fgets(line, sizeof(line), fp))
    {
        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        /* trim */
        while (*key == ' ' || *key == '\t')
            key++;
        char *ke = key + strlen(key);
        while (ke > key && (ke[-1] == ' ' || ke[-1] == '\t'))
            *--ke = '\0';
        while (*val == ' ' || *val == '\t')
            val++;
        char *ve = val + strlen(val);
        while (ve > val && (ve[-1] == '\n' || ve[-1] == '\r' || ve[-1] == ' '))
            *--ve = '\0';

        if (strcmp(key, "ssid") == 0)
            snprintf(ssid, ssid_len, "%s", val);
        else if (strcmp(key, "password") == 0)
            snprintf(pass, pass_len, "%s", val);
    }
    fclose(fp);
    return ssid[0] != '\0';
}

/* Why an association failed matters: 201 is "no AP found" (wrong SSID or
 * band), 15 is a handshake timeout (wrong password), and the two need
 * different fixes. */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        node_log("wifi disconnect reason=%d", d->reason);
    }
}

static bool wifi_start(const char *ssid, const char *password)
{
    /* The desktop's shared app_main already calls these; both are idempotent
     * but return ESP_ERR_INVALID_STATE on a second call, so the result is
     * only fatal when it is something else. */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return false;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return false;
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event, NULL);

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    node_log("wifi: netif=%p", (void *)netif);
    if (!netif)
        return false;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t e = esp_wifi_init(&cfg);
    node_log("wifi: init=%s", esp_err_to_name(e));
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
        return false;

    /* memset rather than = {0}: wifi_config_t is a union of large structs and
     * -Wmissing-field-initializers rejects the brace form under -Werror. */
    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    snprintf((char *)wc.sta.ssid, sizeof(wc.sta.ssid), "%s", ssid);
    snprintf((char *)wc.sta.password, sizeof(wc.sta.password), "%s", password);

    e = esp_wifi_set_mode(WIFI_MODE_STA);
    node_log("wifi: set_mode=%s", esp_err_to_name(e));
    e = esp_wifi_set_config(WIFI_IF_STA, &wc);
    node_log("wifi: set_config=%s", esp_err_to_name(e));
    if (e != ESP_OK)
        return false;
    e = esp_wifi_start();
    node_log("wifi: start=%s", esp_err_to_name(e));
    /* set_config on a STA normally starts the association, but the desktop's
     * shell also owns a WiFi manager, so the connection is requested here
     * explicitly rather than relied upon. */
    /* Peak transmit current is what browns out a marginal supply, and this
     * board is also driving a 2.2" backlight. 8.5 dBm is plenty for a node
     * on the same LAN and roughly halves the peak against the default 20. */
    int8_t tx = 34;              /* 0.25 dBm units: 34 = 8.5 dBm */
    e = esp_wifi_set_max_tx_power(tx);
    node_log("wifi: max_tx=%s", esp_err_to_name(e));

    e = esp_wifi_connect();
    node_log("wifi: connect=%s", esp_err_to_name(e));

    /* Bounded wait: a wrong password must not hang the board before the
     * desktop is reachable to fix it. */
    for (int i = 0; i < 60; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (i % 10 == 9)
        {
            wifi_ap_record_t probe;
            node_log("wifi: t=%ds apinfo=%d", (i + 1) / 2,
                     esp_wifi_sta_get_ap_info(&probe) == ESP_OK);
        }
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        {
            s_stats.rssi_dbm = ap.rssi;
            esp_netif_ip_info_t ip;
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr)
            {
                snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ip.ip));
                node_log("state=wifi_up ip=%s rssi=%d", s_ip, ap.rssi);
                return true;
            }
        }
    }
    ESP_LOGW(TAG, "wifi did not associate");
    return false;
}

/* ---- Reticulum ------------------------------------------------------ */

static RNS::Reticulum s_reticulum({RNS::Type::NONE});
static RNS::Identity s_identity({RNS::Type::NONE});
static RNS::Interface s_udp({RNS::Type::NONE});
static bool s_udp_ready;
static bool s_announced_once;

/* ---- I2C bus ------------------------------------------------------- */

/* The sensors share A4/A5 (GPIO 11/12) with nothing else: the panel is on SPI
 * and the controller is not fitted in this build. One probe per address is
 * enough to inventory the bus, and logging every hit means a new sensor
 * shows up in the boot record rather than being discovered late. */
#define I2C_PORT_SENSORS I2C_NUM_0
#define I2C_SDA_GPIO     GPIO_NUM_11
#define I2C_SCL_GPIO     GPIO_NUM_12
#define I2C_HZ           100000

static bool s_i2c_ready;

static i2c_master_bus_handle_t s_i2c_bus;

static bool sensors_i2c_init(void)
{
    if (s_i2c_bus)
        return true;
    /* The modern master-bus API. The legacy driver/i2c.h path panicked on
     * this build (rst:0xc RTC_SW_CPU_RST, a software reset) and is deprecated
     * in IDF 5.3 anyway. */
    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };
    esp_err_t err = i2c_new_master_bus(&bus, &s_i2c_bus);
    if (err != ESP_OK)
    {
        node_log("i2c: bus init failed (%s)", esp_err_to_name(err));
        s_i2c_bus = NULL;
        return false;
    }
    return true;
}

/* Other translation units need the same bus handle; the driver is refcounted
 * by ESP-IDF for handles, so handing it out is safe. */
i2c_master_bus_handle_t sensors_bus(void)
{
    return s_i2c_bus;
}

/* Probe 0x08..0x77 and log every address that acknowledges.
 *
 * This is the plain form, and it is the one that reported the bus correctly
 * (0x1c 0x44 0x47). An "improved" version added a 2 ms settle before each
 * address and promptly reported 27-30 addresses that changed every pass; the
 * delay let the lines float long enough for the probe to see false ACKs. Do
 * not add pacing here. */
static int sensors_i2c_scan(void)
{
    if (!sensors_i2c_init())
        return -1;
    char found[160];
    int n = 0;
    size_t used = 0;
    found[0] = '\0';
    for (uint8_t a = 0x08; a < 0x78; a++)
    {
        if (i2c_master_probe(s_i2c_bus, a, 20) != ESP_OK)
            continue;
        n++;
        if (used < sizeof(found) - 8)
            used += (size_t)snprintf(found + used, sizeof(found) - used, "%s0x%02x",
                                     used ? " " : "", a);
    }
    node_log("i2c: %d device(s): %s", n, n ? found : "none");
    return n;
}

/* A bare address probe is not trustworthy here: on this bus it ACKed 27-30
 * addresses that changed on every pass, which is a floating-line artefact.
 * Reading a known ID register is definitive -- it only succeeds if a real
 * device answered with real data. */
static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *out)
{
    if (!s_i2c_bus)
        return false;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev) != ESP_OK)
        return false;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, out, 1, 100);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK;
}

/* Read the two bus lines directly. If SDA is held low by a device that
 * brown-outed or lost its supply, every other device goes silent too -- which
 * looks exactly like "nothing is fitted" and is not. */
static void i2c_line_state(const char *when)
{
    gpio_set_direction((gpio_num_t)I2C_SDA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)I2C_SCL_GPIO, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(2));
    int sda = gpio_get_level((gpio_num_t)I2C_SDA_GPIO);
    int scl = gpio_get_level((gpio_num_t)I2C_SCL_GPIO);
    node_log("i2c lines %s: SDA=%d SCL=%d (1/1 = idle and healthy)", when, sda, scl);
}

/* Free a bus that a device is holding low: clock SCL up to nine times with
 * SDA released, then issue a STOP. This is the standard recovery and it costs
 * nothing to try before concluding a device is missing. */
static void i2c_bus_recover(void)
{
    gpio_set_direction((gpio_num_t)I2C_SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)I2C_SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level((gpio_num_t)I2C_SDA_GPIO, 1);
    for (int i = 0; i < 9; i++)
    {
        gpio_set_level((gpio_num_t)I2C_SCL_GPIO, 0);
        esp_rom_delay_us(5);
        gpio_set_level((gpio_num_t)I2C_SCL_GPIO, 1);
        esp_rom_delay_us(5);
    }
    /* STOP: SDA low then high while SCL is high. */
    gpio_set_level((gpio_num_t)I2C_SDA_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level((gpio_num_t)I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_level((gpio_num_t)I2C_SDA_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_direction((gpio_num_t)I2C_SDA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)I2C_SCL_GPIO, GPIO_MODE_INPUT);
}

/* Confirm the bus is sane by reading the IDs of the devices that are known to
 * be fitted. If these read correctly, a silent address really is silent. */
static void sensors_identify(void)
{
    uint8_t v = 0;
    if (i2c_read_reg(0x1c, 0x0F, &v))
        node_log("i2c: LIS3MDL WHO_AM_I=0x%02x (expect 0x3d)", v);
    else
        node_log("i2c: LIS3MDL at 0x1c did not answer");

    if (i2c_read_reg(0x47, 0x01, &v))
        node_log("i2c: BMP581 chip_id=0x%02x (expect 0x50)", v);
    else
        node_log("i2c: BMP581 at 0x47 did not answer");

    if (i2c_read_reg(0x10, 0x00, &v))
        node_log("i2c: GPS at 0x10 answered, reg0=0x%02x", v);
    else
        node_log("i2c: GPS at 0x10 did not answer a register read");

    if (i2c_read_reg(0x52, 0xFA, &v))
        node_log("i2c: Wii adapter at 0x52 answered, id0=0x%02x (expect 0x00)", v);
    else
        node_log("i2c: Wii adapter at 0x52 did not answer");
}

/* The GPS is the one device that can be slow to appear: an MTK3333 does not
 * answer until its own boot completes, and the one-shot scan runs immediately.
 * Probe it repeatedly for the first half minute and log the first time it
 * answers, which separates "not fitted" from "late". */
static void gps_watch(uint8_t addr, int attempts, int delay_ms)
{
    if (!s_i2c_bus)
        return;
    for (int i = 0; i < attempts; i++)
    {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK)
        {
            node_log("i2c: 0x%02x answered on attempt %d (after %d ms)",
                     addr, i + 1, i * delay_ms);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    node_log("i2c: 0x%02x never answered in %d probes over %d ms",
             addr, attempts, attempts * delay_ms);
}

/* ---- storage adapter ------------------------------------------------ */

/* microStore needs a FileSystemImpl; Reticulum works in relative paths
 * (storagepath() is ".", the cache "./cache", the path store "./path_store/").
 * This adapter prefixes every path with the firmware's mount point, which is
 * the mechanism the standalone TinyNode uses with "/ffat".
 *
 * Setting Reticulum::storagepath() does NOT work: the constructor assigns "."
 * and "./cache" to the static arrays after any setter call, so the value is
 * gone before start() reads it.
 *
 * microStore ships a POSIX adapter, but it is written for Arduino: its header
 * includes LittleFS.h under #if defined(ESP32) and braces a struct stat in a
 * way this build's -Werror rejects. This is a small stdio implementation
 * instead -- only the calls Reticulum actually makes. */

class MountedFile : public microStore::FileImpl
{
  public:
    MountedFile(FILE *fp, const std::string &name)
        : _fp(fp), _name(name)
    {
    }
    ~MountedFile() override
    {
        close();
    }

  protected:
    const char *name() const override
    {
        return _name.c_str();
    }
    size_t size() const override
    {
        if (!_fp)
            return 0;
        long cur = ftell(_fp);
        fseek(_fp, 0, SEEK_END);
        long end = ftell(_fp);
        fseek(_fp, cur, SEEK_SET);
        return end > 0 ? (size_t)end : 0;
    }
    void close() override
    {
        if (_fp)
        {
            fclose(_fp);
            _fp = nullptr;
        }
    }
    int read() override
    {
        return _fp ? fgetc(_fp) : -1;
    }
    size_t write(uint8_t ch) override
    {
        return _fp && fputc(ch, _fp) != EOF ? 1 : 0;
    }
    size_t read(uint8_t *buf, size_t n) override
    {
        return _fp ? fread(buf, 1, n, _fp) : 0;
    }
    size_t write(const uint8_t *buf, size_t n) override
    {
        return _fp ? fwrite(buf, 1, n, _fp) : 0;
    }
    int available() override
    {
        if (!_fp)
            return 0;
        long cur = ftell(_fp), end;
        fseek(_fp, 0, SEEK_END);
        end = ftell(_fp);
        fseek(_fp, cur, SEEK_SET);
        return end > cur ? (int)(end - cur) : 0;
    }
    int peek() override
    {
        if (!_fp)
            return -1;
        int c = fgetc(_fp);
        if (c != EOF)
            ungetc(c, _fp);
        return c;
    }
    size_t tell() override
    {
        return _fp ? (size_t)ftell(_fp) : 0;
    }
    long seek(uint32_t pos, microStore::SeekMode mode) override
    {
        int whence = mode == microStore::SeekModeSet   ? SEEK_SET
                     : mode == microStore::SeekModeCur ? SEEK_CUR
                                                       : SEEK_END;
        return _fp && fseek(_fp, (long)pos, whence) == 0 ? (long)pos : -1;
    }
    void flush() override
    {
        if (_fp)
            fflush(_fp);
    }
    bool isValid() const override
    {
        return _fp != nullptr;
    }

  private:
    FILE *_fp;
    std::string _name;
};

class MountedFileSystem : public microStore::FileSystemImpl
{
  public:
    explicit MountedFileSystem(const char *mount)
        : _mount(mount)
    {
    }

    bool init(bool reformatOnFail = true) override
    {
        (void)reformatOnFail;
        return true;
    }

    microStore::File open(const char *path, microStore::File::Mode mode, const bool create = false) override
    {
        using M = microStore::File;
        const char *m = mode == M::ModeRead         ? "rb"
                        : mode == M::ModeWrite      ? (create ? "wb" : "r+b")
                        : mode == M::ModeAppend     ? "ab"
                        : mode == M::ModeReadAppend ? "a+b"
                                                    : "r+b";
        FILE *fp = fopen(full(path).c_str(), m);
        /* microStore's writeFile() calls open(ModeWrite) with create=false and
         * expects it to create; "r+b" fails on a missing file, so fall back to
         * creating. Without this, every Reticulum write -- the identity, the
         * path store, the transport identity -- silently returns zero bytes. */
        if (!fp && mode != M::ModeRead)
            fp = fopen(full(path).c_str(), "w+b");
        return microStore::File(fp ? new MountedFile(fp, path) : nullptr);
    }

    bool exists(const char *path) override
    {
        struct stat st;
        return stat(full(path).c_str(), &st) == 0;
    }
    bool remove(const char *path) override
    {
        return ::remove(full(path).c_str()) == 0;
    }
    bool rename(const char *from, const char *to) override
    {
        return ::rename(full(from).c_str(), full(to).c_str()) == 0;
    }
    bool mkdir(const char *path) override
    {
        return ::mkdir(full(path).c_str(), 0755) == 0;
    }
    bool rmdir(const char *path) override
    {
        return ::rmdir(full(path).c_str()) == 0;
    }
    bool isDirectory(const char *path) override
    {
        struct stat st;
        return stat(full(path).c_str(), &st) == 0 && S_ISDIR(st.st_mode);
    }

    std::list<std::string> listDirectory(const char *path,
                                         Callbacks::DirectoryListing callback = nullptr) override
    {
        std::list<std::string> out;
        DIR *d = opendir(full(path).c_str());
        if (!d)
            return out;
        struct dirent *e;
        while ((e = readdir(d)) != nullptr)
        {
            if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0)
                continue;
            out.push_back(e->d_name);
            if (callback)
                callback(e->d_name);
        }
        closedir(d);
        return out;
    }

    size_t storageSize() override
    {
        return 0;
    }
    size_t storageAvailable() override
    {
        return 0;
    }

  private:
    std::string full(const char *path) const
    {
        std::string p(path ? path : "");
        if (p.rfind("./", 0) == 0)
            p = p.substr(1);                  /* "./x" -> "/x" */
        else if (p.empty() || p[0] != '/')
            p = "/" + p;                      /* "x"    -> "/x" */
        return _mount + p;
    }

    std::string _mount;
};

static microStore::FileSystem s_filesystem;

/* A headless board has no console to read. This appends one line per state
 * change to /fs/node.log so the state can be recovered from flash after the
 * fact; it is also what the desktop reads for its status line. */
static void node_log(const char *fmt, ...)
{
    char line[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > (int)sizeof(line) - 2)
        n = (int)sizeof(line) - 2;
    line[n++] = '\n';

    /* To the file, for a board running headless. */
    FILE *fp = fopen("/fs/node.log", "a");
    if (fp)
    {
        fwrite(line, 1, (size_t)n, fp);
        fclose(fp);
    }

    /* Not to USB. The console owns this port: the shell reads its input from the
     * same FIFO and writes its screen to the same stream, and a log line
     * arriving between those lands in the middle of the operator's line. That
     * is not theoretical -- gating this on "the console has started drawing"
     * still let lines through from the node task, and "node:" turned up inside
     * typed input. Every line goes to /fs/node.log, which `cat node.log`
     * reads from the console. */
    (void)0;
}

/* ---- LXMF telemetry -------------------------------------------------- *
 *
 * Sideband drives a node through message fields, not chat text, so this is
 * the standard form of the same readings the `node` command prints:
 *
 *   FIELD_COMMANDS  (0x09) in   [ {COMMAND: argument}, ... ]
 *   FIELD_TELEMETRY (0x02) out  {SID: value} as one msgpack map
 *
 * The byte layout is not invented here -- it is what the client unpacks
 * (sbapp/sideband/sense.py, LXMF-007 to LXMF-009 in the reference). TIME is
 * an integer, LOCATION is a list of big-endian struct blobs that the client
 * scales back down (1e6, 1e6, 1e2, 1e2, 1e2, 1e2), RAM and NVM are
 * [[label, [capacity, used]]], MAGNETIC_FIELD is three floats, and pressure,
 * temperature and humidity are one float each.
 *
 * microLXMF stores field keys and values as raw msgpack and re-emits them
 * with packRawBytes(), so both halves of this map are packed here. Handing
 * the library a native value would nest a second msgpack object inside the
 * field and the client would see nothing it recognises.
 */

#define LXMF_FIELD_TELEMETRY 0x02
#define LXMF_FIELD_COMMANDS  0x09
#define LXMF_FIELD_ICON_APPEARANCE 0x04

#define LXMF_CMD_TELEMETRY_REQUEST 0x01
#define LXMF_CMD_PING              0x02
#define LXMF_CMD_ECHO              0x03

#define SID_TIME           0x01
#define SID_LOCATION       0x02
#define SID_PRESSURE       0x03
#define SID_TEMPERATURE    0x07
#define SID_HUMIDITY       0x08
#define SID_MAGNETIC_FIELD 0x09
#define SID_INFORMATION    0x0F
#define SID_RAM            0x14
#define SID_NVM            0x15
#define SID_RNS_TRANSPORT  0x19
#define SID_CUSTOM         0xFF

/* A msgpack writer over a caller-owned buffer. Every put fails soft and
 * latches ok=false, so a caller checks once at the end rather than after
 * every field. The buffer is fixed: a telemetry map is a few hundred bytes
 * and a heap allocation here would be a heap allocation in the delivery
 * callback, which is the last place that should fragment. */
struct Mpw
{
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool ok;

    void put(uint8_t b)
    {
        if (len < cap)
            buf[len++] = b;
        else
            ok = false;
    }
    void put_be(uint32_t v, int bytes)
    {
        for (int i = bytes - 1; i >= 0; --i)
            put((uint8_t)(v >> (8 * i)));
    }
    void put_u64(uint64_t v)
    {
        for (int i = 7; i >= 0; --i)
            put((uint8_t)(v >> (8 * i)));
    }
    void blob(const void *d, size_t n)
    {
        if (n == 0)
            return;
        if (len + n > cap)
        {
            ok = false;
            return;
        }
        memcpy(buf + len, d, n);
        len += n;
    }
};

static void mp_map(Mpw &w, unsigned n)
{
    if (n < 16)
        w.put((uint8_t)(0x80 | n));
    else
    {
        w.put(0xde);
        w.put_be(n, 2);
    }
}

static void mp_arr(Mpw &w, unsigned n)
{
    if (n < 16)
        w.put((uint8_t)(0x90 | n));
    else
    {
        w.put(0xdc);
        w.put_be(n, 2);
    }
}

/* Sensor ids are all below 0x80, so this writes the bare byte that is both
 * the fixint encoding of the id and the map key the client expects. */
static void mp_uint(Mpw &w, uint32_t v)
{
    if (v < 0x80)
        w.put((uint8_t)v);
    else
    {
        w.put(0xce);
        w.put_be(v, 4);
    }
}

static void mp_f64(Mpw &w, double d)
{
    uint64_t bits;
    memcpy(&bits, &d, sizeof(bits));
    w.put(0xcb);
    w.put_u64(bits);
}

static void mp_bin(Mpw &w, const void *d, size_t n)
{
    if (n < 256)
    {
        w.put(0xc4);
        w.put((uint8_t)n);
    }
    else
    {
        w.put(0xc5);
        w.put_be((uint32_t)n, 2);
    }
    w.blob(d, n);
}

static void mp_str(Mpw &w, const char *s)
{
    size_t n = strlen(s);
    if (n < 32)
        w.put((uint8_t)(0xa0 | n));
    else
    {
        w.put(0xd9);
        w.put((uint8_t)n);
    }
    w.blob(s, n);
}

static void mp_bool(Mpw &w, bool v)
{
    w.put(v ? 0xc3 : 0xc2);
}

/* The fixed-width big-endian blobs LOCATION is made of. */
static void mp_be32_bin(Mpw &w, int32_t v)
{
    uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    mp_bin(w, b, sizeof(b));
}

static void mp_be16_bin(Mpw &w, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    mp_bin(w, b, sizeof(b));
}

/* A msgpack reader, just wide enough to walk a FIELD_COMMANDS list. */
struct Mpr
{
    const uint8_t *p;
    const uint8_t *end;
};

static bool mp_take(Mpr &r, size_t n)
{
    if ((size_t)(r.end - r.p) < n)
    {
        r.p = r.end;
        return false;
    }
    r.p += n;
    return true;
}

static bool mp_read_be(Mpr &r, size_t n, uint64_t *out)
{
    if ((size_t)(r.end - r.p) < n)
    {
        r.p = r.end;
        return false;
    }
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i)
        v = (v << 8) | r.p[i];
    r.p += n;
    *out = v;
    return true;
}

static bool mp_skip_items(Mpr &r, uint32_t n);
static bool mp_skip_pairs(Mpr &r, uint32_t n);
static bool mp_skip(Mpr &r);

static bool mp_skip_items(Mpr &r, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
        if (!mp_skip(r))
            return false;
    return true;
}

static bool mp_skip_pairs(Mpr &r, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
        if (!mp_skip(r) || !mp_skip(r))
            return false;
    return true;
}

/* Skip one object of any type. Needed because a command we do not implement
 * still has an argument that has to be stepped over, not guessed at. */
static bool mp_skip(Mpr &r)
{
    if (r.p >= r.end)
        return false;
    uint8_t b = *r.p++;
    if (b <= 0x7f || b >= 0xe0)
        return true;                                    /* fixint */
    if ((b & 0xf0) == 0x80)
        return mp_skip_pairs(r, b & 0x0f);              /* fixmap */
    if ((b & 0xf0) == 0x90)
        return mp_skip_items(r, b & 0x0f);              /* fixarray */
    if ((b & 0xe0) == 0xa0)
        return mp_take(r, b & 0x1f);                    /* fixstr */

    uint64_t n = 0;
    switch (b)
    {
    case 0xc0: case 0xc2: case 0xc3: return true;       /* nil, bool */
    case 0xc4: if (!mp_read_be(r, 1, &n)) return false; return mp_take(r, (size_t)n);
    case 0xc5: if (!mp_read_be(r, 2, &n)) return false; return mp_take(r, (size_t)n);
    case 0xc6: if (!mp_read_be(r, 4, &n)) return false; return mp_take(r, (size_t)n);
    case 0xc7: if (!mp_read_be(r, 1, &n)) return false; return mp_take(r, (size_t)n + 1);
    case 0xc8: if (!mp_read_be(r, 2, &n)) return false; return mp_take(r, (size_t)n + 1);
    case 0xc9: if (!mp_read_be(r, 4, &n)) return false; return mp_take(r, (size_t)n + 1);
    case 0xca: return mp_take(r, 4);                    /* float32 */
    case 0xcb: return mp_take(r, 8);                    /* float64 */
    case 0xcc: case 0xd0: return mp_take(r, 1);
    case 0xcd: case 0xd1: return mp_take(r, 2);
    case 0xce: case 0xd2: return mp_take(r, 4);
    case 0xcf: case 0xd3: return mp_take(r, 8);
    case 0xd4: return mp_take(r, 2);                    /* fixext */
    case 0xd5: return mp_take(r, 3);
    case 0xd6: return mp_take(r, 5);
    case 0xd7: return mp_take(r, 9);
    case 0xd8: return mp_take(r, 17);
    case 0xd9: if (!mp_read_be(r, 1, &n)) return false; return mp_take(r, (size_t)n);
    case 0xda: if (!mp_read_be(r, 2, &n)) return false; return mp_take(r, (size_t)n);
    case 0xdb: if (!mp_read_be(r, 4, &n)) return false; return mp_take(r, (size_t)n);
    case 0xdc: if (!mp_read_be(r, 2, &n)) return false; return mp_skip_items(r, (uint32_t)n);
    case 0xdd: if (!mp_read_be(r, 4, &n)) return false; return mp_skip_items(r, (uint32_t)n);
    case 0xde: if (!mp_read_be(r, 2, &n)) return false; return mp_skip_pairs(r, (uint32_t)n);
    case 0xdf: if (!mp_read_be(r, 4, &n)) return false; return mp_skip_pairs(r, (uint32_t)n);
    default: return false;
    }
}

/* Read one integer of any width. Rewinds on failure so the caller can fall
 * back to mp_skip() without losing a byte. */
static bool mp_read_int(Mpr &r, int64_t *out)
{
    const uint8_t *save = r.p;
    if (r.p >= r.end)
        return false;
    uint8_t b = *r.p++;
    if (b <= 0x7f)
    {
        *out = b;
        return true;
    }
    if (b >= 0xe0)
    {
        *out = (int8_t)b;
        return true;
    }

    size_t n;
    bool sign = false;
    switch (b)
    {
    case 0xcc: n = 1; break;
    case 0xcd: n = 2; break;
    case 0xce: n = 4; break;
    case 0xcf: n = 8; break;
    case 0xd0: n = 1; sign = true; break;
    case 0xd1: n = 2; sign = true; break;
    case 0xd2: n = 4; sign = true; break;
    case 0xd3: n = 8; sign = true; break;
    default:
        r.p = save;
        return false;
    }

    uint64_t v = 0;
    if (!mp_read_be(r, n, &v))
    {
        r.p = save;
        return false;
    }
    if (!sign)
        *out = (int64_t)v;
    else if (n == 1)
        *out = (int8_t)v;
    else if (n == 2)
        *out = (int16_t)v;
    else if (n == 4)
        *out = (int32_t)v;
    else
        *out = (int64_t)v;
    return true;
}

/* Read the payload of a bin or str object. Rewinds on failure, like
 * mp_read_int(). */
static bool mp_read_blob(Mpr &r, const uint8_t **d, size_t *n)
{
    const uint8_t *save = r.p;
    if (r.p >= r.end)
        return false;
    uint8_t b = *r.p++;
    uint64_t len = 0;

    if ((b & 0xe0) == 0xa0)
        len = b & 0x1f;
    else if (b == 0xc4 || b == 0xd9)
    {
        if (!mp_read_be(r, 1, &len))
        {
            r.p = save;
            return false;
        }
    }
    else if (b == 0xc5 || b == 0xda)
    {
        if (!mp_read_be(r, 2, &len))
        {
            r.p = save;
            return false;
        }
    }
    else if (b == 0xc6 || b == 0xdb)
    {
        if (!mp_read_be(r, 4, &len))
        {
            r.p = save;
            return false;
        }
    }
    else
    {
        r.p = save;
        return false;
    }

    if ((size_t)(r.end - r.p) < (size_t)len)
    {
        r.p = save;
        return false;
    }
    *d = r.p;
    *n = (size_t)len;
    r.p += len;
    return true;
}

/* Read an array or map header. */
static bool mp_read_count(Mpr &r, bool is_map, uint32_t *out)
{
    const uint8_t *save = r.p;
    if (r.p >= r.end)
        return false;
    uint8_t b = *r.p++;
    uint64_t n = 0;

    if (is_map && (b & 0xf0) == 0x80)
    {
        *out = b & 0x0f;
        return true;
    }
    if (!is_map && (b & 0xf0) == 0x90)
    {
        *out = b & 0x0f;
        return true;
    }
    if (is_map && (b == 0xde || b == 0xdf))
    {
        if (!mp_read_be(r, b == 0xde ? 2 : 4, &n))
        {
            r.p = save;
            return false;
        }
        *out = (uint32_t)n;
        return true;
    }
    if (!is_map && (b == 0xdc || b == 0xdd))
    {
        if (!mp_read_be(r, b == 0xdc ? 2 : 4, &n))
        {
            r.p = save;
            return false;
        }
        *out = (uint32_t)n;
        return true;
    }
    r.p = save;
    return false;
}

struct node_commands
{
    bool telemetry;
    bool ping;
    bool echo;
    RNS::Bytes echo_text;
};

/* FIELD_COMMANDS is a list of single-key dicts, one command each. An entry
 * we cannot read is stepped over rather than aborting the list: an unknown
 * command should not cost us the requests we do understand. */
static void parse_commands(const RNS::Bytes &field, struct node_commands *out)
{
    out->telemetry = false;
    out->ping = false;
    out->echo = false;
    out->echo_text.clear();

    Mpr r = {field.data(), field.data() + field.size()};
    uint32_t items = 0;
    if (!mp_read_count(r, false, &items))
        return;

    for (uint32_t i = 0; i < items; ++i)
    {
        uint32_t pairs = 0;
        if (!mp_read_count(r, true, &pairs))
        {
            if (!mp_skip(r))
                return;
            continue;
        }
        for (uint32_t j = 0; j < pairs; ++j)
        {
            int64_t key = -1;
            if (!mp_read_int(r, &key))
            {
                if (!mp_skip(r) || !mp_skip(r))
                    return;
                continue;
            }
            if (key == LXMF_CMD_ECHO)
            {
                const uint8_t *d = nullptr;
                size_t n = 0;
                if (mp_read_blob(r, &d, &n))
                {
                    out->echo = true;
                    out->echo_text = RNS::Bytes(d, n);
                }
                else if (!mp_skip(r))
                    return;
            }
            else
            {
                if (key == LXMF_CMD_TELEMETRY_REQUEST)
                    out->telemetry = true;
                else if (key == LXMF_CMD_PING)
                    out->ping = true;
                if (!mp_skip(r))
                    return;
            }
        }
    }
}

/* One line identifying this node, for the client's Information row.
 *
 * The client renders SID_INFORMATION as a single copyable string, so this is
 * the place a remote operator learns what they are talking to without
 * guessing from telemetry: firmware version, how long it has been up, and the
 * address to reach it on. Firmware version comes from the app descriptor the
 * build embeds, not from a literal here, so it cannot drift from the image. */
static void info_string(char *out, size_t n)
{
    char up[24];
    const uint32_t s = (uint32_t)(esp_timer_get_time() / 1000000);
    if (s >= 86400)
        snprintf(up, sizeof up, "%ud%02uh", (unsigned)(s / 86400),
                 (unsigned)((s % 86400) / 3600));
    else if (s >= 3600)
        snprintf(up, sizeof up, "%uh%02um", (unsigned)(s / 3600),
                 (unsigned)((s % 3600) / 60));
    else
        snprintf(up, sizeof up, "%um%02us", (unsigned)(s / 60),
                 (unsigned)(s % 60));

    const esp_app_desc_t *d = esp_app_get_description();
    snprintf(out, n, "TinyNode Nano, tinydesk %s, up %s, ip %s",
             d ? d->version : "?", up, s_ip[0] ? s_ip : "none");
}

/* Pack the current readings as a FIELD_TELEMETRY map. Returns the length
 * written, or 0 if the buffer was too small. */
/* compact=true builds a map small enough to travel in one encrypted packet
 * (packed_size <= 159), which needs no path to the peer and therefore works
 * the moment a request arrives. compact=false is the full report, which has
 * to go over a link. */
static size_t telemetry_pack(uint8_t *buf, size_t cap, bool compact)
{
    const struct sensor_snapshot *sn = sensors_latest();
    const uint32_t now = (uint32_t)time(nullptr);

    /* The two numbers a remote operator otherwise cannot see. Internal 8-bit
     * heap, not PSRAM: that is the pool Reticulum and the desktop both draw
     * from for small allocations. */
    const size_t heap_total = heap_caps_get_total_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    const size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    size_t fs_total = 0, fs_used = 0;
    if (esp_littlefs_info(TDSH_PARTITION_LABEL, &fs_total, &fs_used) != ESP_OK)
        fs_total = fs_used = 0;

    unsigned entries = compact ? 1 : 3;  /* TIME, and RAM + RNS_TRANSPORT */
    if (!compact)
        entries++;                       /* INFORMATION */
    if (!compact && sn->gps_fix)
        entries++;
    if (sn->have_pressure)
        entries++;
    if (sn->have_env)
        entries += 2;                    /* temperature and humidity */
    if (!compact && sn->have_mag)
        entries++;
    if (!compact && sn->have_heading)
        entries++;                       /* CUSTOM: magnetic heading */
    if (!compact && fs_total > 0)
        entries++;                       /* NVM */

    Mpw w = {buf, cap, 0, true};
    mp_map(w, entries);

    /* The client validates this one and drops the whole map when it is not
     * a number, so it is always written, even at 1970. */
    mp_uint(w, SID_TIME);
    mp_uint(w, now);

    /* One copyable line saying what this node is. Cheap -- well under 80
     * bytes -- and it answers the question a new operator has before any
     * telemetry means anything. Full only, to keep the compact map inside one
     * packet. */
    if (!compact)
    {
        char info[128];
        info_string(info, sizeof info);
        mp_uint(w, SID_INFORMATION);
        mp_str(w, info);
    }

    if (!compact && sn->gps_fix)
    {
        /* Position and altitude come from the fix. Speed and course are
         * measured values, taken from RMC fields 7 and 8 rather than the
         * hardcoded zeros this used to send. They still read zero when the
         * receiver reported nothing, because the array is positional: the
         * client indexes packed[0] through packed[6] inside a bare except, so
         * a shorter array would silently discard the whole sensor. A
         * stationary receiver omits course entirely, which is not the same as
         * course zero, and this format cannot express that difference.
         * Accuracy is HDOP times an assumed range error -- an approximation
         * this project chooses, since the receiver measures HDOP and not
         * metres -- so it is labelled derived rather than measured. */
        mp_uint(w, SID_LOCATION);
        mp_arr(w, 7);
        mp_be32_bin(w, (int32_t)llround(sn->gps_lat * 1e6));
        mp_be32_bin(w, (int32_t)llround(sn->gps_lon * 1e6));
        mp_be32_bin(w, (int32_t)llround(sn->gps_alt_m * 1e2));
        mp_be32_bin(w, (int32_t)llround(sn->gps_speed_mps * 1e2));   /* cm/s */
        mp_be32_bin(w, (int32_t)llround(sn->gps_course_deg * 1e2));  /* 1e-2 deg */
        {
            double cm = sn->gps_accuracy_m * 1e2;
            if (cm < 0.0) cm = 0.0;
            if (cm > 65535.0) cm = 65535.0;   /* the field is uint16 */
            mp_be16_bin(w, (uint16_t)llround(cm));
        }
        mp_uint(w, sn->gps_unix ? sn->gps_unix : now);
    }

    if (sn->have_pressure)
    {
        mp_uint(w, SID_PRESSURE);
        mp_f64(w, sn->pressure_hpa);
    }
    if (sn->have_env)
    {
        mp_uint(w, SID_TEMPERATURE);
        mp_f64(w, sn->temperature_c);
        mp_uint(w, SID_HUMIDITY);
        mp_f64(w, sn->humidity_pct);
    }
    if (!compact && sn->have_mag)
    {
        mp_uint(w, SID_MAGNETIC_FIELD);
        mp_arr(w, 3);
        mp_f64(w, sn->mag_x_ut);
        mp_f64(w, sn->mag_y_ut);
        mp_f64(w, sn->mag_z_ut);
    }

    /* Heading goes in CUSTOM, not in MAGNETIC_FIELD or LOCATION.
     *
     * The client's sensor ids have no heading anywhere in them, and the two
     * slots that look like candidates both refuse it. MAGNETIC_FIELD is exactly
     * three floats: pack() writes [x, y, z], unpack() reads packed[0..2], and a
     * fourth element is silently dropped. LOCATION does carry a field called
     * `bearing` that Location.render() renames to `heading`, but the UI only
     * prints it when speed exceeds 0.1 km/h -- so a stationary node, which is
     * the only kind this board will ever be, would never show it, and the slot
     * means GPS course over ground anyway.
     *
     * CUSTOM is a list of [label, [value, icon]] and the client renders each
     * entry as its own row with that icon, moving or not. So the heading
     * reaches the operator labelled as what it is.
     *
     * Full only: the compact map is already 151 packed bytes against a
     * 159-byte single-packet limit, so roughly forty more would push it onto a
     * link and lose the property that makes compact worth having. */
    if (!compact && sn->have_heading)
    {
        char h[16];
        snprintf(h, sizeof h, "%.1f\xc2\xb0", sn->mag_heading_deg);
        mp_uint(w, SID_CUSTOM);
        mp_arr(w, 1);                    /* the list of entries */
        mp_arr(w, 2);                    /* one entry: [label, [value, icon]] */
        mp_str(w, "Magnetic heading");
        mp_arr(w, 2);
        mp_str(w, h);
        mp_str(w, "compass-rose");
    }

    /* Label 0x00 is the client's own "Storage" bucket. Capacity must be
     * non-zero: the renderer divides used by it. */
    if (!compact)
    {
    mp_uint(w, SID_RAM);
    mp_arr(w, 1);
    mp_arr(w, 2);
    mp_uint(w, 0);
    mp_arr(w, 2);
    mp_uint(w, (uint32_t)heap_total);
    mp_uint(w, (uint32_t)(heap_total - heap_free));

    if (!compact && fs_total > 0)
    {
        mp_uint(w, SID_NVM);
        mp_arr(w, 1);
        mp_arr(w, 2);
        mp_uint(w, 0);
        mp_arr(w, 2);
        mp_uint(w, (uint32_t)fs_total);
        mp_uint(w, (uint32_t)fs_used);
    }

    }   /* !compact */

    if (!compact)
    {
        RNS::Bytes ident = s_identity.hash();
        const uint32_t rxb = (uint32_t)RNS::Transport::traffic_rxb();
        const uint32_t txb = (uint32_t)RNS::Transport::traffic_txb();
        const uint32_t rxs = RNS::Transport::packets_received();
        const uint32_t txs = RNS::Transport::packets_sent();

        mp_uint(w, SID_RNS_TRANSPORT);
        mp_map(w, 14);
        mp_str(w, "transport_enabled");
        mp_bool(w, true);
        mp_str(w, "transport_identity");
        mp_bin(w, ident.data(), ident.size());
        mp_str(w, "transport_uptime");
        mp_uint(w, (uint32_t)(esp_timer_get_time() / 1000000));
        mp_str(w, "traffic_rxb");
        mp_uint(w, rxb);
        mp_str(w, "traffic_txb");
        mp_uint(w, txb);
        mp_str(w, "speed_rx");
        mp_f64(w, 0.0);
        mp_str(w, "speed_tx");
        mp_f64(w, 0.0);
        mp_str(w, "speed_rx_inst");
        mp_uint(w, rxs);
        mp_str(w, "speed_tx_inst");
        mp_uint(w, txs);
        mp_str(w, "memory_used");
        mp_uint(w, (uint32_t)(heap_total - heap_free));
        mp_str(w, "ifstats");
        mp_map(w, 6);
        mp_str(w, "rss");
        mp_uint(w, 0);
        mp_str(w, "rxb");
        mp_uint(w, rxb);
        mp_str(w, "txb");
        mp_uint(w, txb);
        mp_str(w, "rxs");
        mp_uint(w, rxs);
        mp_str(w, "txs");
        mp_uint(w, txs);
        /* Interface list, enumerated from the live table rather than written
         * by hand. It was a single hardcoded UDP entry before LoRa existed, so
         * once the radio registered the map still claimed one interface and
         * Sideband showed a transport with a single link when there were two. */
        const RNS::Transport::InterfaceTable &ifaces =
            RNS::Transport::get_interfaces();
        mp_str(w, "interfaces");
        mp_arr(w, (unsigned)ifaces.size());
        for (const RNS::Interface &ifc : ifaces)
        {
            const std::string nm = ifc.name();
            /* The client keys its interface dictionary by this name, so it
             * has to be distinct per interface and stable. */
            char short_name[16];
            size_t sn = 0;
            if (!nm.empty() && nm[0] == 'U') sn = 3;              /* "UDP" */
            else if (nm.rfind("RYLR", 0) == 0) sn = 4;            /* "RYLR" */
            if (sn == 0)
            {
                for (size_t i = 0; i < nm.size() && sn + 1 < sizeof(short_name); ++i)
                    short_name[sn++] = nm[i];
            }
            else
                std::memcpy(short_name, nm.c_str(), sn);
            short_name[sn] = '\0';

            /* Sideband maps a handful of type strings to icons and falls back
             * to a generic one for anything else, so an accurate name it does
             * not know is better than a familiar name that is a lie. */
            const char *type = "RYLR998Interface";
            if (nm.find("UDP") != std::string::npos)
                type = "UDPInterface";

            mp_map(w, 8);
            mp_str(w, "name");
            mp_str(w, nm.c_str());
            mp_str(w, "short_name");
            mp_str(w, short_name);
            mp_str(w, "status");
            mp_bool(w, ifc.online());
            mp_str(w, "mode");
            mp_uint(w, (uint32_t)ifc.mode());
            mp_str(w, "type");
            mp_str(w, type);
            mp_str(w, "bitrate");
            mp_uint(w, ifc.bitrate());
            mp_str(w, "rxb");
            mp_uint(w, (uint32_t)ifc.traffic_counter_rxb());
            mp_str(w, "txb");
            mp_uint(w, (uint32_t)ifc.traffic_counter_txb());
        }

        /* These three are read from the top level of the map, not from an
         * interface entry: RNSTransport.render() takes d["path_table"],
         * d["link_count"] and d["interface_count"] directly. Their absence
         * raised KeyError as well, so one misplaced key and three missing
         * ones each cost the client the whole sensor.
         *
         * path_table stays empty and path_count is not reported: microReticulum
         * exposes no way to enumerate the transport's live paths, only
         * cumulative counters, and inventing a number from those would be
         * worse than showing none. */
        mp_str(w, "path_table");
        mp_arr(w, 0);
        mp_str(w, "link_count");
        mp_uint(w, 0);
        mp_str(w, "interface_count");
        mp_uint(w, (uint32_t)ifaces.size());
    }

    return w.ok ? w.len : 0;
}

/* Peer public keys, learned from announces we accepted.
 *
 * This exists because Identity::recall() does not work on this build: an
 * announce is validated and added to the path table (announce handler fires,
 * paths_added() climbs) but neither the known-destination store nor the
 * announce-packet fallback inside recall() ever returns the key again, so
 * every reply died on "identity for <peer> not known". Keeping the key that
 * the announce callback already handed us makes the node able to answer any
 * peer it has heard, which is all the node needs; the store is still worth
 * fixing, but that is a microReticulum problem, not a node one.
 *
 * Fixed size and no heap: this runs in the delivery path.
 */
#define MAX_PEERS 12

struct peer_entry
{
    RNS::Bytes hash;
    RNS::Bytes pubkey;
};

static peer_entry s_peers[MAX_PEERS];
static size_t s_peer_count;
static uint32_t s_announces_accepted;

static void remember_peer(const RNS::Bytes &hash, const RNS::Bytes &pubkey)
{
    if (hash.size() != 16 || pubkey.size() == 0)
        return;
    for (size_t i = 0; i < s_peer_count; ++i)
    {
        if (s_peers[i].hash == hash)
        {
            s_peers[i].pubkey = pubkey;
            return;
        }
    }
    /* Ring: a peer we have not heard from in a full rotation is dropped rather
     * than evicting on every announce, which would thrash in a busy mesh. */
    if (s_peer_count < MAX_PEERS)
    {
        s_peers[s_peer_count].hash = hash;
        s_peers[s_peer_count].pubkey = pubkey;
        ++s_peer_count;
    }
    else
    {
        static size_t next_slot;
        s_peers[next_slot].hash = hash;
        s_peers[next_slot].pubkey = pubkey;
        next_slot = (next_slot + 1) % MAX_PEERS;
    }
}

/* An identity for a peer, from the library store if it works and from our own
 * table otherwise. Returns a falsy Identity when the peer is unknown. */
static RNS::Identity peer_identity(const RNS::Bytes &hash)
{
    RNS::Identity known = RNS::Identity::recall(hash);
    if (known)
        return known;

    for (size_t i = 0; i < s_peer_count; ++i)
    {
        if (s_peers[i].hash == hash && s_peers[i].pubkey.size() > 0)
        {
            RNS::Identity peer(false);
            peer.load_public_key(s_peers[i].pubkey);
            if (peer)
                return peer;
        }
    }
    return RNS::Identity(false);
}

/* Icon and colours the client shows for this node, in its conversation list
 * and on the map.
 *
 * The field is [icon name, fg, bg] where the two colours are three raw bytes
 * each, not hex strings -- the client sends them to itself as
 * struct.pack("!BBB") and decodes them with the same three-byte read. An icon
 * name the font lacks renders blank rather than raising, so the name used here
 * was checked against the MaterialDesignIcons glyph table shipped inside
 * Sideband 2.2.0 rather than assumed. */
static void lxmf_apply_appearance(LXMF::LXMessage &msg)
{
    static const char *ICON = "radio-tower";
    static const uint8_t FG[3] = {0xF5, 0xF5, 0xF5};   /* near-white glyph */
    static const uint8_t BG[3] = {0x00, 0x69, 0x5C};   /* teal */

    uint8_t buf[48];
    Mpw w = {buf, sizeof(buf), 0, true};
    mp_arr(w, 3);
    mp_str(w, ICON);
    mp_bin(w, FG, sizeof FG);
    mp_bin(w, BG, sizeof BG);
    if (!w.ok)
        return;

    static const uint8_t key[1] = {LXMF_FIELD_ICON_APPEARANCE};
    msg.fields_set(RNS::Bytes(key, sizeof key), RNS::Bytes(buf, w.len));
}

static void lxmf_send_text(const RNS::Bytes &peer_hash, const std::string &text)
{
    RNS::Identity peer = peer_identity(peer_hash);
    if (!peer)
    {
        node_log("reply: identity for %s not known", peer_hash.toHex().c_str());
        return;
    }
    RNS::Destination dest(peer, RNS::Type::Destination::OUT,
                          RNS::Type::Destination::SINGLE, "lxmf", "delivery");
    LXMF::LXMessage reply(dest, s_lxmf->delivery_destination(),
                          RNS::Bytes((const uint8_t *)text.data(), text.size()),
                          RNS::bytesFromString("re"));
    lxmf_apply_appearance(reply);
    s_lxmf->handle_outbound(reply);
}

static void lxmf_send_telemetry(const RNS::Bytes &peer_hash)
{
    /* Which map goes back is a runtime choice, because the two differ in how
     * they travel and that is exactly what has to be measured on the bench.
     *
     * compact: TIME plus temperature, humidity and pressure, 151 packed bytes,
     *   which fits one encrypted packet and therefore needs no link. This is
     *   the answer that works today.
     *
     * full: adds LOCATION and MAGNETIC_FIELD, which puts the reply past the
     *   159-byte single-packet threshold, so the router sends it as a DIRECT
     *   link with a resource transfer. That path is what LXMF-012 is still
     *   open on: this node has carried a 614-byte reply that way before, while
     *   another bench could not assemble one. Whether a link resource actually
     *   lands here is settled by asking for telemetry in `full` mode.
     *
     * `node telemetry full` and `node telemetry compact` switch it, so the
     * bench can A/B the two without a reflash. Default is full, because
     * position and heading are the readings a remote operator cannot otherwise
     * see, and a reply that silently omits them is the bigger surprise. */
    const bool full = s_telemetry_full;

    static uint8_t tel[768];
    size_t n = telemetry_pack(tel, sizeof(tel), !full);
    if (n == 0)
    {
        node_log("telemetry: pack overflow");
        return;
    }

    RNS::Identity peer = peer_identity(peer_hash);
    if (!peer)
    {
        node_log("telemetry: identity for %s not known", peer_hash.toHex().c_str());
        return;
    }
    RNS::Destination dest(peer, RNS::Type::Destination::OUT,
                          RNS::Type::Destination::SINGLE, "lxmf", "delivery");

    /* Content and title stay empty: a message that carries only telemetry is
     * not shown as a chat message by the client. */
    LXMF::LXMessage reply(dest, s_lxmf->delivery_destination(), RNS::Bytes(), RNS::Bytes(),
                          full ? LXMF::Type::Message::DIRECT
                               : LXMF::Type::Message::OPPORTUNISTIC);

    /* The field value must itself be msgpack-encoded *bytes*, not the map.
     * microLXMF writes fields with packRawBytes(), so whatever is stored here
     * lands in the payload verbatim: handing it the bare map makes the outer
     * msgpack parse it into a dictionary on the far end, and the client's
     * Telemeter.from_packed() rejects it with "packed data must be type
     * 'bytes'". Wrapping the map in a bin header is exactly what the Python
     * reference produces, since it packs the field value with umsgpack.packb
     * and the outer packer then emits it as a bin. */
    static uint8_t field_value[800];
    Mpw fv = {field_value, sizeof(field_value), 0, true};
    mp_bin(fv, tel, n);
    if (!fv.ok)
    {
        node_log("telemetry: field buffer overflow");
        return;
    }

    static const uint8_t field_key[1] = {LXMF_FIELD_TELEMETRY};
    reply.fields_set(RNS::Bytes(field_key, sizeof(field_key)),
                     RNS::Bytes(field_value, fv.len));
    /* Not on the compact reply. The appearance field costs about 26 bytes, and
     * compact exists to fit one encrypted packet so no link and no known path
     * are needed; measured on this board it took the reply from one packet to a
     * 175-byte link transfer. The icon can wait for a full request. */
    if (full)
        lxmf_apply_appearance(reply);
    if (full)
        RNS::Transport::request_path(peer_hash);
    s_lxmf->handle_outbound(reply);
    node_log("telemetry: queued %u bytes (%s) for %s", (unsigned)n,
             full ? "full/link" : "compact/packet", peer_hash.toHex().c_str());
}

/* Counts announces that survived validation and were added to the path
 * table. Unlike Transport::announces_received() -- which counts packets that
 * arrived -- this only moves when an announce was accepted, so the two
 * together separate "the network is silent" from "announces arrive and are
 * rejected", which look identical from the outside. */

/* Announce handlers are called from inside the path-table insert, before the
 * filesystem has flushed the new record. Anything here must therefore be
 * cheap and must not read the stores back: a get() at this point sees a
 * half-written record, and doing it re-entrantly from the insert callback
 * takes the Reticulum task down with it. Caching the key is safe -- it comes
 * from the callback itself, not from storage. */
class NodeAnnounceHandler : public RNS::AnnounceHandler
{
public:
    NodeAnnounceHandler() : RNS::AnnounceHandler() {}

    void received_announce(const RNS::Bytes &destination_hash,
                           const RNS::Identity &announced_identity,
                           const RNS::Bytes &app_data) override
    {
        ++s_announces_accepted;
        remember_peer(destination_hash, announced_identity.get_public_key());
        node_log("announce: %s accepted peers=%u app=%s", destination_hash.toHex().c_str(),
                 (unsigned)s_peer_count, app_data.toString().c_str());
    }
};

static RNS::HAnnounceHandler s_announce_handler;

static void node_task(void *arg)
{
    /* Reset reason first: it separates a panic in the last boot from a
     * genuine hang, which otherwise look identical in an append-only log. */
    node_log("== boot, reset_reason=%d ==", (int)esp_reset_reason());

    /* Let the desktop paint its first frame before the radio starts. */
    vTaskDelay(pdMS_TO_TICKS(3000));

    char ssid[64], pass[96];
    if (!read_wifi_conf(ssid, sizeof(ssid), pass, sizeof(pass)))
    {
        s_state = NODE_NO_CONFIG;
        node_log("state=no_config (%s missing)", WIFI_CONF);
        ESP_LOGW(TAG, "no credentials in %s", WIFI_CONF);
        vTaskDelete(NULL);
        return;
    }

    s_state = NODE_WIFI;
    if (!wifi_start(ssid, pass))
    {
        s_state = NODE_WIFI_FAILED;
        node_log("state=wifi_failed ssid=%s", ssid);
        vTaskDelete(NULL);
        return;
    }

    s_state = NODE_RETICULUM;

    /* Reticulum's stores are rooted at the instance's storage path and are
     * opened by start(), so the directory has to exist before it runs -- not
     * only when the identity is first written. */
    /* Reticulum's stores are opened by start(), so the adapter and the
     * directories they use have to exist first. */
    mkdir("/fs/storage", 0755);
    mkdir("/fs/path_store", 0755);
    mkdir("/fs/cache", 0755);
    mkdir("/fs/reticulum", 0755);
    s_filesystem = new MountedFileSystem("/fs");
    bool fs_ok = s_filesystem.init();
    RNS::Utilities::OS::register_filesystem(s_filesystem);
    node_log("rns: storage /fs init=%d", (int)fs_ok);

    /* Probe the adapter directly, so a write failure is attributed to the
     * adapter rather than to whatever Reticulum happened to be doing. */
    {
        microStore::File probe = s_filesystem.open("/probe.txt",
                                                   microStore::File::ModeWrite, true);
        bool ok_open = probe.isValid();
        size_t wrote = ok_open ? probe.write((const uint8_t *)"probe", 5) : 0;
        probe.close();
        struct stat pst;
        bool exists_abs = stat("/fs/probe.txt", &pst) == 0;
        node_log("probe: open=%d write=%u file=%d size=%ld", (int)ok_open,
                 (unsigned)wrote, (int)exists_abs,
                 exists_abs ? (long)pst.st_size : -1L);
    }

    /* Inventory the sensor bus before anything else claims it, so a missing
     * device is visible in the boot record. */
    /* Inventory with the plain probe scan -- this is the form that reported
     * the bus correctly. Register reads follow for the devices we know how to
     * identify, since a real register value is stronger evidence than an ACK. */
    sensors_i2c_scan();
    sensors_identify();

    node_log("rns: constructing");
    s_reticulum = RNS::Reticulum();
    node_log("rns: constructed");
    s_reticulum.transport_enabled(true);
    s_reticulum.start();
    node_log("rns: started");

    s_announce_handler = std::make_shared<NodeAnnounceHandler>();
    RNS::Transport::register_announce_handler(s_announce_handler);
    node_log("rns: announce handler registered");

    /* Identity: recalled from flash so the destination hash is stable across
     * reboots, which is the whole point of a node having an address. */
    /* Identity::from_file() on a MISSING file returns an object that is
     * truthy but uninitialised, and hashing it faults -- so the file's
     * existence is checked first rather than trusting the result. */
    node_log("rns: loading identity");
    /* Existence is checked through the mount, matching how it is written. */
    char id_abs[96];
    snprintf(id_abs, sizeof(id_abs), "/fs%s", IDENTITY_FILE);
    struct stat id_st;
    if (stat(id_abs, &id_st) == 0)
    {
        s_identity = RNS::Identity::from_file(IDENTITY_FILE);
        node_log("rns: identity from file (%s)", s_identity ? "ok" : "unreadable");
    }
    else
    {
        node_log("rns: no identity file, generating");
    }
    if (!s_identity)
    {
        s_identity = RNS::Identity();
        if (!s_identity.to_file(IDENTITY_FILE))
            node_log("rns: IDENTITY NOT PERSISTED (%s)", IDENTITY_FILE);
        else
            node_log("rns: identity written to %s", IDENTITY_FILE);
    }
    node_log("rns: hashing identity");
    {
        /* Keep the hex string alive while it is copied: toHex() returns a
         * temporary, and both the Arduino String and std::string variants
         * hand back a buffer that dies with it. */
        auto h = s_identity.hash().toHex();
        node_log("rns: hex len=%u", (unsigned)h.length());
        snprintf(s_identity_hex, sizeof(s_identity_hex), "%s", h.c_str());
    }
    node_log("identity=%s", s_identity_hex);

    /* A crypto health line. Signature verification sits on the path of every
     * announce and every inbound message, and a broken verifier does not
     * report an error -- it just makes the network look empty, with zero
     * paths and no known peers forever. A sign/verify round trip here makes
     * that one kind of failure visible in a single line. */
    {
        static const uint8_t probe_data[] = "sign-verify self test";
        RNS::Bytes probe(probe_data, sizeof(probe_data) - 1);
        RNS::Bytes other((const uint8_t *)"sign-verify self tesT", sizeof(probe_data) - 1);
        RNS::Bytes sig = s_identity.sign(probe);
        bool ok = s_identity.validate(sig, probe);
        bool rejects = !s_identity.validate(sig, other);
        node_log("crypto: ed25519 sig=%u verify=%d tampered_rejected=%d",
                 (unsigned)sig.size(), (int)ok, (int)rejects);
    }

    node_log("rns: free heap=%u", (unsigned)esp_get_free_heap_size());
    node_log("rns: creating interface");
    try
    {
        s_udp = new UDPInterface();
        node_log("rns: interface allocated");
        s_udp.mode(RNS::Type::Interface::MODE_GATEWAY);
        RNS::Transport::register_interface(s_udp);
        node_log("rns: interface registered");
        bool up = s_udp.start();
        s_udp_ready = up;
        node_log("rns: interface start=%d", (int)up);
    }
    catch (const std::exception &e)
    {
        node_log("rns: EXCEPTION %s", e.what());
        s_state = NODE_FAILED;
        vTaskDelete(NULL);
        return;
    }
    catch (...)
    {
        node_log("rns: EXCEPTION (unknown)");
        s_state = NODE_FAILED;
        vTaskDelete(NULL);
        return;
    }

    /* LoRa, when the module is fitted.
     *
     * Registered after UDP so a single announce reaches both media, and before
     * the LXMF router so the delivery destination exists by the time the radio
     * is up. A missing module is not fatal: the node keeps running on WiFi
     * alone, which is what a board without the harness should do. */
    lora::set_logger([](const char *line) { node_log("lora: %s", line); });
    node_log("lora: starting");
    if (lora::begin())
        node_log("lora: interface registered");
    else
        node_log("lora: not available; continuing on WiFi only");

    /* LXMF on top of the Reticulum instance. The router needs the instance
     * and the interface up first: it creates a destination on the delivery
     * aspect and announces it, so a router started earlier would announce
     * into nothing. */
    node_log("lxmf: starting router");
    s_lxmf = new LXMF::LXMRouter(s_identity, "/lxmf", false);
    s_lxmf->set_display_name("TinyNode Nano");
    s_lxmf->register_delivery_callback([](LXMF::LXMessage &msg) {
        s_messages_rx++;
        node_log("lxmf: message from %s, %u bytes", msg.source_hash().toHex().c_str(),
                 (unsigned)msg.content().size());

        /* Commands arrive in a field, not the content: a telemetry request
         * from the client carries an empty body, which is also why clients
         * do not show it as a chat message. */
        static const uint8_t cmd_key[1] = {LXMF_FIELD_COMMANDS};
        const RNS::Bytes *cmds = msg.fields_get(RNS::Bytes(cmd_key, sizeof(cmd_key)));
        if (!cmds)
            return;

        struct node_commands c;
        parse_commands(*cmds, &c);
        node_log("lxmf: commands tel=%d ping=%d echo=%d fields=%u",
                 (int)c.telemetry, (int)c.ping, (int)c.echo,
                 (unsigned)msg.fields_count());

        if (c.telemetry)
            lxmf_send_telemetry(msg.source_hash());
        if (c.ping)
            lxmf_send_text(msg.source_hash(), "Ping reply");
        if (c.echo)
            lxmf_send_text(msg.source_hash(), "Echo reply: " + c.echo_text.toString());
    });
    s_lxmf->register_sent_callback([](LXMF::LXMessage &m) {
        node_log("lxmf: sent %s (%u bytes, method %u)", m.hash().toHex().c_str(),
                 (unsigned)m.packed_size(), (unsigned)m.method());
    });
    s_lxmf->register_delivered_callback([](LXMF::LXMessage &m) {
        node_log("lxmf: delivered %s", m.hash().toHex().c_str());
    });
    s_lxmf->register_failed_callback([](LXMF::LXMessage &m) {
        node_log("lxmf: FAILED %s after %u attempts", m.hash().toHex().c_str(),
                 (unsigned)m.delivery_attempts());
    });
    s_lxmf->announce();
    s_lxmf_up = true;
    node_log("lxmf: announced, delivery address %s",
             s_lxmf->delivery_destination().hash().toHex().c_str());

    s_state = NODE_READY;
    node_log("state=ready ip=%s free=%u", s_ip, (unsigned)esp_get_free_heap_size());

    /* Watched after the node is up, so the probes cannot delay Reticulum. The
     * GPS is the only device that may still be booting during the scan. */
    gps_watch(0x10, 10, 3000);
    int nsensors = sensors_begin();
    node_log("sensors: %d of 4 configured", nsensors);

    /* Hard-iron offsets from the last `magcal` run. Absent or unreadable
     * leaves the field uncorrected, which the command reports as such rather
     * than pretending the reading is calibrated. */
    {
        struct mag_offsets o = mag_cal_offsets();
        if (mag_cal_load())
        {
            struct mag_offsets n = mag_cal_offsets();
            node_log("mag: offsets loaded X %d Y %d Z %d", n.x, n.y, n.z);
        }
        else
        {
            node_log("mag: no calibration (X %d Y %d Z %d); run `magcal`",
                     o.x, o.y, o.z);
        }
    }

    uint32_t elapsed = 0;
    uint32_t next_announce = (ANNOUNCE_MIN_S +
                              (esp_random() % (ANNOUNCE_MAX_S - ANNOUNCE_MIN_S)));
    s_announced_once = false;

    for (;;)
    {
        s_reticulum.loop();
        /* The GPS is drained every tick: its NMEA stream runs at about
         * 480 B/s and a 32-byte read every 2 s would leave the buffer
         * overrunning and the clock tens of seconds stale. */
        sensors_poll_gps();
        /* Calibration samples only while a run is active; the command that
         * starts one blocks in its own task and reads progress from here. */
        mag_cal_poll();
        if ((elapsed % 40) == 0)         /* every 2 s: the slow sensors */
            sensors_refresh();

        if (s_lxmf_up)
        {
            s_lxmf->process_inbound();
            s_lxmf->process_outbound();
        }

        /* elapsed counts 50 ms ticks, so seconds elapsed is elapsed / 20.
         * Comparing ticks against a value in seconds -- which an earlier
         * version did -- makes the interval twenty times too short, and that
         * version also reset the timer without ever announcing, so the node
         * announced once at boot and never again. Both are fixed here. */
        uint32_t seconds_up = elapsed / 20;
        if (!s_announced_once)
        {
            s_announced_once = true;
        }
        else if (seconds_up >= next_announce || s_announce_requested)
        {
            s_announce_requested = false;
            elapsed = 0;
            next_announce = (ANNOUNCE_MIN_S +
                             (esp_random() % (ANNOUNCE_MAX_S - ANNOUNCE_MIN_S)));
            if (s_lxmf_up && s_lxmf)
            {
                s_lxmf->announce();
                node_log("lxmf: re-announced (next in %us)", (unsigned)next_announce);
            }
        }

        if (s_persist_requested)
        {
            s_persist_requested = false;
            RNS::Transport::persist_data();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed += 1;
        if ((elapsed % 1200) == 0)          /* every 60 s */
        {
            /* Path count is the mesh-visibility signal: it goes up when an
             * announce from a peer arrives over the UDP interface, so a
             * non-zero value proves the radio path works both ways. */
            {
                const struct sensor_snapshot *sn = sensors_latest();
                /* Set the wall clock from GPS whenever it is meaningfully out.
                 * There is no RTC on this board, so without this the system
                 * clock sits at 1970 + uptime: every LXMF message carries that
                 * timestamp, and microStore's sweep drops records that look
                 * like the future (FLS-005). The reference policy is the same
                 * one the standalone node used: correct when more than 2 s
                 * out, so the clock is not being reset on every sample. */
                node_log("clock: system=%ld gps=%lu", (long)time(NULL),
                         (unsigned long)sn->gps_unix);
                if (sn->gps_unix > 1700000000)
                {
                    long sys_now = (long)time(NULL);
                    long drift = (long)sn->gps_unix - sys_now;
                    if (drift > 2 || drift < -2)
                    {
                        struct timeval tv = {
                            .tv_sec = (time_t)sn->gps_unix,
                            .tv_usec = 0,
                        };
                        settimeofday(&tv, NULL);
                        node_log("clock: set from GPS, was %ld drift %lds", sys_now, drift);
                    }
                }
                node_log("sensors: env=%d %.1fC/%.1f%% press=%d %.1fhPa mag=%d gps=%d fix=%d unix=%lu",
                         (int)sn->have_env, sn->temperature_c, sn->humidity_pct,
                         (int)sn->have_pressure, sn->pressure_hpa,
                         (int)sn->have_mag, (int)sn->have_gps, (int)sn->gps_fix,
                         (unsigned long)sn->gps_unix);
            }
            node_log("alive t=%lus free=%u paths=%u acc=%u ann=%u rx=%u pkts=%u/%u",
                     (unsigned long)(elapsed / 20),
                     (unsigned)esp_get_free_heap_size(),
                     /* paths_added() is maintained by Transport itself; an
                      * earlier version called size() on the path store and
                      * printed bytes of a path string instead. */
                     (unsigned)RNS::Transport::paths_added(),
                     (unsigned)s_announces_accepted,
                     (unsigned)RNS::Transport::announces_received(),
                     (unsigned)s_messages_rx,
                     (unsigned)RNS::Transport::packets_received(),
                     (unsigned)RNS::Transport::packets_sent());
            s_stats.announces_heard = RNS::Transport::announces_received();
            s_stats.packets_in = RNS::Transport::packets_received();
            s_stats.packets_out = RNS::Transport::packets_sent();
        }
    }
}

/* ---- public --------------------------------------------------------- */

/* `node` in the desktop's Terminal: the status surface. Prints what the
 * service knows, from the same state the log and the UI read. */
/* ---- magnetometer calibration command -------------------------------- */

/* The field this bench sits in, from IGRF-14 evaluated at 33.2939 N,
 * -111.8810 E, 377 m for 2026. These are the numbers a healthy calibration
 * should reproduce, and they replace the "about 48 uT" this command used to
 * quote, which was a guess and three percent high.
 *
 *   total   F = 46.6 uT      horizontal H = 23.8 uT
 *   vertical Z = 40.0 uT      dip = 59.3 deg, declination = 9.6 deg E
 *
 * A different site needs different numbers; the dip angle is the part that
 * transfers, since it depends on latitude rather than on the bench. */
#define BENCH_FIELD_UT 46.6
#define BENCH_HORIZ_UT 23.8
#define BENCH_DIP_DEG  59.3


/* Read one line from the shell's stdin, which blocks until the user types.
 * The Terminal sends CR for Enter, and the shell's own reader accepts CR or
 * LF, so both terminate here. Ctrl+C aborts rather than being buffered. */
static bool shell_read_line(const char *prompt, char *buf, size_t cap)
{
    if (prompt)
    {
        fputs(prompt, stdout);
        fflush(stdout);
    }
    size_t n = 0;
    for (;;)
    {
        int c = getchar();
        if (c == EOF)
            return false;
        if (c == 0x03)                  /* Ctrl+C */
        {
            fputc('\n', stdout);
            return false;
        }
        if (c == '\r' || c == '\n')
            break;
        if (cap > 0 && n + 1 < cap)
            buf[n++] = (char)c;
    }
    if (cap > 0)
        buf[n] = '\0';
    return true;
}

/* Current field as one line, so the before and after readings cannot drift
 * apart in wording.
 *
 * Read from the shared snapshot, never by touching the bus: this runs in the
 * shell task, and the node task owns the I2C bus. A read here would race the
 * background sensor refresh. The snapshot is refreshed every two seconds,
 * which is current enough to judge a calibration. */
static void magcal_readings(char *out, size_t n)
{
    const struct sensor_snapshot *sp = sensors_latest();
    const double x = sp->mag_x_ut;
    const double y = sp->mag_y_ut;
    const double z = sp->mag_z_ut;
    const double b = sqrt(x * x + y * y + z * z);
    /* Taken from the snapshot rather than computed here, so this line and the
     * heading sent as telemetry can never disagree about the same field. */
    char h[24];
    if (sp->have_heading)
        snprintf(h, sizeof h, "%5.1f deg", sp->mag_heading_deg);
    else
        snprintf(h, sizeof h, "   --   ");
    snprintf(out, n,
             "X %+6.1f  Y %+6.1f  Z %+6.1f uT   |B| %5.1f uT   magnetic %s",
             x, y, z, b, h);
}

/* Verify a calibration from the board's own sampling.
 *
 * The host-side test is too slow to judge this: polling telemetry over LXMF
 * takes about three seconds per reading, so a minute of turning yields twenty
 * clustered samples and no useful fit. The board polls its magnetometer at
 * 20 Hz, so the same minute gives over a thousand points.
 *
 * The run reports |B| spread -- the direct test of whether the offsets work --
 * and the centre of the sphere the samples lie on, which is the bias still
 * left. That centre is offered as a correction, so a calibration that came out
 * imperfect can be finished without redoing it. */
static int mag_check(void)
{
    printf("\n  TURN THE BOARD SLOWLY THROUGH EVERY ORIENTATION.\n");
    printf("    Flat, on both long edges, face down, and tip it through\n");
    printf("    vertical. Keep it moving -- a still board tests nothing.\n");
    printf("    Press Enter when you have been round a few times.\n\n");
    fflush(stdout);

    if (!mag_cal_begin())
    {
        printf("  cannot start: the magnetometer is not answering\n");
        return 1;
    }

    double lo = 1e9, hi = -1e9;
    uint32_t elapsed = 0, next_report = 0;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        elapsed += 250;

        const int k = link_any_key();
        if (k == 1)
        {
            mag_cal_abort();
            printf("\ncancelled\n");
            return 1;
        }
        if (k != 0)
        {
            printf("\n");
            break;
        }

        if (elapsed >= next_report && (elapsed % 1000) == 0)
        {
            /* Live |B| from the newest sample, so the operator sees it move. */
            const struct sensor_snapshot *sn = sensors_latest();
            const double b = sqrt(sn->mag_x_ut * sn->mag_x_ut +
                                  sn->mag_y_ut * sn->mag_y_ut +
                                  sn->mag_z_ut * sn->mag_z_ut);
            if (b > 0.1)
            {
                if (b < lo) lo = b;
                if (b > hi) hi = b;
                printf("    %4lus  %5lu samples  |B| %5.2f   range %5.2f..%5.2f  spread %5.2f\n",
                       (unsigned long)(elapsed / 1000),
                       (unsigned long)mag_cal_samples(), b, lo, hi, hi - lo);
                fflush(stdout);
            }
            next_report = elapsed;
        }
    }

    /* Stop sampling; the point cloud stays for the analysis below. */
    mag_cal_abort();

    const double rot = mag_cal_rotation_ut();
    printf("\n  samples %lu, largest axis spread %.1f uT\n",
           (unsigned long)mag_cal_samples(), rot);
    if (rot < 20.0)
    {
        printf("\n  The board barely moved, so this tests nothing.\n");
        printf("  Run `magcal check` again and turn it right through.\n");
        return 1;
    }

    int32_t res[3] = {0, 0, 0};
    double radius = 0, scatter = 0;
    if (!mag_cal_residual(res, &radius, &scatter))
    {
        printf("\n  Not enough spread of samples to fix a sphere.\n");
        printf("  Turn it through more orientations and try again.\n");
        return 1;
    }

    const double rmag = sqrt((double)res[0] * res[0] + (double)res[1] * res[1] +
                             (double)res[2] * res[2]) * (100.0 / 6842.0);
    printf("  |B| range %.2f .. %.2f uT, spread %.2f uT (%.1f%%)\n",
           lo, hi, hi - lo, (hi - lo) / ((hi + lo) / 2) * 100.0);
    printf("  sphere radius %.2f uT, scatter %.2f uT\n", radius, scatter);
    printf("  residual bias %.2f uT\n", rmag);
    printf("\n");

    if (rmag < 0.7 && (hi - lo) / ((hi + lo) / 2) < 0.03)
    {
        printf("  GOOD: |B| is steady in every orientation. Calibration done.\n");
        return 0;
    }

    printf("  NOT yet: |B| still varies by %.1f%%, so %.2f uT of bias is left.\n",
           (hi - lo) / ((hi + lo) / 2) * 100.0, rmag);
    if (scatter > 1.5)
    {
        printf("  Scatter is %.2f uT, above sensor noise, so there is some\n", scatter);
        printf("  soft-iron distortion as well; a correction will help but\n");
        printf("  not make it perfect.\n");
    }
    /* res is the centre of the sphere the raw samples lie on, which IS the
     * full hard-iron offset, not an increment on top of the stored one. So it
     * replaces them; adding would double-count. */
    printf("\n  Replace the stored offsets with %.0f %.0f %.0f counts? [Y/n]: ",
           (double)res[0], (double)res[1], (double)res[2]);
    fflush(stdout);

    char line[64];
    if (!shell_read_line("", line, sizeof line))
        return 0;
    if (line[0] == 'n' || line[0] == 'N')
    {
        printf("  not applied\n");
        return 0;
    }

    struct mag_offsets o;
    o.x = (int16_t)res[0];
    o.y = (int16_t)res[1];
    o.z = (int16_t)res[2];
    mag_cal_set_offsets(&o);
    if (mag_cal_save())
        printf("  offsets now X %d Y %d Z %d, saved\n", o.x, o.y, o.z);
    else
        printf("  offsets now X %d Y %d Z %d (could not save)\n", o.x, o.y, o.z);
    printf("  Run `magcal check` again to confirm.\n");
    return 0;
}

static int cmd_magcal(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    char line[64];

    if (argc > 1 && strcmp(argv[1], "show") == 0)
    {
        char rd[128];
        magcal_readings(rd, sizeof rd);
        double M[3][3], c[3];
        if (mag_cal_matrix(M, c))
        {
            printf("matrix (soft + hard iron):\n");
            for (int i = 0; i < 3; ++i)
                printf("  %.9g %.9g %.9g\n", M[i][0], M[i][1], M[i][2]);
            printf("centre     : %.3f %.3f %.3f counts\n", c[0], c[1], c[2]);
        }
        else
        {
            struct mag_offsets o = mag_cal_offsets();
            printf("offsets: X %d  Y %d  Z %d\n", o.x, o.y, o.z);
        }
        printf("reading: %s\n", rd);
        return 0;
    }

    /* Set the correction's gain against a known field.
     *
     * The fit fixes the shape of the correction; its size depends on how much
     * of the ellipsoid was swept, and a partial tumble under-sizes it so every
     * reading comes out high. Point the board at a field of known strength --
     * hold it clear of the desk, or pass the local magnitude -- and this scales
     * the matrix to match, leaving bearing and dip alone. */
    if (argc > 1 && strcmp(argv[1], "gain") == 0)
    {
        if (!mag_cal_using_matrix())
        {
            printf("no matrix stored; run `magcal` first\n");
            return 1;
        }

        double target = BENCH_FIELD_UT;
        if (argc > 2)
            target = atof(argv[2]);
        if (target < 10.0 || target > 100.0)
        {
            printf("target must be 10..100 uT (here it is %.1f)\n",
                   (double)BENCH_FIELD_UT);
            return 1;
        }

        /* Average several seconds of the snapshot. The field is steady and
         * the sensor is not, so one sample is worth about a percent. */
        double sum = 0.0;
        int n = 0;
        for (int i = 0; i < 8; ++i)
        {
            const struct sensor_snapshot *sp = sensors_latest();
            const double b = sqrt(sp->mag_x_ut * sp->mag_x_ut +
                                  sp->mag_y_ut * sp->mag_y_ut +
                                  sp->mag_z_ut * sp->mag_z_ut);
            if (b > 1.0)
            {
                sum += b;
                ++n;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (n < 4 || sum / n < 5.0)
        {
            printf("field too weak to measure; hold the board still\n");
            return 1;
        }

        const double measured = sum / n;
        char before[128];
        magcal_readings(before, sizeof before);

        if (!mag_cal_rescale(measured, target))
        {
            printf("no matrix stored\n");
            return 1;
        }
        if (!mag_cal_save())
        {
            printf("scaled, but saving failed -- the change is live only\n");
            return 1;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));    /* let a fresh snapshot land */
        char after[128];
        magcal_readings(after, sizeof after);

        printf("\n  measured : %.1f uT\n", measured);
        printf("  target   : %.1f uT\n", target);
        printf("  scale    : %.5f\n", target / measured);
        printf("  before   : %s\n", before);
        printf("  after    : %s\n", after);
        if (fabs(measured - target) / target > 0.20)
            printf("\n  That is %.0f%% out. Check the board is clear of metal\n"
                   "  before trusting the gain set here.\n",
                   100.0 * fabs(measured - target) / target);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "check") == 0)
        return mag_check();

    if (argc > 1 && strcmp(argv[1], "clear") == 0)
    {
        struct mag_offsets zero = {0, 0, 0};
        mag_cal_set_offsets(&zero);
        remove("/fs/mag_cal");
        printf("offsets cleared; the reading is now uncorrected\n");
        return 0;
    }

    if (argc > 1)
    {
        printf("usage: magcal [show|gain|check|clear]\n");
        return 1;
    }

    /* ---- guided run ---- */
    struct mag_offsets before = mag_cal_offsets();
    char rd_before[128];
    magcal_readings(rd_before, sizeof rd_before);

    printf("\n");
    printf("Magnetometer calibration\n");
    printf("=========================================================\n");
    printf("Hard-iron offsets correct for the metal around the sensor:\n");
    printf("the field sweeps a sphere as you turn the board, and the\n");
    printf("midpoint of each axis is the offset the metal adds.\n");
    printf("\n");
    printf("offsets now  : X %d  Y %d  Z %d%s\n", before.x, before.y, before.z,
           (before.x == 0 && before.y == 0 && before.z == 0) ? "  (none saved)" : "");
    printf("reading now  : %s\n", rd_before);
    printf("\nexpected |B| : %.1f uT here (IGRF-14, 33.29N -111.88E, 2026),\n",
           (double)BENCH_FIELD_UT);
    printf("               of which %.1f uT is horizontal and %.1f uT vertical\n",
           (double)BENCH_HORIZ_UT,
           sqrt((double)BENCH_FIELD_UT * BENCH_FIELD_UT -
                (double)BENCH_HORIZ_UT * BENCH_HORIZ_UT));
    printf("               at a dip of %.1f deg. Readings well under that mean\n",
           (double)BENCH_DIP_DEG);
    printf("               the offsets are missing, or there is a magnet or\n");
    printf("               steel within about 20 cm.\n");
    printf("\n");
    printf("Before you start:\n");
    printf("  * clear 20 cm around the board: no steel, magnets, speakers,\n");
    printf("    power supply or the monitor\n");
    printf("  * TUMBLE it through every orientation, not just a flat spin:\n");
    printf("    flat, both long edges, face down, back up\n");
    printf("  * the percentages show how far each axis has swept. 60%% on all\n");
    printf("    three is enough -- the fit uses every sample, not the extremes\n");
    printf("\n");

    if (!shell_read_line("Press Enter to start sampling, Ctrl+C to cancel: ",
                         line, sizeof line))
    {
        printf("cancelled; offsets unchanged\n");
        return 1;
    }

    if (!mag_cal_begin())
    {
        printf("cannot start: the magnetometer is not answering\n");
        return 1;
    }

    /* The operator decides when to stop: "press Enter when the numbers
     * stabilise". Nothing here ends the run on its own.
     *
     * Three earlier versions tried to detect completion automatically --
     * per-axis span, then a nominal radius target, then convergence plus
     * octant coverage -- and each managed to either stop early on bad data or
     * refuse to stop at all. The person turning the board can see whether the
     * radius has settled; the code cannot see the board. So the code reports
     * and the person decides, which is also the only version that cannot trap
     * someone in a loop whose exit condition it failed to reach. */
    printf("\n  Tumble the board slowly through every orientation:\n");
    printf("    flat, on edge, face down, back up. Not just a flat spin --\n");
    printf("    that traces a circle and cannot fix a sphere.\n\n");
    printf("  Press Enter when the span numbers stop growing. Ctrl+C cancels.\n\n");
    fflush(stdout);

    uint32_t elapsed = 0;
    uint32_t next_report = 0;
    /* spans replace the radius readout; last_r no longer needed */

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        elapsed += 250;

        const int k = link_any_key();
        if (k == 1)                       /* Ctrl+C: throw the samples away */
        {
            mag_cal_abort();
            printf("\ncancelled; offsets unchanged\n");
            return 1;
        }
        if (k != 0)                       /* Enter or any other key */
        {
            printf("\n");
            break;
        }

        if (elapsed >= next_report && (elapsed % 1000) == 0)
        {
            /* Spans are what the min/max estimator uses, so they are what
             * the operator needs to watch: each one stops growing when that
             * axis has been swept end to end. */
            /* Spans against what a full tumble would give, because "4732"
             * means nothing to the operator and "74%" does. A full rotation
             * sweeps the field diameter, 2 * 46.6 uT = 6377 counts, so each
             * axis needs to reach most of that or its midpoint is biased. */
            int32_t span[3];
            mag_cal_spans(span);
            const double full = 2.0 * BENCH_FIELD_UT * 6842.0 / 100.0;
            /* A sphere fit uses every sample, so it does not need the
             * extremes the way a min/max midpoint does. 60 percent on each
             * axis is ample; beyond that the extra samples hardly change the
             * fitted centre. */
            const int p0 = (int)(span[0] / full * 100.0);
            const int p1 = (int)(span[1] / full * 100.0);
            const int p2 = (int)(span[2] / full * 100.0);
            printf("    %4lus %5lu  X %3d%%  Y %3d%%  Z %3d%%   %s\n",
                   (unsigned long)(elapsed / 1000),
                   (unsigned long)mag_cal_samples(), p0, p1, p2,
                   (p0 >= 60 && p1 >= 60 && p2 >= 60) ? "enough - press Enter"
                                                     : "keep turning");
            fflush(stdout);
            next_report = elapsed;
        }
    }

    struct mag_offsets found;
    char report[192];
    const bool ok = mag_cal_finish(&found, report, sizeof report);

    printf("\nSampling stopped.\n");
    printf("  %s\n", report);

    {
        int32_t sp[3];
        mag_cal_spans(sp);
        const double full = 2.0 * BENCH_FIELD_UT * 6842.0 / 100.0;
        int lo = (int)(sp[0] / full * 100.0);
        if ((int)(sp[1] / full * 100.0) < lo) lo = (int)(sp[1] / full * 100.0);
        if ((int)(sp[2] / full * 100.0) < lo) lo = (int)(sp[2] / full * 100.0);
        if (lo < 60)
        {
            printf("\n  Coverage was only %d%% on the weakest axis. The fit\n", lo);
            printf("  needs the field seen from several directions, so tip the\n");
            printf("  board further over next time -- face down as well as up.\n");
        }
    }

    /* Diagnostics matter most when the fit failed, so they print either way. */
    mag_cal_diagnose();

    if (!ok)
    {
        printf("\n  That is not enough to fix an offset, so nothing was\n");
        printf("  changed. Run `magcal` again and tumble further -- and for\n");
        printf("  longer than a few seconds.\n");
        return 1;
    }

    /* The radius is what the fit measured. It is NOT compared against the
     * IGRF value for the bench's coordinates: an undisturbed model gives
     * 46.6 uT here, a first run measured 38.0, and two independent estimates
     * agreed -- the fit's radius and sd*sqrt(3) over the samples -- so the
     * field at this bench really is about 38 uT. Steel within a metre
     * screens several uT. Calling that a bad calibration was wrong.
     *
     * What matters for a hard-iron calibration is that the samples lie on a
     * sphere at all, and the acceptance test is that |B| stops varying with
     * orientation. So report the measured value, and say how to check it. */
    {
        const double r = mag_cal_fit_radius_ut();
        const double cov = mag_cal_coverage() * 100.0;
        printf("\n  Field radius measured here: %.1f uT (coverage %.0f%%).\n", r, cov);
        printf("  The undisturbed model for this location is %.1f uT; a lower\n",
               (double)BENCH_FIELD_UT);
        printf("  reading means something within a metre screens the field,\n");
        printf("  which does not stop the offsets being right.\n");
        printf("\n  The test that matters: turn the board slowly. |B| should\n");
        printf("  hold steady at about %.0f uT in every orientation. If it\n", r);
        printf("  swings, the offsets are incomplete -- run magcal again and\n");
        printf("  tumble further, especially face down.\n");
    }

    /* Prefer the full model: three offsets cannot reshape an ellipsoid back
     * into a sphere, and soft iron is what the residuals here show. The
     * offsets from mag_cal_finish are kept as a fallback and still used when
     * the fit cannot be solved. */
    {
        char erep[128];
        if (mag_cal_fit_ellipsoid(BENCH_FIELD_UT, erep, sizeof erep))
        {
            printf("  %s\n", erep);
            printf("  (soft-iron matrix, not just offsets -- |B| should now be\n");
            printf("   steady in every orientation)\n");
        }
        else
        {
            printf("  %s; using offsets only\n", erep);
            mag_cal_set_offsets(&found);
        }
    }

    char rd_after[128];
    magcal_readings(rd_after, sizeof rd_after);
    if (mag_cal_using_matrix())
        printf("\n  correction  : soft-iron matrix applied\n");
    else
        printf("\n  new offsets : X %d  Y %d  Z %d\n", found.x, found.y, found.z);
    printf("  reading now : %s\n", rd_after);
    if (!shell_read_line("Save these offsets to flash? [Y/n]: ", line, sizeof line))
    {
        printf("not saved; they apply until the next reboot\n");
        return 0;
    }
    if (line[0] == 'n' || line[0] == 'N')
    {
        printf("not saved; they apply until the next reboot\n");
        return 0;
    }
    if (mag_cal_save())
        printf("saved to /fs/mag_cal\n");
    else
        printf("!! could not write /fs/mag_cal; they apply until reboot\n");
    return 0;
}

/* Console width, for the shell's line editor.
 *
 * The width is not measured: asking a terminal means reading the reply from
 * the same USB FIFO the shell reads input from, and two attempts at that ate
 * the operator's keystrokes. Setting it by hand costs one command and cannot
 * corrupt input. */
extern "C" void link_set_console_columns(int cols);
extern "C" int link_console_columns(void);

/* Write /fs/etc/wifi.conf.
 *
 * Credentials used to arrive only as -DTD_WIFI_SSID / -DTD_WIFI_PASSWORD on
 * the build command, which meant the network lived in CMake's cache: a plain
 * `idf.py build` silently produced a board with no network, and clearing the
 * cache once took the node off the air. The file is the durable form and this
 * command is how it gets written.
 *
 * The node task reads the file once at start and exits if it is absent, so a
 * change needs a restart. Saying so is better than appearing to apply it. */
static int cmd_wifi(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc > 1 && strcmp(argv[1], "clear") == 0)
    {
        if (remove(WIFI_CONF) == 0)
            printf("removed %s; restart to come up without a network\n", WIFI_CONF);
        else
            printf("no %s to remove\n", WIFI_CONF);
        return 0;
    }

    if (argc > 1)
    {
        if (argc < 3)
        {
            printf("usage: wifi <ssid> <password>\n"
                   "       wifi            show what is configured\n"
                   "       wifi clear      remove the configuration\n");
            return 1;
        }
        /* The file lives under /fs, which is the shell's own root, so make
         * sure /etc exists before opening it. */
        mkdir("/fs/etc", 0755);
        FILE *fp = fopen(WIFI_CONF, "w");
        if (!fp)
        {
            printf("!! cannot write %s\n", WIFI_CONF);
            return 1;
        }
        fprintf(fp, "ssid = %s\npassword = %s\n", argv[1], argv[2]);
        fclose(fp);
        printf("saved to %s\n", WIFI_CONF);
        printf("restart to apply: the node reads this once at start\n");
        return 0;
    }

    /* Status, without echoing the password back to the terminal. */
    char ssid[64], pass[96];
    if (read_wifi_conf(ssid, sizeof ssid, pass, sizeof pass))
        printf("ssid:     %s\npassword: %s\n", ssid,
               pass[0] ? "(set)" : "(empty)");
    else
        printf("no credentials in %s\n", WIFI_CONF);
    printf("state:    %s\n", node_state_text());
    printf("ip:       %s\n", s_ip[0] ? s_ip : "(none)");
    return 0;
}

static int cmd_cols(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    if (argc < 2)
    {
        printf("%d columns\n", link_console_columns());
        return 0;
    }
    const int n = atoi(argv[1]);
    if (n < 20 || n > 500)
    {
        printf("usage: cols [20..500]\n");
        return 1;
    }
    link_set_console_columns(n);
    printf("console width set to %d columns\n", n);
    return 0;
}

/* ---- one status view, three renderers ------------------------------------
 *
 * The node describes itself three times: the console's `node` command, the
 * LXMF telemetry map and the Tang's text. Each used to format its own facts,
 * and they drifted -- the console printed a GPS position to six decimal places
 * and the Tang to four, and the Tang's radio counters were a placeholder
 * printing zeros because the real string did not fit its frame budget.
 *
 * Everything lands here first, in display units and at one precision, so a
 * renderer only decides how to present a fact and never recomputes it. The
 * policy that matters is the precision: six places for a position is about
 * 11 cm, well past what this receiver resolves, and the Tang's frame budget
 * cannot afford six anyway.
 */
struct node_view
{
    const char *state;      /* node_state_text() */
    uint32_t uptime_s;
    const char *ip;
    int rssi_dbm;

    bool have_env;
    double temperature_c;
    double humidity_pct;
    bool have_pressure;
    double pressure_hpa;

    bool have_mag;
    double mag_ut;          /* magnitude, not the vector */
    bool have_heading;
    double heading_deg;

    bool gps_fix;
    double lat, lon, alt_m;
    unsigned sats;

    bool lora_up;
    uint32_t lora_tx, lora_rx;
    int lora_rssi, lora_snr;
    const char *lora_text;  /* filled by node_view_fill */
};

static char s_lora_text[96];

static void node_view_fill(struct node_view *v)
{
    const struct sensor_snapshot *sn = sensors_latest();
    memset(v, 0, sizeof *v);
    v->state    = node_state_text();
    v->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    v->ip       = s_ip[0] ? s_ip : "none";
    v->rssi_dbm = (int)s_stats.rssi_dbm;

    v->have_env        = sn->have_env;
    v->temperature_c   = sn->temperature_c;
    v->humidity_pct    = sn->humidity_pct;
    v->have_pressure   = sn->have_pressure;
    v->pressure_hpa    = sn->pressure_hpa;

    v->have_mag = sn->have_mag;
    if (sn->have_mag)
        v->mag_ut = sqrt(sn->mag_x_ut * sn->mag_x_ut +
                         sn->mag_y_ut * sn->mag_y_ut +
                         sn->mag_z_ut * sn->mag_z_ut);
    v->have_heading  = sn->have_heading;
    v->heading_deg   = sn->mag_heading_deg;

    v->gps_fix = sn->gps_fix;
    v->lat     = sn->gps_lat;
    v->lon     = sn->gps_lon;
    v->alt_m   = sn->gps_alt_m;
    v->sats    = sn->gps_sats;

    v->lora_up = lora::status_line(s_lora_text, sizeof s_lora_text);
    v->lora_text = s_lora_text;
    lora::counters(&v->lora_tx, &v->lora_rx, &v->lora_rssi, &v->lora_snr);
}

const char *trim_copy(const char *s, size_t n, char *out, size_t cap)
{
    while (n > 0 && (*s == ' ' || *s == '\t')) { ++s; --n; }
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r'))
        --n;
    const size_t k = (n < cap - 1) ? n : cap - 1;
    for (size_t i = 0; i < k; ++i)
        out[i] = (char)tolower((unsigned char)s[i]);
    out[k] = '\0';
    return out;
}

/* Append one line, dropping it whole rather than truncating: half a line on a
 * 320x240 panel reads as data when it is only the end of a buffer. */
void append_line(char *buf, size_t cap, size_t *used, const char *line)
{
    if (!line || !line[0])
        return;
    const size_t n = strlen(line);
    if (*used + n + 2 >= cap)
        return;
    memcpy(buf + *used, line, n);
    *used += n;
    buf[(*used)++] = '\n';
    buf[*used] = '\0';
}


static int cmd_node(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    if (argc > 1 && strcmp(argv[1], "announce") == 0)
    {
        /* Announces are the only way a peer learns this address, and the
         * scheduled ones are hours apart, so there has to be a way to ask. */
        node_announce_now();
        printf("announce requested\n");
        node_log("node: announce requested from shell");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "telemetry") == 0)
    {
        /* `node telemetry full` / `node telemetry compact` choose what a
         * reply carries for the rest of the session; a bare `node telemetry`
         * is a dry run that packs the current choice and reports its size
         * without sending. The size is what decides opportunistic versus link
         * delivery, and the pack is otherwise only exercised by a client. */
        if (argc > 2)
        {
            if (strcmp(argv[2], "full") == 0 || strcmp(argv[2], "compact") == 0)
            {
                s_telemetry_full = (strcmp(argv[2], "full") == 0);
                printf("telemetry replies: %s\n",
                       s_telemetry_full ? "full (LOCATION + MAGNETIC_FIELD, link)"
                                        : "compact (environment triad, one packet)");
                node_log("node: telemetry replies set to %s",
                         s_telemetry_full ? "full" : "compact");
                return 0;
            }
            printf("usage: node telemetry [full|compact]\n");
            return 1;
        }
        static uint8_t probe[768];
        size_t n = telemetry_pack(probe, sizeof(probe), !s_telemetry_full);
        if (n == 0)
        {
            printf("telemetry: pack failed\n");
            node_log("telemetry: pack failed (dry run)");
            return 1;
        }
        char buf[128];
        snprintf(buf, sizeof(buf), "telemetry: %u bytes, %s", (unsigned)n,
                 s_telemetry_full ? "full/link" : "compact/packet");
        printf("%s\n", buf);
        node_log("%s (dry run)", buf);
        return 0;
    }
    if (argc > 1)
    {
        printf("usage: node [announce|telemetry [full|compact]]\n");
        return 1;
    }
    /* Printed for the Terminal on the panel, and echoed to node_log so the
     * same lines reach USB -- the panel is 40x15 and reading it back for a
     * bench check is not practical. */
    struct node_view v;
    node_view_fill(&v);

    char buf[160];
    snprintf(buf, sizeof(buf), "node:    %s", v.state);
    printf("%s\n", buf);
    node_log("%s", buf);
    snprintf(buf, sizeof(buf), "ip:      %s", v.ip);
    printf("%s\n", buf);
    node_log("%s", buf);
    snprintf(buf, sizeof(buf), "identity %s", s_identity_hex[0] ? s_identity_hex : "(not yet)");
    printf("%s\n", buf);
    node_log("%s", buf);
    if (v.rssi_dbm)
    {
        snprintf(buf, sizeof(buf), "rssi:    %d dBm", v.rssi_dbm);
        printf("%s\n", buf);
        node_log("%s", buf);
    }

    if (v.have_env)
    {
        snprintf(buf, sizeof(buf), "env:     %.1f C, %.1f %%RH", v.temperature_c, v.humidity_pct);
        printf("%s\n", buf);
        node_log("%s", buf);
    }
    if (v.have_pressure)
    {
        snprintf(buf, sizeof(buf), "press:   %.1f hPa", v.pressure_hpa);
        printf("%s\n", buf);
        node_log("%s", buf);
    }
    if (v.have_mag)
    {
        snprintf(buf, sizeof(buf), "mag:     %.1f uT%s", v.mag_ut,
                 v.have_heading ? "" : "");
        printf("%s\n", buf);
        node_log("%s", buf);
        if (v.have_heading)
        {
            snprintf(buf, sizeof(buf), "heading: %.1f deg (magnetic)", v.heading_deg);
            printf("%s\n", buf);
            node_log("%s", buf);
        }
    }
    if (v.gps_fix)
    {
        snprintf(buf, sizeof(buf), "gps:     %.6f, %.6f, %.0f m, %u sat",
                 v.lat, v.lon, v.alt_m, v.sats);
        printf("%s\n", buf);
        node_log("%s", buf);
    }
    else
    {
        printf("gps:     no fix\n");
        node_log("gps:     no fix");
    }
    return 0;
}

static const tdsh_command_t s_node_cmd = {
    .name = "node",
    .usage = "node [announce|telemetry]",
    .help = "Show the node's state, announce it, or pack a telemetry report",
    .fn = cmd_node,
    .flags = 0,
};

static const tdsh_command_t s_wifi_cmd = {
    .name = "wifi",
    .usage = "wifi [<ssid> <password> | clear]",
    .help = "Show, set or clear the WiFi credentials written to " WIFI_CONF,
    .fn = cmd_wifi,
    .flags = 0,
};

static const tdsh_command_t s_cols_cmd = {
    .name = "cols",
    .usage = "cols [20..500]",
    .help = "Show or set the console width, so lines wrap where the terminal does",
    .fn = cmd_cols,
    .flags = 0,
};

static void shell_log(const char *line) { printf("%s\n", line); }

/* The driver logs through a hook so the same text can land in /node.log at boot
 * and on the console when asked for. This swaps the hook for the length of one
 * report rather than duplicating every format string. */
static int cmd_lora(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    if (argc > 1 && strcmp(argv[1], "status") == 0)
    {
        char line[160];
        lora::status_line(line, sizeof line);
        printf("%s\n", line);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "verbose") == 0)
    {
        const bool on = (argc > 2 && strcmp(argv[2], "on") == 0);
        lora::set_verbose(on);
        printf("lora verbose %s\n", on ? "on" : "off");
        return 0;
    }

    lora::set_logger(shell_log);
    lora::console_report();
    lora::set_logger([](const char *line) { node_log("lora: %s", line); });
    return 0;
}

static const tdsh_command_t s_lora_cmd = {
    .name = "lora",
    .usage = "lora [status|verbose on|verbose off]",
    .help = "Show the RYLR998 link: band, address, network, air parameters, "
            "counters and the last rssi/snr; 'status' prints one line, "
            "'verbose on' logs every message the radio reassembles",
    .fn = cmd_lora,
    .flags = 0,
};

static const tdsh_command_t s_magcal_cmd = {
    .name = "magcal",
    .usage = "magcal [show|gain|check|clear]",
    .help = "Calibrate the magnetometer: run with no argument to calibrate, "
            "'check' to verify and correct, 'gain' to set its scale against a "
            "known field, 'show' to read, 'clear' to drop",
    .fn = cmd_magcal,
    .flags = 0,
};

void node_start(void)
{
    if (s_state != NODE_OFF)
        return;
    /* Registered here rather than in the shared app so the command exists
     * exactly when the service does. */
    {
        int rc = tdsh_register_command(&s_node_cmd);
        /* Verify by looking it up again: rc=0 means accepted, the lookup
         * means it is in the table the Terminal reads. */
        const tdsh_command_t *found = tdsh_command_find("node");
        node_log("node: shell command registered rc=%d found=%d", rc, found != nullptr);
        int rc2 = tdsh_register_command(&s_magcal_cmd);
        const tdsh_command_t *found2 = tdsh_command_find("magcal");
        node_log("node: magcal registered rc=%d found=%d", rc2, found2 != nullptr);
        int rc3 = tdsh_register_command(&s_cols_cmd);
        const tdsh_command_t *found3 = tdsh_command_find("cols");
        node_log("node: cols registered rc=%d found=%d", rc3, found3 != nullptr);
        int rc4 = tdsh_register_command(&s_wifi_cmd);
        const tdsh_command_t *found4 = tdsh_command_find("wifi");
        node_log("node: wifi registered rc=%d found=%d", rc4, found4 != nullptr);
        int rc5 = tdsh_register_command(&s_lora_cmd);
        const tdsh_command_t *found5 = tdsh_command_find("lora");
        node_log("node: lora registered rc=%d found=%d", rc5, found5 != nullptr);
    }
    s_state = NODE_WIFI;
    xTaskCreate(node_task, "rns", 32768, NULL, 4, NULL);
}

node_state_t node_state(void)
{
    return s_state;
}

const char *node_state_text(void)
{
    switch (s_state)
    {
    case NODE_WIFI:
        return "wifi connecting";
    case NODE_NO_CONFIG:
        return "no wifi credentials";
    case NODE_WIFI_FAILED:
        return "wifi failed";
    case NODE_RETICULUM:
        return "reticulum starting";
    case NODE_READY:
        return "online";
    case NODE_FAILED:
        return "reticulum failed";
    default:
        return "off";
    }
}

const char *node_identity_hex(void)
{
    return s_identity_hex;
}
const char *node_ip(void)
{
    return s_ip;
}

void node_get_stats(struct node_stats *out)
{
    if (out)
        *out = s_stats;
}

void node_persist(void)
{
    s_persist_requested = true;
}
void node_announce_now(void)
{
    s_announce_requested = true;
}

/* ---------------------------------------------------------------------------
 * The Tang link's answers
 *
 * TinyTang is a fixed client that speaks tang_proto over the LoRa framing: it
 * sends one application message and reads one reply. Since it is not being
 * changed, this node answers in the form it expects, which is the same command
 * vocabulary the node's own console uses, so a new console command needs no
 * work here.
 *
 * The reply body is at most TANG_PROTO_MAX_BODY (492) bytes and travels as
 * three LoRa frames, so the status text is deliberately a fixed set of short
 * lines rather than everything the console can print.
 * ------------------------------------------------------------------------- */

namespace {

}  // namespace

bool tanglink::node_answer(const char *command, size_t length, char *out, size_t capacity)
{
    if (!out || capacity == 0)
        return false;

    char cmd[48];
    trim_copy(command, length, cmd, sizeof cmd);

    struct node_view v;
    node_view_fill(&v);
    char line[160];

    if (strcmp(cmd, "ping") == 0)
    {
        snprintf(out, capacity, "pong up %lus", (unsigned long)v.uptime_s);
        return true;
    }
    if (strcmp(cmd, "status") == 0)
    {
        size_t used = 0;
        out[0] = '\0';
        /* Kept inside one LoRa frame: the framing carries 179 bytes, of which
         * tang_proto takes 8, so the body has 171. A longer reply spans frames,
         * which costs about 0.6 s of airtime each and then competes with the
         * Tang's own 1.2 s retry -- measured as a reply that arrived in pieces
         * and was abandoned. Short lines, one frame, always. */
        snprintf(line, sizeof line, "TinyNode %s %luh%lum", v.state,
                 (unsigned long)(v.uptime_s / 3600),
                 (unsigned long)((v.uptime_s % 3600) / 60));
        append_line(out, capacity, &used, line);
        snprintf(line, sizeof line, "wifi %s %ddBm", v.ip, v.rssi_dbm);
        append_line(out, capacity, &used, line);
        if (v.have_env)
        {
            snprintf(line, sizeof line, "env %.1fC %.0f%% %.0fhPa",
                     v.temperature_c, v.humidity_pct,
                     v.have_pressure ? v.pressure_hpa : 0.0);
            append_line(out, capacity, &used, line);
        }
        if (v.have_mag)
        {
            if (v.have_heading)
                snprintf(line, sizeof line, "mag %.0fuT hdg %.0f", v.mag_ut,
                         v.heading_deg);
            else
                snprintf(line, sizeof line, "mag %.0fuT", v.mag_ut);
            append_line(out, capacity, &used, line);
        }
        if (v.gps_fix)
        {
            snprintf(line, sizeof line, "gps %.4f %.4f %.0fm %us",
                     v.lat, v.lon, v.alt_m, v.sats);
            append_line(out, capacity, &used, line);
        }
        else
            append_line(out, capacity, &used, "gps nofix");
        /* Counters only, not the full status line: that names the address,
         * network, band and air parameters the Tang has no use for and costs
         * the ~75 bytes that push this reply into a second frame. */
        if (v.lora_up)
            snprintf(line, sizeof line, "lora tx%lu rx%lu %ddBm %d",
                     (unsigned long)v.lora_tx, (unsigned long)v.lora_rx,
                     v.lora_rssi, v.lora_snr);
        else
            snprintf(line, sizeof line, "lora down");
        append_line(out, capacity, &used, line);
        if (used == 0)
            snprintf(out, capacity, "TinyNode %s", v.state);
        return true;
    }
    if (strcmp(cmd, "env") == 0)
    {
        if (!v.have_env)
        {
            snprintf(out, capacity, "env unavailable");
            return true;
        }
        snprintf(out, capacity, "env %.1f C %.1f %%RH %.1f hPa",
                 v.temperature_c, v.humidity_pct,
                 v.have_pressure ? v.pressure_hpa : 0.0);
        return true;
    }
    if (strcmp(cmd, "gps") == 0)
    {
        if (!v.gps_fix)
        {
            snprintf(out, capacity, "gps no fix");
            return true;
        }
        /* Six places here and not in `status`: this is the answer to a direct
         * question about position, and it may be the only place the operator
         * can read it. It still costs the same number of frames. */
        snprintf(out, capacity, "gps %.6f %.6f %.0f m %u sat",
                 v.lat, v.lon, v.alt_m, v.sats);
        return true;
    }
    if (strcmp(cmd, "mag") == 0)
    {
        if (!v.have_mag)
        {
            snprintf(out, capacity, "mag unavailable");
            return true;
        }
        if (v.have_heading)
            snprintf(out, capacity, "mag %.1f uT heading %.0f deg", v.mag_ut,
                     v.heading_deg);
        else
            snprintf(out, capacity, "mag %.1f uT", v.mag_ut);
        return true;
    }
    if (strcmp(cmd, "lora") == 0)
    {
        if (v.lora_up)
            snprintf(out, capacity, "%.96s", v.lora_text);
        else
            snprintf(out, capacity, "lora not started");
        return true;
    }
    if (strcmp(cmd, "ident") == 0 || strcmp(cmd, "id") == 0)
    {
        snprintf(out, capacity, "id %s",
                 s_identity_hex[0] ? s_identity_hex : "(not yet)");
        return true;
    }
    return false;
}
