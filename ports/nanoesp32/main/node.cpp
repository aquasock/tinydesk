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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <microReticulum.h>
#include <microReticulum/Transport.h>
#include <microReticulum/Utilities/OS.h>

/* microStore, for the prefixed filesystem adapter below. */
#include <microStore/FileSystem.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "UDPInterface.h"

#include "esp_random.h"

#include <sys/stat.h>

static const char *TAG = "node";

#define WIFI_CONF      "/fs/etc/wifi.conf"
#define IDENTITY_FILE  "/fs/reticulum/identity"
#define RETICULUM_DIR  "/fs/reticulum"

/* TinyNode announces on a random 90-300 minute schedule. Same policy here:
 * a fixed interval makes every node in earshot announce together. */
#define ANNOUNCE_MIN_S (90u * 60u)
#define ANNOUNCE_MAX_S (300u * 60u)

static volatile node_state_t s_state = NODE_OFF;
static char s_identity_hex[33];
static char s_ip[16];
static struct node_stats s_stats;
static volatile bool s_persist_requested;
static volatile bool s_announce_requested;

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
        while (*key == ' ' || *key == '\t') key++;
        char *ke = key + strlen(key);
        while (ke > key && (ke[-1] == ' ' || ke[-1] == '\t')) *--ke = '\0';
        while (*val == ' ' || *val == '\t') val++;
        char *ve = val + strlen(val);
        while (ve > val && (ve[-1] == '\n' || ve[-1] == '\r' || ve[-1] == ' ')) *--ve = '\0';

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
                ESP_LOGI(TAG, "wifi up: %s, %d dBm", s_ip, ap.rssi);
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
static bool s_announced_once;

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
    MountedFile(FILE *fp, const std::string &name) : _fp(fp), _name(name) {}
    ~MountedFile() override { close(); }

protected:
    const char *name() const override { return _name.c_str(); }
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
    int read() override { return _fp ? fgetc(_fp) : -1; }
    size_t write(uint8_t ch) override { return _fp && fputc(ch, _fp) != EOF ? 1 : 0; }
    size_t read(uint8_t *buf, size_t n) override { return _fp ? fread(buf, 1, n, _fp) : 0; }
    size_t write(const uint8_t *buf, size_t n) override { return _fp ? fwrite(buf, 1, n, _fp) : 0; }
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
    size_t tell() override { return _fp ? (size_t)ftell(_fp) : 0; }
    long seek(uint32_t pos, microStore::SeekMode mode) override
    {
        int whence = mode == microStore::SeekModeSet ? SEEK_SET
                   : mode == microStore::SeekModeCur ? SEEK_CUR : SEEK_END;
        return _fp && fseek(_fp, (long)pos, whence) == 0 ? (long)pos : -1;
    }
    void flush() override
    {
        if (_fp)
            fflush(_fp);
    }
    bool isValid() const override { return _fp != nullptr; }

private:
    FILE *_fp;
    std::string _name;
};

class MountedFileSystem : public microStore::FileSystemImpl
{
public:
    explicit MountedFileSystem(const char *mount) : _mount(mount) {}

    bool init(bool reformatOnFail = true) override
    {
        (void)reformatOnFail;
        return true;
    }

    microStore::File open(const char *path, microStore::File::Mode mode, const bool create = false) override
    {
        using M = microStore::File;
        const char *m = mode == M::ModeRead ? "rb"
                      : mode == M::ModeWrite ? (create ? "wb" : "r+b")
                      : mode == M::ModeAppend ? "ab"
                      : mode == M::ModeReadAppend ? "a+b"
                      : "r+b";
        FILE *fp = fopen(full(path).c_str(), m);
        if (!fp && create && mode != M::ModeRead)
            fp = fopen(full(path).c_str(), "w+b");
        return microStore::File(fp ? new MountedFile(fp, path) : nullptr);
    }

    bool exists(const char *path) override
    {
        struct stat st;
        return stat(full(path).c_str(), &st) == 0;
    }
    bool remove(const char *path) override { return ::remove(full(path).c_str()) == 0; }
    bool rename(const char *from, const char *to) override
    {
        return ::rename(full(from).c_str(), full(to).c_str()) == 0;
    }
    bool mkdir(const char *path) override { return ::mkdir(full(path).c_str(), 0755) == 0; }
    bool rmdir(const char *path) override { return ::rmdir(full(path).c_str()) == 0; }
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

    size_t storageSize() override { return 0; }
    size_t storageAvailable() override { return 0; }

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
    FILE *fp = fopen("/fs/node.log", "a");
    if (!fp)
        return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(fp, fmt, ap);
    va_end(ap);
    fputc('\n', fp);
    fclose(fp);
}

static void node_task(void *arg)
{
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
    s_filesystem = new MountedFileSystem("/fs");
    bool fs_ok = s_filesystem.init();
    RNS::Utilities::OS::register_filesystem(s_filesystem);
    node_log("rns: storage /fs init=%d", (int)fs_ok);

    node_log("rns: constructing");
    s_reticulum = RNS::Reticulum();
    node_log("rns: constructed");
    s_reticulum.transport_enabled(true);
    s_reticulum.start();
    node_log("rns: started");

    /* Identity: recalled from flash so the destination hash is stable across
     * reboots, which is the whole point of a node having an address. */
    node_log("rns: loading identity");
    s_identity = RNS::Identity::from_file(IDENTITY_FILE);
    node_log("rns: identity loaded");
    if (!s_identity)
    {
        ESP_LOGI(TAG, "no identity at %s, generating", IDENTITY_FILE);
        s_identity = RNS::Identity();
        if (!s_identity.to_file(IDENTITY_FILE))
            ESP_LOGW(TAG, "could not persist identity; it will change on reboot");
    }
    snprintf(s_identity_hex, sizeof(s_identity_hex), "%s",
             s_identity.hash().toHex().c_str());
    node_log("identity=%s", s_identity_hex);
    ESP_LOGI(TAG, "identity %s", s_identity_hex);

    node_log("rns: creating interface");
    s_udp = new UDPInterface();
    s_udp.mode(RNS::Type::Interface::MODE_GATEWAY);
    RNS::Transport::register_interface(s_udp);
    s_udp.start();

    s_state = NODE_READY;
    node_log("state=ready ip=%s", s_ip);
    ESP_LOGI(TAG, "Reticulum ready on %s", s_ip);

    uint32_t elapsed = 0;
    uint32_t next_announce = (ANNOUNCE_MIN_S +
        (esp_random() % (ANNOUNCE_MAX_S - ANNOUNCE_MIN_S))) ;
    s_announced_once = false;

    for (;;)
    {
        s_reticulum.loop();

        if (!s_announced_once)
        {
            s_announced_once = true;
            ESP_LOGI(TAG, "first announce");
        }
        else if (elapsed >= next_announce || s_announce_requested)
        {
            s_announce_requested = false;
            elapsed = 0;
            next_announce = (ANNOUNCE_MIN_S +
                (esp_random() % (ANNOUNCE_MAX_S - ANNOUNCE_MIN_S)));
        }

        if (s_persist_requested)
        {
            s_persist_requested = false;
            RNS::Transport::persist_data();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed += 1;
    }
}

/* ---- public --------------------------------------------------------- */

void node_start(void)
{
    if (s_state != NODE_OFF)
        return;
    s_state = NODE_WIFI;
    xTaskCreate(node_task, "rns", 16384, NULL, 4, NULL);
}

node_state_t node_state(void) { return s_state; }

const char *node_state_text(void)
{
    switch (s_state)
    {
    case NODE_WIFI:         return "wifi connecting";
    case NODE_NO_CONFIG:    return "no wifi credentials";
    case NODE_WIFI_FAILED:  return "wifi failed";
    case NODE_RETICULUM:    return "reticulum starting";
    case NODE_READY:        return "online";
    case NODE_FAILED:       return "reticulum failed";
    default:                return "off";
    }
}

const char *node_identity_hex(void) { return s_identity_hex; }
const char *node_ip(void) { return s_ip; }

void node_get_stats(struct node_stats *out)
{
    if (out)
        *out = s_stats;
}

void node_persist(void) { s_persist_requested = true; }
void node_announce_now(void) { s_announce_requested = true; }
