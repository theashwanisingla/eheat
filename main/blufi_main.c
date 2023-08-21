
#include "eheat.h"

#define WIFI_CONNECTION_MAXIMUM_RETRY CONFIG_WIFI_CONNECTION_MAXIMUM_RETRY
#define INVALID_REASON 255
#define INVALID_RSSI -128
int32_t blufi_state = 0;

static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);
int dis_ota();
int startOTA();
#define WIFI_LIST_NUM 10

static wifi_config_t sta_config;
// static wifi_config_t ap_config;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static uint8_t example_wifi_retry = 0;

/* store the station info for send back to phone */
static bool gl_sta_connected = false;
static bool gl_sta_got_ip = false;
bool ble_is_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;
static wifi_sta_list_t gl_sta_list;
static bool gl_sta_is_connecting = false;
static esp_blufi_extra_info_t gl_sta_conn_info;
bool wifi_connect_state = false;
char mac_id[12] = {0};
char envo[6] = {0};
bool ble_is_intialised = false;

static uint8_t server_if;
static uint16_t conn_id;

static uint8_t blufi_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};

static esp_ble_adv_data_t blufi_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = blufi_service_uuid128,
    .flag = 0x6,
};

void blufi_record_wifi_conn_info(int rssi, uint8_t reason)
{
    memset(&gl_sta_conn_info, 0, sizeof(esp_blufi_extra_info_t));
    if (gl_sta_is_connecting)
    {
        gl_sta_conn_info.sta_max_conn_retry_set = true;
        gl_sta_conn_info.sta_max_conn_retry = WIFI_CONNECTION_MAXIMUM_RETRY;
    }
    else
    {
        gl_sta_conn_info.sta_conn_rssi_set = true;
        gl_sta_conn_info.sta_conn_rssi = rssi;
        gl_sta_conn_info.sta_conn_end_reason_set = true;
        gl_sta_conn_info.sta_conn_end_reason = reason;
    }
}

void blufi_wifi_connect(void)
{
    example_wifi_retry = 0;
    gl_sta_is_connecting = (esp_wifi_connect() == ESP_OK);
    blufi_record_wifi_conn_info(INVALID_RSSI, INVALID_REASON);
}

bool blufi_wifi_reconnect(void)
{
    bool ret;
    if (gl_sta_is_connecting && example_wifi_retry++ < WIFI_CONNECTION_MAXIMUM_RETRY)
    {
        BLUFI_INFO(" Wi-Fi starts reconnection\n");
        gl_sta_is_connecting = (esp_wifi_connect() == ESP_OK);
        blufi_record_wifi_conn_info(INVALID_RSSI, INVALID_REASON);
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

int softap_get_current_connection_number(void)
{
    esp_err_t ret;
    ret = esp_wifi_ap_get_sta_list(&gl_sta_list);
    if (ret == ESP_OK)
    {
        return gl_sta_list.num;
    }

    return 0;
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    wifi_mode_t mode;

    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
    {
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_wifi_get_mode(&mode);

        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(info.sta_bssid, gl_sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = gl_sta_ssid;
        info.sta_ssid_len = gl_sta_ssid_len;
        gl_sta_got_ip = true;
        wifi_connect_state = true;
        if (ble_is_connected == true)
        {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, softap_get_current_connection_number(), &info);
        }
        else
        {
            // BLUFI_INFO("BLUFI BLE is not connected yet\n");

        }
        break;
    }
    default:
        break;
    }
    return;
}

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    wifi_event_sta_connected_t *event;
    wifi_event_sta_disconnected_t *disconnected_event;

    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        blufi_wifi_connect();
        break;

    case WIFI_EVENT_STA_CONNECTED:
        gl_sta_connected = true;
        gl_sta_is_connecting = false;
        event = (wifi_event_sta_connected_t *)event_data;
        memcpy(gl_sta_bssid, event->bssid, 6);
        memcpy(gl_sta_ssid, event->ssid, event->ssid_len);
        gl_sta_ssid_len = event->ssid_len;
        wifi_connect_state = true;
        if (wifi_change_state == true)
        {
            wifi_change_state = false;
            BLUFI_INFO(" got the ssid n stoppig \n");
            // esp_netif_destroy_default_wifi();
            esp_wifi_stop();
            BLUFI_INFO(" wifi deinit\n");
            esp_wifi_deinit();
            BLUFI_INFO(" wifi init \n");
            initialise_wifi();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        /* Only handle reconnection during connecting */

        if (gl_sta_connected == false && blufi_wifi_reconnect() == false)
        {
            gl_sta_is_connecting = false;
            disconnected_event = (wifi_event_sta_disconnected_t *)event_data;
            blufi_record_wifi_conn_info(disconnected_event->rssi, disconnected_event->reason);
        }

        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        gl_sta_connected = false;

        gl_sta_got_ip = false;
        memset(gl_sta_ssid, 0, 32);
        memset(gl_sta_bssid, 0, 6);
        gl_sta_ssid_len = 0;
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        BLUFI_INFO(" Wi-Fi disconnected\n");
        wifi_connect_state = false;

        break;
    default:
        break;
    }
    return;
}

// Initialize the WIFI interface
esp_err_t initialise_wifi(void)
{
    esp_err_t err = ESP_FAIL;
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, mac_id));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    blufi_record_wifi_conn_info(INVALID_RSSI, INVALID_REASON);
    err = esp_wifi_start();
    vTaskDelay(2000 / portTICK_PERIOD_MS); // sleep for 2 seconds after trying to connect wifi connection
    esp_wifi_connect();
    ESP_LOGI("Eheat", " Establishing Wifi Connection, WIFI Tries %d", CONFIG_WIFI_CONNECTION_MAXIMUM_RETRY);
    return err;
}

// Configure callback function for bluetooth connection.
static esp_blufi_callbacks_t blufi_callbacks = {
    .event_cb = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */

    static client_cert_t client_certificate;
    static client_cert_t client_certificate_key;

    switch (event)
    {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");

        esp_ble_gap_set_device_name(mac_id);
        esp_ble_gap_config_adv_data(&blufi_adv_data);

        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        BLUFI_INFO("BLUFI deinit finish\n");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        BLUFI_INFO("BLUFI ble connect\n");
        ble_is_connected = true;
        esp_blufi_adv_stop();
        blufi_security_init();
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:

        if ((client_certificate.cert_len >= 1000) && (client_certificate_key.cert_len >= 1500))
        {
            BLUFI_INFO("Device is Provisioned and disconnecting Blurtooth\n");
            ble_is_connected = false;
            blufi_security_deinit();
            // Find the Partition
            const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
            assert(partition != NULL);
            // Erase entire partition
            ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));

            // Write the data, starting from the beginning of the partition
            ESP_ERROR_CHECK(esp_partition_write(partition, 0, client_certificate.cert, sizeof(client_certificate.cert)));
            // ESP_LOGI("CERT", "Written data: %s", store_data);
            //
            memset(client_certificate.cert, 0, sizeof(client_certificate.cert_len));
            // Read back the data, checking that read data and written data match
            ESP_ERROR_CHECK(esp_partition_read(partition, 0, client_certificate.cert, sizeof(client_certificate.cert_len)));
            ESP_LOGI("CERTS", "Read data:\n %s", client_certificate.cert);

            //////////////////////////
            nvs_handle_t my_handle;
            esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
            if (err != ESP_OK)
            {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            }
            // int32_t cert_len = param->ca.cert_len; // value will default to 0, if not set yet in NVS
            err = nvs_set_i32(my_handle, "cert_len", client_certificate.cert_len);
            printf((err != ESP_OK) ? "cert lenght Failed!\n" : "Cert Length done\n");
            // Erase entire partition
            const esp_partition_t *partition1 = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "certs");
            assert(partition1 != NULL);

            ESP_ERROR_CHECK(esp_partition_erase_range(partition1, 0, partition1->size));

            // Write the data, starting from the beginning of the partition
            ESP_ERROR_CHECK(esp_partition_write(partition1, 0, client_certificate_key.cert, sizeof(client_certificate_key.cert)));
            //
            // Read back the data, checking that read data and written data match
            memset(client_certificate_key.cert, 0, sizeof(client_certificate_key.cert));
            // Read back the data, checking that read data and written data match
            ESP_ERROR_CHECK(esp_partition_read(partition1, 0, client_certificate_key.cert, sizeof(client_certificate_key.cert)));
            ESP_LOGI("CERTS", "Read data:\n %s", client_certificate_key.cert);

            // int32_t pkey_len = param->client_cert.cert_len; // value will default to 0, if not set yet in NVS
            err = nvs_set_i32(my_handle, "pkey_len", client_certificate_key.cert_len);
            printf((err != ESP_OK) ? "pkey_len Failed!\n" : " pkey_len Done\n");
            int32_t value = true; // value will default to 0, if not set yet in NVS
            err = nvs_set_i32(my_handle, "blufi_state", value);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            // Close
            nvs_close(my_handle);
            esp_restart();
        }
        else
        {
            BLUFI_INFO("Failed to provision the device\n");
            ble_is_connected = false;
            blufi_security_deinit();
            BLUFI_INFO("Starting Again ..\n");
            esp_ble_gap_set_device_name(mac_id);
            esp_ble_gap_config_adv_data(&blufi_adv_data);

            if (wifi_change_state == false && wifi_connect_state == true)
            {
                BLUFI_INFO("disabing ble\n");
                esp_restart();
            }
        }
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        BLUFI_INFO("BLUFI Set WIFI opmode %d\n", param->wifi_mode.op_mode);
        ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("BLUFI requset wifi connect to AP\n");
        /* there is no wifi callback when the device has already connected to this wifi
        so disconnect wifi before connection.
        */
        esp_wifi_disconnect();
        blufi_wifi_connect();
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("BLUFI requset wifi disconnect from AP\n");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("BLUFI report error, error code %d\n", param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS:
    {
        wifi_mode_t mode;
        esp_blufi_extra_info_t info;

        esp_wifi_get_mode(&mode);

        if (gl_sta_connected)
        {
            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            esp_blufi_send_wifi_conn_report(mode, gl_sta_got_ip ? ESP_BLUFI_STA_CONN_SUCCESS : ESP_BLUFI_STA_CONN_FAIL, softap_get_current_connection_number(), &info);
        }
        else if (gl_sta_is_connecting)
        {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_get_current_connection_number(), &gl_sta_conn_info);
        }
        else
        {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_get_current_connection_number(), &gl_sta_conn_info);
        }
        BLUFI_INFO("BLUFI get wifi status from AP\n");
        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        BLUFI_INFO("blufi close a gatt connection");
        esp_blufi_disconnect();
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
    case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA BSSID %s\n", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA SSID %s\n", sta_config.sta.ssid);
        break;
    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA PASSWORD %s\n", sta_config.sta.password);
        break;

    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        BLUFI_INFO("Recv Custom Data %d\n", param->custom_data.data_len);
        char buffer[50];
        for (int i = 0; i < param->custom_data.data_len; i++)
        {
            printf("%c", param->custom_data.data[i]);
            buffer[i] = param->custom_data.data[i];
        }
        buffer[param->custom_data.data_len] = "\0";
        printf(" Rececived Custom\n");

        cJSON const *env = cJSON_Parse(&buffer);
        cJSON const *env_state = cJSON_GetObjectItem(env, "env");
        if (env_state != NULL)
        {
            ESP_LOGI("MESSAGE", "env_state %d", env_state->valueint);
            nvs_handle_t my_handle;
            esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
            if (err != ESP_OK)
            {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            }
            // storing the display info into the nvs
            err = nvs_set_i32(my_handle, "env_state", env_state->valueint);
            err = nvs_commit(my_handle);
            nvs_close(my_handle);
        }
        cJSON const *timezone_re = cJSON_GetObjectItem(env, "tz");
        if (timezone_re != NULL)
        {
            ESP_LOGI("MESSAGE", "timezone_re %s", timezone_re->valuestring);

            char *minus_ptr = strchr(timezone_re->valuestring, '-');
            char *plus_ptr = strchr(timezone_re->valuestring, '+');

            if (minus_ptr != NULL)
            {
                *minus_ptr = '+';
            }
            if (plus_ptr != NULL)
            {
                *plus_ptr = '-';
            }

            ESP_LOGI("MESSAGE", "timezone_re after correcting %s", timezone_re->valuestring);
            // storing the display info into the nvs
            store_string("timezone", timezone_re->valuestring);
        }
        break;

    case ESP_BLUFI_EVENT_RECV_USERNAME:
        BLUFI_INFO("ESP_BLUFI_EVENT_RECV_USERNAME");
        /* Not handle currently */
        break;
    case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        BLUFI_INFO("ESP_BLUFI_EVENT_RECV_CA_CERT");
        BLUFI_INFO("Recv Custom Data %d\n", param->ca.cert_len);
        memcpy(client_certificate.cert, param->ca.cert, param->ca.cert_len);
        client_certificate.cert_len = param->ca.cert_len;

        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        BLUFI_INFO("ESP_BLUFI_EVENT_RECV_CLIENT_CERT");
        BLUFI_INFO("Recv Custom Data %d\n", param->client_cert.cert_len);
        printf(" Rececived Private key\n");

        memcpy(client_certificate_key.cert, param->client_cert.cert, param->client_cert.cert_len);
        client_certificate_key.cert_len = param->client_cert.cert_len;
        break;
    case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        printf("\n");
        BLUFI_INFO("\nESP_BLUFI_EVENT_RECV_SERVER_CERT");
        // memcpy(cert + count2, param->client_pkey.pkey, param->client_pkey.pkey_len);
        // count2 = count2 + param->client_pkey.pkey_len;
        memcpy(client_certificate.cert + client_certificate.cert_len, param->server_cert.cert, param->server_cert.cert_len);
        client_certificate.cert_len = client_certificate.cert_len + param->server_cert.cert_len;
        break;
    case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        BLUFI_INFO("\nESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY");
        // client_pkey.pkey_len
        // memcpy(cert1 + count1, param->client_pkey.pkey, param->client_pkey.pkey_len);
        // count1 = count1 + param->client_pkey.pkey_len;
        memcpy(client_certificate_key.cert + client_certificate_key.cert_len, param->client_pkey.pkey, param->client_pkey.pkey_len);
        client_certificate_key.cert_len = client_certificate_key.cert_len + param->client_pkey.pkey_len;
        break;

    case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        BLUFI_INFO("ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY");
        break;
    default:
        break;
    }
}

void initialise_blufi(void)
{
    esp_err_t ret;
    if (wifi_change_state == true || blufi_state != true)
    {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret)
        {
            BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret)
        {
            BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        ret = esp_blufi_host_and_cb_init(&blufi_callbacks);
        if (ret)
        {
            BLUFI_ERROR("%s initialise failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
    }
    else
    {
        esp_disable_blufi();
    }
}

// disable the bluetooth connection
void esp_disable_blufi(void)
{
    // Disable the  Bluetooth   interface
    esp_blufi_disconnect();
    esp_blufi_adv_stop();
    blufi_security_deinit();
    esp_blufi_host_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bt_mem_release(ESP_BT_MODE_BLE));
}