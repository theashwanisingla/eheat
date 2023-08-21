#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <time.h>
#include <sys/time.h>

#include <ping/ping.h>

#include "esp_http_client.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "mdns.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
// #include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_attr.h"
#include "esp_sleep.h"

#include "esp_mac.h"

#include "nvs_flash.h"
#include <assert.h>
#include "esp_partition.h"
#include "spi_flash_mmap.h"
#include "nvs.h"

#include "esp_sntp.h"
#include "esp_bt.h"

#include "esp_blufi.h"
#include "esp_blufi_api.h"
#include "blufi_init.h"
#include "eheat_ota_config.h"

#include "u8g2_esp32_hal.h"
// #include "icons.h"
#include <math.h>
#include "cJSON.h"

// #include "driver/rtc_io.h"
// #include "driver/rtc_cntl.h"
#include "esp_timer.h"

#include "esp_sleep.h"
#include "driver/gptimer.h"
#include "shadow_helpers.h"
// #include "mdns.h"

#define NVS_NAMESPACE "nvs"
#define NVS_KEY "schedule_data"

/*----------------   START ADC REFERENCE VALUE DEFINITION   ---------------------*/
#define USE_ADC
#ifdef USE_ADC
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF
#define tMax_c 2.78
#define adcMax 4096
#define Vs 3.3
#define R1 10000.0  // Voltage Divider R1 Value.
#define Beta 3380.0 // Beta Value Obtained from data sheet
#define To 298.15   // Temperature in kelvin for 25 Degree Celcius
#define Ro 10000.0  // Resistance values at To, Obtained from datasheet.
#define LUX_SCALE_FACTOR (100.0 / 1024.0)
#define ADC_SAMPLES 100
#endif
/*----------------   END ADC REFERENCE VALUE DEFINITION   ---------------------*/
#define FREEZE_PROTECTION_POINT_CEL 5
#define FREEZE_PROTECTION_POINT_FAH 40
// Threshold value that should increase once freeze point is detected
#define FREEZE_OFF_THRESHOLD 2

#define ESP_INTR_FLAG_DEFAULT 0
#define DISPLAY_TIMEOUT 30
#define BLUFI_TIMEOUT 100

#define MIN_SUPPORTED_INTENSITY 1
#define MAX_SUPPORTED_INTENSITY 255
#define MIN_INPUT_INTENS 1
#define MAX_INPUT_INTENS 100
#define AUTO_MIN_OUT_INTENS 5
#define AUTO_INTENS_INPUT_MIN 0
#define AUTO_INTENS_INPUT_MAX 3030

#define PILOT_LIGHT_MIN_INTENS MIN_SUPPORTED_INTENSITY
#define PILOT_LIGHT_MAX_INTENS MAX_SUPPORTED_INTENSITY
#define DISPLAY_LIGHT_MIN_INTENS MIN_SUPPORTED_INTENSITY
#define DISPLAY_LIGHT_MAX_INTENS MAX_SUPPORTED_INTENSITY
#define NIGHT_LIGHT_MIN_INTENS MIN_SUPPORTED_INTENSITY
#define NIGHT_LIGHT_MAX_INTENS 16
// output_start + ((output_end - output_start) * (input - input_start)) / (input_end - input_start)
#define GET_NIGHT_LIGHT_INTENS(INTENSITY) (NIGHT_LIGHT_MIN_INTENS + (NIGHT_LIGHT_MAX_INTENS - NIGHT_LIGHT_MIN_INTENS) * (INTENSITY - MIN_INPUT_INTENS) / (MAX_INPUT_INTENS - MIN_INPUT_INTENS))
#define GET_PILOT_LIGHT_INTENS(INTENSITY) (PILOT_LIGHT_MIN_INTENS + (PILOT_LIGHT_MAX_INTENS - PILOT_LIGHT_MIN_INTENS) * (INTENSITY - MIN_INPUT_INTENS) / (MAX_INPUT_INTENS - MIN_INPUT_INTENS))
#define GET_DISPLAY_INTENS(INTENSITY) (DISPLAY_LIGHT_MIN_INTENS + (PILOT_LIGHT_MAX_INTENS - DISPLAY_LIGHT_MIN_INTENS) * (INTENSITY - MIN_INPUT_INTENS) / (MAX_INPUT_INTENS - MIN_INPUT_INTENS))
#define GET_AUTO_INTENS(INTENSITY) (AUTO_MIN_OUT_INTENS + (MAX_INPUT_INTENS - AUTO_MIN_OUT_INTENS) * (INTENSITY - AUTO_INTENS_INPUT_MIN) / (AUTO_INTENS_INPUT_MAX - AUTO_INTENS_INPUT_MIN))

// (params->heater.s_temp * 9) / 5 + 32;
#define GET_FAHRENHEIT(TEMP) ((TEMP * 9) / 5 + 32)
// (45°F − 32) × 5/9
#define GET_CELSIUS(TEMP) ((TEMP - 32) * 5 / 9)
#define TEMPERATURE_HYSTERISIS_OFFSET 0.5
#define ROOM_TEMP_REFRESH_SEC 20
/*----------------   START GPIO DEFINITION   ---------------------*/
#define USE_DISPLAY
#ifdef USE_DISPLAY
// Display I2C pins
#define PIN_SDA GPIO_NUM_16
#define PIN_SCL GPIO_NUM_17
#define OLED_PwM_DIM GPIO_NUM_27
#define OLED_RESET GPIO_NUM_15
#define OLED_POWER_CTL GPIO_NUM_4

// #define PIN_SDA GPIO_NUM_21      //
// #define PIN_SCL GPIO_NUM_22

#endif
#define USE_GPIO
#ifdef USE_GPIO

// Heater Panel
#define GPIO_HEATER_PANEL GPIO_NUM_22

// PILOT LED PINS
#define PILOT_LED1 GPIO_NUM_23
#define PILOT_LED2 GPIO_NUM_18

// NIGHT LIGHT PINS
#define RED_PIN GPIO_NUM_12
#define GREEN_PIN GPIO_NUM_13
#define BLUE_PIN GPIO_NUM_2
// INPUT BUTTON PIN

// #define BUTTON_UP_GPIO 25
// #define BUTTON_DOWN_GPIO 26
// #define BUTTON_SELECT_GPIO 35
// #define BUTTON_BACK_GPIO 34
#endif
/*----------------   END GPIO DEFINITION   ---------------------*/

/*----------------   START PWM DEFINITION   ---------------------*/
#define PWM_CHANNEL_RED LEDC_CHANNEL_0
#define PWM_CHANNEL_GREEN LEDC_CHANNEL_1
#define PWM_CHANNEL_BLUE LEDC_CHANNEL_2
#define PILOT_LED1_CHANNEL LEDC_CHANNEL_3
#define PILOT_LED2_CHANNEL LEDC_CHANNEL_4
#define PWM_FREQ_HZ 4000
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
/*----------------   END PWM DEFINITION   ---------------------*/

/*----------------   START TIMER DEFINITION   ---------------------*/
#define USE_TIMER
#ifdef USE_TIMER
#define TIMER_INTERVAL_SEC 1
#define TIMER_CHANGE_INTERVAL 1800
#define TIMER_CHANGE_STEP 6
// timer_macro_funcions
#define Hours(seconds) (seconds / 3600)
#define Minutes(seconds) ((seconds - ((seconds / 3600) * 3600)) / 60)
#define Seconds(seconds) (seconds - ((seconds / 60) * 60))
#endif
/*----------------   END TIMER DEFINITION   ---------------------*/
#define BUTTON_LONGPRESS_DURATION 2
/*----------------   DISPLAY DEFINITION     ------------------------*/
#define NUM_ITEMS 10       // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
#define MAX_ITEM_LENGTH 30 // maximum characters for the item name

// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes

#define BUTTON_UP_PIN 26     // pin for UP button
#define BUTTON_SELECT_PIN 34 // pin for SELECT button
#define BUTTON_DOWN_PIN 25   // pin for DOWN button
#define BUTTON_BACK_PIN 35

#define MAC "%s"
#define MQTT(name) "/cmd/eheat/" MQTT_ENV name "/health"

#define MQTT_DEVICE_SHADOW_BUFFER_SIZE 4096

extern u8g2_t u8g2;
extern uint8_t disp_intensity;
extern float disp_temperature;
extern char disp_time[9];
extern char temperature_buf[15];
extern uint8_t temp_value;
extern char timer_set_buffer[6];
extern volatile bool tim_timer_mode_started;
extern bool wifi_connect_state;
extern bool wifi_change_state;
extern char mac_id[12];
extern char envo[6];
extern int32_t blufi_state; // value will default to 0, if not set yet in NVS
extern bool ble_is_connected;
extern bool ble_is_intialised;
extern bool t_unit_flag;
extern int prev_time;

extern int refresh_temperature;
extern int temperature_offset;
extern int ambient_temperature;
extern double report_ambient_temperature;
extern TaskHandle_t routine_taskHandler;
extern TaskHandle_t shadow_taskHandler;
extern TaskHandle_t display_taskHandler;

extern uint8_t heater_value;
extern bool heat_on_freeze_protect;
extern bool flag_button_up;
extern bool flag_button_down;
extern bool flag_button_select;
extern bool flag_button_back;
extern uint8_t gb_freeze_protection_flag;
extern bool panelStateChanged_night_light;
extern bool panelStateChanged_pilot_light;
extern bool panelStateChanged_display;
extern bool panelStateChanged_heater;
extern bool panelStateChanged_child_lock;
extern bool panelStateChanged_freeze_lock;
extern bool panelStateChanged_timer;
extern bool panelStateChanged_op_mode;
extern uint8_t select_button_press_type;
extern bool panelStateChanged_t_unit;
extern int current_screen;
extern int calibration_flag;
extern uint8_t operation_mode;
extern uint8_t prev_operation_mode;
extern bool ota_update_flag;
extern uint8_t gb_hour;
extern uint8_t gb_minute;
extern uint8_t gb_weekday;
extern bool gb_schedule_flag;

extern bool heater_state_changed;
extern bool heater_on_flag;
extern bool net_connection_flag;

extern bool display_timeout_flag;
extern bool display_off_flag;
extern uint8_t day_light_saving_flag;
extern int64_t disp_prev_time;
extern char timezone[10];

extern volatile bool timer_stopped_flag;
extern gptimer_handle_t gptimer;
extern uint64_t current_epoch;

/**
 * This enum defines button state as long pressed or short pressed
 */
typedef enum down_button_press_type
{
    shortPress = 0,
    longPress = 1,
    not_pressed = 2
} BUTTON_PRESS;

/**
 * This enum defines button state as long pressed or short pressed
 */
typedef enum button_state
{
    pressed = 0,
    released = 1
} BUTTON_STATE;

/**
 * @brief This defines operation mode for the device.
 */
typedef enum operation_mode
{
    OFF = 0,
    ON = 1,
    AUTO = 2,
    TIMER = 3,
    SCHEDULE = 4,
    GEO_FENCE = 4,
    MANUAL = 1,
    DISP_AUTO = 0,
    TIMEOUT = 1,
    LOW = 0,
    HIGH = 1
} OP_MODE;

/**
 * @brief This defines operation mode for the device.
 */
typedef enum config_mode
{
    OFF_MODE = 0,
    AUTO_MODE = 1,
    TIMER_MODE = 2,
    SCHEDULE_MODE = 3,
    GEOFENCE_MODE = 4
} CONF_MODE;

/**
 * @brief Heater Response Struct
 */
typedef struct panel
{
    /** @brief This stores State value. */
    volatile uint8_t state;
    /** @brief This stores the set heater temperature on the device(changes with op_mode). */
    float s_temp;
    /** @brief This stores the ambient heater temperature on the device(changes with op_mode). */
    uint8_t a_temp;
    /** @brief This stores the ambient heater temperature in ferhanite on the device(changes with op_mode). */
    float s_temp_fer;

} heater_t;

/**
 * @brief TIMER ACTIVITY Response Struct
 */
typedef struct
{
    /** @brief This stores State value. */
    uint8_t state;
    /** @brief This stores timeout value. */
    uint8_t t_value;
    /** @brief This is will set the timer temperature. */
    uint8_t t_temp;
} heat_timer_t;

typedef struct
{
    uint8_t cert[2000];
    uint16_t cert_len;
} client_cert_t; // used to store the client certificates.

typedef struct
{
    heater_t heater;
    heater_t auto_heater;
    heater_t schedule_heater;
    heater_t geofence_heater;
    heater_t f_protect_heater;

    struct
    {
        /** @brief This stores State value. */
        uint8_t state;
        /** @brief This struct stores RGB values for the night light color. */
        struct
        {
            uint8_t red;
            uint8_t green;
            uint8_t blue;
        } color;
        /** @brief This is will set the Brightness control mode. */
        uint8_t intens_type;
        /** @brief This is will set the Brightness of RGB LED. */
        uint8_t intensity;
    } night_light;

    struct
    {
        /** @brief This stores State value. */
        uint8_t state;
        /** @brief This is will set the Brightness of RED LED. */
        uint8_t intensity;
        /** @brief This is will set the Brightness control mode. */
        uint8_t intens_type; // 0 for always on , 1 for auto dim -> changing intensity.
    } pilot_light;

    struct
    {
        /** @brief This stores State value. */
        uint8_t state;
        /** @brief This stores display timeout value. */
        uint8_t t_out;
        /** @brief This is will set the Brightness oled display. */
        uint8_t intensity;
        /** @brief This is will set the Brightness as per ambient light sensor. */
        uint8_t intens_type;
    } display;

    struct
    {
        /** @brief This stores State value. */
        uint8_t state;
        /** @brief This stores timeout value. */
        uint8_t t_value;
        /** @brief This is will set the timer temperature. */
        float t_temp;
        /** @brief This is will set the timer t_action. */
        uint8_t t_action;
        /** @brief This is will set the timer s_time. */
        uint64_t s_time;
    } heat_timer;

    /// @brief this stoare the tempearature unit 0-> celcius, 1-> ferhanites
    uint8_t t_unit;
    uint8_t c_lock;
    uint8_t f_protection;
    uint8_t op_mode;
    uint8_t hold;

} params_t;

extern params_t gb_params;

#define MAX_EVENTS 12
#define WEEK_DAYS 7

typedef struct Schedule
{
    uint8_t hour[MAX_EVENTS];   // hour-> int
    uint8_t minute[MAX_EVENTS]; // minute-> uint8_t
    uint8_t num_events;
    float temperature[MAX_EVENTS];
} sch_t; // 12*7 = 84 events

extern sch_t gb_sch[WEEK_DAYS];
//---------------------

/**
 * @brief This function initialises the BLUFI bluethoot protocol for Device Provision.
 * @param NONE
 */
void initialise_blufi(void);
/**
 * @brief This function initialises the DISPLAY.
 * @param NONE
 */
esp_err_t initialise_display(void);
/**
 * @brief This function initialises the ADC.
 * @param NONE
 */
void initialise_adc(void);
/**
 * @brief This is a TASK function to read the TEMPERATURE AND LIGHT SENSOR values.
 * @param args PASS NULL
 */
void read_adc_value(void *args);
int32_t get_provisioning_state();
void esp_disable_blufi(void);
void read_temp_task(void *args);
void initialise_gpio(void);
void aws_iot_shadow_main(void);

void get_time_update(void);
void obtain_time(void);
void initialize_sntp(void);
void timer_stop_call(void);
void timer0_callback(void);
void initialise_timer(void);
void timer_stop_lp_buttonpress(gptimer_handle_t timer);
void initialise_tim_select_button_detect(void);
void initialise_tim_timer_button_detect(void);
void initialise_tim_combo_button_detect(void);
esp_err_t initialise_wifi(void);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
int softap_get_current_connection_number(void);
void blufi_record_wifi_conn_info(int rssi, uint8_t reason);
bool blufi_wifi_reconnect(void);
void blufi_wifi_connect(void);
void initialise_pwm(void);
void set_color(params_t *params);
void set_pilot_light(params_t *params);
void set_display(params_t *params);
void storeIntoPartition(unsigned char *cert, int cert_len, char *partition_name);
void timer_schedule(void);
int aws_iot_ota_main(void);
int startOTA(void);
void save_curr_values(params_t *params);
void update_prev_values();
extern int connectToServerWithBackoffRetries(NetworkContext_t *pNetworkContext);
void show_bootloop(void);
void set_temperature(params_t *params);
void set_timer(params_t *params);
esp_err_t get_mac_id(char *device_uid);
void control_auto_intensity();
void reset_factory_device(void);
void control_child_lock(params_t *params);
int32_t EstablishMqttSession(MQTTEventCallback_t eventCallback);
void setting_env(void);
void setting_default_values();
void set_device_time(int hour, int minutes, int seconds);
void retrieve_string(const char *key, char *buffer, size_t buffer_size);
void store_string(const char *key, const char *value);
void retrieveScheduleData();
void storeScheduleData();
void process_schedule(char *json_data);
void get_schedule_data();
void run_schedule();
void addTime(int *hour, int *minute, int add_hour, int add_minute);
int getTimezoneOffset(const char *timezone_curr);
bool ping_google();
esp_err_t http_event_handler(esp_http_client_event_t *evt);

void callback_button_up(void);
void callback_button_down(void);
void callback_button_select(void);
void callback_button_timer(void);
void timer_callback(void);
void cb_lp_select_button(void);
void cb_lp_timer_button(void);
void cb_lp_combo_button(void);

void show_homescreen();
void show_menu();
void show_pilot_options();
void update_options(int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items);
void update_option_screen(char (*menu_list)[30], int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items);
void display_task(void *args);
void show_brightness();
void show_timer();
void show_enable_disable(char (*menu_list)[30], int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items);
void update_brightness(uint8_t *value);
void update_enable_option(uint8_t *value, int *enable_selected);
void update_timer();
void show_ip_address();
void show_wifi_signal();
void show_wifi_change();
void show_temperature();
void update_child_lock();
void show_firmware_version();
void show_factory_reset();
void show_device_about();
void show_cali();
void show_set_time_day();
void button_state();
void show_present_day();
void show_set_utc_zone();
void show_schedule_with_day();
void show_schedul_clear();

/*  adding MQTT LAYERS */

/**
 * @brief Publish a message to a MQTT topic.
 *
 * @param[in] pTopicFilter Points to the topic.
 * @param[in] topicFilterLength The length of the topic.
 * @param[in] pPayload Points to the payload.
 * @param[in] payloadLength The length of the payload.
 *
 * @return EXIT_SUCCESS if PUBLISH was successfully sent;
 * EXIT_FAILURE otherwise.
 */
int32_t PublishToTopic(const char *pTopicFilter,
                       int32_t topicFilterLength,
                       const char *pPayload,
                       size_t payloadLength);

extern bool stateChanged_timer;

/*  This function is used to calculate the adjustment factor i.e. cHeat and cCool
    These variables should be updated globally so that these can be used by other functions.
    # cHeat: An adjustment factor used to account for prior cycling when heating in seconds
    # cCool: An adjustment factor used to account for prior cycling when cooling in seconds*/
void set_adjustment_factor();
void check_freeze_protection(double Tc, double Tf);
float verify_temperature_celsius(float value);
