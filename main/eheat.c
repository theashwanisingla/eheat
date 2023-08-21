#include "eheat.h"

bool heat_on_freeze_protect = false;
bool t_unit_flag = true; // 0 -> ferhanite, 1-> celcius
bool net_connection_flag = false;
bool gb_schedule_flag = false;
bool heater_on_flag = false;
bool heater_state_changed = false;
bool heater_last_state = false;
volatile bool inc_det = false;
volatile bool dec_det = false;
volatile bool start_det = false;
volatile bool start_det_cloud = false;
volatile bool tim_timer_mode_stop = false;
volatile bool timer_stopped_flag = false;
volatile bool timer_stop_after_complete = false;
volatile bool flag_lp_select_button_complete = false;
volatile bool longpress_select_button_released = true;
volatile bool flag_lp_timer_button_complete = false;
volatile bool longpress_timer_button_released = true;
volatile bool flag_lp_selectbutton_task = false;
volatile bool flag_lp_timerbutton_task = false;
volatile bool flag_select_button_timer_start = false;
volatile bool flag_timer_button_timer_start = false;
volatile bool flag_lp_combo_button_complete = false;
volatile bool child_lock_flag = false; // true if child lock is enabled else false
volatile bool flag_combo_button_timer_start = false;
volatile bool flag_child_lock_select_button_pressed = false;
volatile bool flag_child_lock_timer_button_pressed = false;
volatile bool child_lock_disable_task_start = false;
static bool timeinfo_update = false;

uint8_t operation_mode = 1;
uint8_t prev_operation_mode = 1;
uint8_t select_button_press_type = not_pressed;
uint8_t timer_button_press_type = not_pressed;
uint8_t gb_hour = 0;
uint8_t gb_minute = 0;
uint8_t gb_weekday = 0;
uint8_t current_opmode = AUTO;
uint8_t previous_freeze_protect_state = 8;
uint8_t heater_value = 8;

uint16_t tHeat = 500;
uint16_t tCool = 3000;
uint16_t tElapsedCycle = 0;

int32_t elapsed_secs = 0;
int temperature_offset = 0;
int temperature_offset_in_fer = 0;

uint64_t remaining_timer_seconds = 0;
int64_t disp_prev_time = 0;
uint64_t current_epoch = 0;

time_t manualTime;
time_t currentTime;

double cHeat = 0;
double cCool = 2147483646;
double tDelta = 0;
double heater_state_change_duration = 0;
double heater_last_state_change_duration = 0;

adc_oneshot_unit_handle_t adc1_handle;
TaskHandle_t routine_taskHandler = NULL;
TaskHandle_t shadow_taskHandler = NULL;
TaskHandle_t display_taskHandler = NULL;

struct tm timeinfo = {0};

char strftime_buf[64];
extern char disp_time[9];
extern u8g2_t u8g2;

sch_t gb_sch[WEEK_DAYS] = {0};

gptimer_alarm_config_t alarm_config;
gptimer_handle_t gptimer;
gptimer_alarm_config_t alarm_config_select_button_detect;
gptimer_handle_t gptimer_select_button_detect;
gptimer_alarm_config_t alarm_config_timer_button_detect;
gptimer_handle_t gptimer_timer_button_detect;
gptimer_alarm_config_t alarm_config_combo_button_detect;
gptimer_handle_t gptimer_combo_button_detect;

void initialize_mdns(void) {
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());

    // Set mDNS hostname (e.g., "heater") to be discoverable as "heater.local"
    ESP_ERROR_CHECK(mdns_hostname_set("heater"));

    // Set mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 Heater Controller"));

    // Register an mDNS service
    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"user", "admin"}
    };
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, serviceTxtData, 2));
}

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = get_mac_id(&mac_id);
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Failed to fetch MAC ID (error code %d)\n", ret);
        esp_restart();
    }

    ret = initialise_wifi();
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Failed to intialise WIFI (error code %d)\n", ret);
        // esp_restart();
    }
    else
        ESP_LOGI("Eheat", "WIFI intialised");

    // Setting the environment variable
    setting_env();
    ESP_LOGI("Eheat", "Setting ENV variable");

    initialize_mdns();

    // Initialise the PWM
    initialise_pwm();
    ESP_LOGI("Eheat", "Initialised PWM");
    // Initialise the GPIO pins
    initialise_gpio();
    ESP_LOGI("Eheat", "Initialised GPIOS");
    // Initialise the TIMER
    initialise_timer();
    ESP_LOGI("Eheat", "Initialised TIMER");
    // Initialise the ADC
    initialise_adc();
    ESP_LOGI("Eheat", "Initialised ADC");
    // Create the Display Task
    if (pdFALSE == xTaskCreate(&display_task, "Display Task", 3048, NULL, 3, &display_taskHandler))
    {
        ESP_LOGE("Eheat", "Failed to create display task");
        esp_restart();
    }
    if (pdFALSE == xTaskCreate(&read_temp_task, "Temperature Task", 3048, NULL, 3, &routine_taskHandler))
    {
        ESP_LOGE("Eheat", "Failed to create Temperature Read Task");
        esp_restart();
    }

    ESP_LOGI("Eheat", "Getting Provisioning State");
    blufi_state = get_provisioning_state();
    if (blufi_state != true)
    {
        // setting the default state.
        setting_default_values();
        // initializing the bluetooth state.
        initialise_blufi();
        ESP_LOGI("Eheat", "Device is Not Provisioned");
    }
    else
    {
        // get previous values
        update_prev_values();
        ESP_LOGI("Eheat", "Device Provisioned");
        ESP_LOGI("Eheat", "Initialising Device Shadow Task");
        if (pdFALSE == xTaskCreate(&aws_iot_shadow_main, "AWS SHADOW", 4096, NULL, 3, &shadow_taskHandler))
        {
            ESP_LOGE("Eheat", "Failed to device shadow task");
            esp_restart();
        };
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI("Eheat", "Initialising AWS OTA Task");
        aws_iot_ota_main();
    }
}

int32_t get_provisioning_state()
{
    int32_t provisioning_state = 0;
    nvs_handle_t my_handle; // creating handle
    esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    // fetches the value of the provisioning state.
    err = nvs_get_i32(my_handle, "blufi_state", &provisioning_state);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI("Eheat", "BluFI Configure State = %" PRIu32 "\n", provisioning_state);
        nvs_close(my_handle);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI("Eheat", "The value is not initialized yet!\n");
        provisioning_state = false;
        nvs_close(my_handle);
        break;
    default:
        ESP_LOGI("Eheat", "Error (%s) reading!\n", esp_err_to_name(err));
        nvs_close(my_handle);
    }
    return provisioning_state;
}

esp_err_t initialise_display(void)
{
    // initialize the u8g2 hal
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_SDA;
    u8g2_esp32_hal.scl = PIN_SCL;
    u8g2_esp32_hal.reset = OLED_RESET;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    // set the display address
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3c);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        // u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);

    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    show_bootloop();
    return ESP_OK;
}

void initialise_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH_BIT_DEFAULT,
        .atten = ADC_ATTEN,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));
}

void read_temp_task(void *args)
{
    initialise_adc();
    double Vout;
    double Rt = 0;
    double T;
    double Tc = 0;
    double Tf = 0;
    uint32_t Vo = 0;
    uint32_t raw_temp = 0;
    int num = 0;

    while (1)
    {
        static int refresh = 0;
        if (flag_lp_combo_button_complete == true)
        {
            flag_lp_combo_button_complete = false;
            gb_params.c_lock = !gb_params.c_lock;
            panelStateChanged_child_lock = true;
            control_child_lock(&gb_params);
            flag_child_lock_select_button_pressed = true;
            flag_child_lock_timer_button_pressed = true;
            // ESP_LOGI("CHILD-LOCK", "Child lock state is: %d", gb_params.c_lock);
        }
        button_state();
        if (refresh % 10 == 0)
        {
            refresh = 0;

            // Check for the freeze protection
            check_freeze_protection(Tc, Tf);

            // if the heater state toggling condition is triggered, control the heater.
            if (heater_state_changed == true)
            {
                // Turn on the heater panel
                if (heater_on_flag == true)
                {
                    gpio_set_level(GPIO_HEATER_PANEL, HIGH);
                }
                else
                {
                    // Turn off the heater panel
                    gpio_set_level(GPIO_HEATER_PANEL, LOW);
                }

                // get the timestamp when the heater state is cycled
                heater_last_state = heater_on_flag;
                heater_last_state_change_duration = (double)esp_timer_get_time() / (double)1000000;

                // calculate the cHeat and cCool for temperature correction algorithm
                set_adjustment_factor();

                heater_state_changed = false;
            }

            uint32_t free_heap_size = 0;
            uint32_t min_free_heap_size = 0;
            free_heap_size = esp_get_free_heap_size();
            min_free_heap_size = esp_get_minimum_free_heap_size();
            ESP_LOGI("Eheat", "free heap size = %d \t  min_free_heap_size = %d \n", free_heap_size, min_free_heap_size);

            // Attempt a reconnect if WiFi goes down
            if (wifi_connect_state == false)
            {
                blufi_wifi_connect();
            }

            Vo = 0;
            for (int k = 0; k < ADC_SAMPLES; k++)
            {
                adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &raw_temp);
                Vo += raw_temp;
                vTaskDelay(2 / portTICK_PERIOD_MS);
            }
            Vo = Vo / ADC_SAMPLES;

            Vout = Vo * Vs / adcMax;
            Rt = R1 * Vout / (Vs - Vout);
            T = 1 / (1 / To + log(Rt / Ro) / Beta); // Temperature in Kelvin
            Tc = T - 273.15;                        // Celsius

            // adding callibraion offset to the temp.
            Tc += temperature_offset;

            // -------------------- Temperature correction Algorithm---------------------
            double get_time = ((double)esp_timer_get_time() / (double)1000000);
            heater_state_change_duration = get_time - heater_last_state_change_duration;

            // ESP_LOGI("GETTIME", "get time = %f", heater_state_change_duration);
            // Calculate the Delta (tDelta): estimated error between read temperature and Ambient temperature in celcius
            if (heater_on_flag)
            {
                tDelta = tMax_c * (1 - (exp(-(heater_state_change_duration + cHeat) / tHeat)));
            }
            else
            {
                tDelta = tMax_c * (exp(-(heater_state_change_duration + cCool) / tCool));
            }
            Tc -= tDelta;
            Tf = (Tc * 9 / 5) + 32; // Fahrenheit
            report_ambient_temperature = Tc;

            ESP_LOGI("Temperature Values",
                     "\n\tLast state changed duration :\t%f,\n\toffset value \t\t:\t%d\n\tFinal temperature \t:\t%f,\n\tDelta is \t\t:\t%f",
                     heater_state_change_duration,
                     temperature_offset,
                     Tc,
                     tDelta);

            // Check if the temperature has crossed the threshold
            if (gb_params.heater.state)
            {
                // if the temperature is above threshold and last state was on, turn off the heater
                if ((Tc > gb_params.heater.s_temp) && heater_last_state)
                {
                    heater_on_flag = false;
                    heater_state_changed = true;
                }
                // if the temperature is less than threshold and last state was off, turn on the heater
                else if ((Tc < (gb_params.heater.s_temp - TEMPERATURE_HYSTERISIS_OFFSET)) && !heater_last_state)
                {
                    heater_on_flag = true;
                    heater_state_changed = true;
                }
            }

            if (t_unit_flag == true)
            {
                // to avoid Minus sign with value 0
                if ((Tc > -1) && (Tc < 1))
                {
                    sprintf(temperature_buf, "%.fC", (double)0);
                }
                else
                {
                    sprintf(temperature_buf, "%.fC", Tc);
                }
                ambient_temperature = (int)Tc;
            }
            else
            {
                sprintf(temperature_buf, "%.fF", Tf);
                ambient_temperature = (int)Tf;
            }

            if (!blufi_state || !wifi_connect_state)
            {
                get_time_update();
            }

            if (num % 10 == 0)
            {
                run_schedule();
                if (blufi_state == true && wifi_connect_state != false)
                {
                    ping_google();
                }
                get_time_update();
                // ESP_LOGI("Temperature control", " Values  = %f", gb_params.heater.s_temp);
                // ESP_LOGI("Room Temperature ", " Values  = %d", ambient_temperature);
                num = 0;

                if (net_connection_flag == false && wifi_connect_state == true)
                {
                    ESP_LOGW("Eheat", "Connected to WIFI but no internet connection");
                }
                else if (wifi_connect_state == false)
                {
                    ESP_LOGE("Eheat", "Not Connected to WIFI");
                }
                else
                {
                    ESP_LOGI("Eheat", "Connected to WIFI with internet connection");
                }
            }
            num++;

            int64_t current_time = esp_timer_get_time();
            current_time = current_time / 1000000;

            if (((current_time - prev_time) > BLUFI_TIMEOUT) && (ble_is_connected != true && wifi_change_state == true))
            {
                esp_restart();
            }

            if (gb_params.display.state == false || display_off_flag == false)
            {
                if (((current_time - disp_prev_time) > DISPLAY_TIMEOUT) && (display_timeout_flag == false))
                {
                    ESP_LOGI("DISPLAY TOGGLE", "Display timeout completed. Setting flag for display off");
                    display_timeout_flag = true;
                }
            }
            else
            {
                ESP_LOGI("DISPLAY TOGGLE", "Display timeout completed. Setting flag for display off");
                display_timeout_flag = false;
            }

            timer_schedule();
            control_auto_intensity();
        }
        refresh++;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void initialise_gpio(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << BUTTON_UP_PIN | 1ULL << BUTTON_DOWN_PIN | 1ULL << BUTTON_SELECT_PIN | 1ULL << BUTTON_BACK_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // assinging the GPIO Int priority
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);

    gpio_isr_handler_add(BUTTON_UP_PIN, callback_button_up, NULL);
    gpio_isr_handler_add(BUTTON_DOWN_PIN, callback_button_down, NULL);
    gpio_isr_handler_add(BUTTON_SELECT_PIN, callback_button_select, NULL);
    gpio_isr_handler_add(BUTTON_BACK_PIN, callback_button_timer, NULL);

    gpio_set_direction(GPIO_HEATER_PANEL, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_HEATER_PANEL, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(OLED_POWER_CTL, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(OLED_POWER_CTL, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(OLED_PwM_DIM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(OLED_PwM_DIM, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(OLED_RESET, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(OLED_RESET, GPIO_PULLUP_ONLY);
}

void initialise_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    alarm_config.alarm_count = TIMER_CHANGE_INTERVAL * 1000000;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = &timer_callback, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // button event timer initialization.
    initialise_tim_select_button_detect();
    initialise_tim_timer_button_detect();
    initialise_tim_combo_button_detect();
}

void initialise_tim_select_button_detect(void)
{
    gptimer_config_t timer_config1 = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &gptimer_select_button_detect));

    alarm_config_select_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_select_button_detect, &alarm_config_select_button_detect));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = &cb_lp_select_button, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_select_button_detect, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_select_button_detect));
}

void initialise_tim_timer_button_detect(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_timer_button_detect));

    alarm_config_timer_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_timer_button_detect, &alarm_config_timer_button_detect));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = &cb_lp_timer_button, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_timer_button_detect, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_timer_button_detect));
}

void initialise_tim_combo_button_detect(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_combo_button_detect));

    alarm_config_combo_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_combo_button_detect, &alarm_config_combo_button_detect));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = &cb_lp_combo_button, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_combo_button_detect, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer_combo_button_detect));
}

void set_color(params_t *params)
{
    int out = GET_NIGHT_LIGHT_INTENS(params->night_light.intensity);
    // ESP_LOGI("RGB", "INTENSITY %d", out);
    if (params->night_light.state >= ON && params->night_light.intensity != LOW)
    {
        // LogInfo(("Setting up the night lamp"));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_RED, params->night_light.color.red * out);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_GREEN, params->night_light.color.green * out / 2);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BLUE, params->night_light.color.blue * out / 2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_RED);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_GREEN);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BLUE);
    }
    if (params->night_light.state == OFF || params->night_light.intensity == LOW)
    {
        // LogInfo(("Turning of the night lamp"));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_RED, OFF);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_GREEN, OFF);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BLUE, OFF);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_RED);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_GREEN);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BLUE);
    }
}

void initialise_pwm(void)
{
    gpio_set_direction(PILOT_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PILOT_LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_PIN, GPIO_MODE_OUTPUT);

    // configure PWM for LED control
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0};
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t red_channel = {
        .channel = PWM_CHANNEL_RED,
        .duty = 0,
        .gpio_num = RED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};
    ledc_channel_config_t green_channel = {
        .channel = PWM_CHANNEL_GREEN,
        .duty = 0,
        .gpio_num = GREEN_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};
    ledc_channel_config_t blue_channel = {
        .channel = PWM_CHANNEL_BLUE,
        .duty = 0,
        .gpio_num = BLUE_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};

    // pilot light included.
    ledc_channel_config_t red_channel1 = {
        .channel = PILOT_LED1_CHANNEL,
        .duty = 0,
        .gpio_num = PILOT_LED1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};

    ledc_channel_config_t red_channel2 = {
        .channel = PILOT_LED2_CHANNEL,
        .duty = 0,
        .gpio_num = PILOT_LED2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};

    //---------------------------------//
    ledc_channel_config(&red_channel1);
    ledc_channel_config(&red_channel2);
    ///--------------------------------//
    ledc_channel_config(&red_channel);
    ledc_channel_config(&green_channel);
    ledc_channel_config(&blue_channel);
    set_pilot_light(&gb_params);
}

void set_pilot_light(params_t *params)
{
    int out = GET_PILOT_LIGHT_INTENS(params->pilot_light.intensity);
    if (params->heater.state == true)
    {
        uint32_t brightness = ((4096) - (out * 16));
        // ESP_LOGI("PILOT LIGHT", "INTENSITY %d", brightness);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PILOT_LED1_CHANNEL, brightness);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PILOT_LED2_CHANNEL, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PILOT_LED1_CHANNEL);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PILOT_LED2_CHANNEL);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PILOT_LED1_CHANNEL, 4096);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PILOT_LED2_CHANNEL, 4096);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PILOT_LED1_CHANNEL);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PILOT_LED2_CHANNEL);
    }
}

void set_display(params_t *params)
{
    int out = GET_DISPLAY_INTENS(params->display.intensity);
    // ESP_LOGI("Display", "Intensity %d", out);
    disp_intensity = (uint8_t)out;
}

void set_temperature(params_t *params)
{
    switch (operation_mode)
    {
    case AUTO_MODE:
        gb_params.heater.state = params->auto_heater.state;
        gb_params.heater.s_temp = verify_temperature_celsius(gb_params.auto_heater.s_temp);
        gb_params.heater.s_temp_fer = round(((gb_params.heater.s_temp * 9) / 5 + 32));
        ESP_LOGI("MODES", "AUTO MODE ACTIVATED");
        break;
    case TIMER_MODE:
        gb_params.heater.state = gb_params.heat_timer.state;
        gb_params.heater.s_temp = verify_temperature_celsius(gb_params.heat_timer.t_temp);
        gb_params.heater.s_temp_fer = round(((gb_params.heater.s_temp * 9) / 5 + 32));
        ESP_LOGI("MODES", "TIMER MODE ACTIVATED");
        break;
    case SCHEDULE_MODE:
        /* code */
        gb_params.heater.state = gb_params.schedule_heater.state;
        gb_params.heater.s_temp = verify_temperature_celsius(gb_params.schedule_heater.s_temp);
        gb_params.heater.s_temp_fer = round(((gb_params.heater.s_temp * 9) / 5 + 32));
        ESP_LOGI("MODES", "SCHEDULE MODE ACTIVATED");
        break;
    case GEOFENCE_MODE:
        /* code */
        gb_params.heater.state = gb_params.geofence_heater.state;
        gb_params.heater.s_temp = verify_temperature_celsius(gb_params.geofence_heater.s_temp);
        gb_params.heater.s_temp_fer = round(((gb_params.heater.s_temp * 9) / 5 + 32));
        ESP_LOGI("MODES", "GEOFENCE MODE ACTIVATED");
        break;

    default:
        break;
    }
    set_pilot_light(&gb_params);

    if (gb_params.heater.state == false)
    {
        ESP_LOGI("DISPLAY TOGGLE", "Turning OFF");
        display_off_flag = false;
        display_timeout_flag = true;
    }
    else
    {
        ESP_LOGI("DISPLAY TOGGLE", "Turning ON");
        display_off_flag = true;
    }
}

void set_timer(params_t *params)
{
    static uint64_t prev_time = 30;
    get_time_update();
    int64_t t_time = (uint64_t)params->heat_timer.t_value * 60;

    // get the epoch time diff, it will always be zero or negative
    int64_t epoch_diff = params->heat_timer.s_time - current_epoch;

    // Check timer state, if already stopped from the app, do not set time
    if (params->heat_timer.state == ON)
    {
        if (tim_timer_mode_started == false)
        {
            alarm_config.alarm_count = t_time * 1000000;
            gptimer_set_raw_count(gptimer, abs(epoch_diff)*1000000);
            ESP_LOGW("TIMER DURATION1", "%lld", alarm_config.alarm_count);
        }
    }

    // Check if the timer duration has been updated.
    // If duration received is updated, update the timer duration
    if (params->heat_timer.t_value != prev_time)
    {
        alarm_config.alarm_count = t_time* 1000000;
        ESP_LOGW("TIMER DURATION2", "%lld", alarm_config.alarm_count);
        prev_time = params->heat_timer.t_value;
        gptimer_set_raw_count(gptimer, abs(epoch_diff)*1000000);
    }

    // ESP_LOGI("SET TIMER", "%lld", alarm_config.alarm_count);
    if (params->heat_timer.state == ON)
    {
        ESP_LOGI("HEATING TIMER", "%d state :%d temp: %.2f", params->heat_timer.t_value,
                 params->heat_timer.state, params->heat_timer.t_temp);
        start_det_cloud = true;
        elapsed_secs = 0;
        remaining_timer_seconds = 0;
        timer_schedule();
        params->heater.s_temp = params->heat_timer.t_temp;
        params->heater.state = params->heat_timer.state;
        set_temperature(params);
    }
    else
    {
        if (operation_mode == TIMER_MODE)
        {
            timer_stop_call();
        }
    }
}

// SNTP TIMER Callback Functions
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI("SNTP", "Notification of a time synchronization event");
}

void initialize_sntp(void)
{
    ESP_LOGI("SNTP", "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    // timeinfo = {0};
    int retry = 0;
    const int retry_count = 5;
    if (wifi_connect_state == true)
    {
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
        {
            if (wifi_connect_state == false)
            {
                break;
            }
            ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}

void get_time_update(void)
{
    time_t now;
    // struct tm timeinfo;
    // static bool second_flag = false;
    // now = mktime(&timeinfo);
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900)

    if (wifi_connect_state == true)
    {
        timeinfo_update = true;
        if (timeinfo.tm_year < (2023 - 1900))
        {
            ESP_LOGI("SNTP", "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time();
            // update 'now' variable with current time
            time(&now);
        }

        setenv("TZ", timezone, 1);
        tzset();

        time(&now);
        localtime_r(&now, &timeinfo);
        current_epoch = now;
        ESP_LOGI("EPOCH", "TIME: %lld", current_epoch);
    }
    else if (timeinfo_update == false)
    {
        // Set the manual time using settimeofday
        struct timeval tv;
        tv.tv_sec = manualTime;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);

        // settimeofday(&tv, NULL);
        // time_t now;
        time(&now);

        // ESP_LOGI("Eheat","Offline: %s", ctime(&now));
        manualTime++;
    }

    gb_hour = timeinfo.tm_hour;
    gb_minute = timeinfo.tm_min;
    gb_weekday = timeinfo.tm_wday;

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    for (int i = 0; i < 5; i++)
    {
        disp_time[i] = strftime_buf[11 + i];
    }
    disp_time[2] = ' '; // this is filled with : with bitmap information

    ESP_LOGI("SNTP", "The current date/time is: %s", strftime_buf);
    memset(strftime_buf, 0, sizeof(strftime_buf));
}

void timer_schedule(void)
{
    // start the timer counter once the user starts
    ESP_LOGI("MODE VALUES", "Current mode: %d\t\tPrevious mode:%d", operation_mode, prev_operation_mode);
    if (tim_timer_mode_started)
    {
        gptimer_get_raw_count(gptimer, &elapsed_secs);
        remaining_timer_seconds = (alarm_config.alarm_count / 1000000) - (elapsed_secs / 1000000);
        sprintf(timer_set_buffer, "%01lld %02lld", Hours(remaining_timer_seconds), Minutes(remaining_timer_seconds));
        ESP_LOGI("TIMER", "remaining time is %02lld:%02lld:%02lld", Hours(remaining_timer_seconds), Minutes(remaining_timer_seconds), Seconds(remaining_timer_seconds));
    }

    // Increase the timer by TIMER_CHANGE_INTERVAL once the user press the increase button
    if (inc_det)
    {
        if (tim_timer_mode_started)
        {
            ESP_LOGI("TIMER", "Timer already running. Stop the timer to change the values");
        }
        else
        {
            if ((alarm_config.alarm_count / 1000000) < TIMER_CHANGE_INTERVAL * TIMER_CHANGE_STEP)
            {
                alarm_config.alarm_count += TIMER_CHANGE_INTERVAL * 1000000;
                gb_params.heat_timer.t_value += TIMER_CHANGE_INTERVAL / 60;
                sprintf(timer_set_buffer, "%01lld %02lld", Hours(alarm_config.alarm_count / 1000000), Minutes(alarm_config.alarm_count / 1000000));
                ESP_LOGI("TIMER", "Increment TIMER detected. Timer value set to %02lld:%02lld:%02lld", Hours(alarm_config.alarm_count / 1000000), Minutes(alarm_config.alarm_count / 1000000), Seconds(alarm_config.alarm_count / 1000000));
            }
            else
            {
                ESP_LOGI("TIMER", "Max limit reached");
            }
        }
        inc_det = false;
    }

    // Decrease the timer by TIMER_CHANGE_INTERVAL (not below TIMER_CHANGE_INTERVAL) when user press the decrease TIMER
    if (dec_det)
    {
        if (tim_timer_mode_started)
        {
            ESP_LOGI("TIMER", "Timer already running. Stop the timer to change the values");
        }
        else
        {
            if ((alarm_config.alarm_count / 1000000) > TIMER_CHANGE_INTERVAL)
            {
                alarm_config.alarm_count -= TIMER_CHANGE_INTERVAL * 1000000;
                gb_params.heat_timer.t_value -= TIMER_CHANGE_INTERVAL / 60;
                sprintf(timer_set_buffer, "%01lld %02lld", Hours(alarm_config.alarm_count / 1000000), Minutes(alarm_config.alarm_count / 1000000));
                ESP_LOGI("TIMER", "Decrement TIMER detected. Timer value set to %02lld:%02lld:%02lld", Hours(alarm_config.alarm_count / 1000000), Minutes(alarm_config.alarm_count / 1000000), Seconds(alarm_config.alarm_count / 1000000));
            }
            else
            {
                ESP_LOGI("TIMER", "Timer value is already at minimum value.");
            }
        }
        dec_det = false;
    }

    // start the timer once the user has set the values
    if (start_det || start_det_cloud)
    {
        if (tim_timer_mode_started)
        {
            ESP_LOGI("TIMER", "Timer is already running.");
        }
        else
        {
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
            ESP_ERROR_CHECK(gptimer_enable(gptimer));
            ESP_ERROR_CHECK(gptimer_start(gptimer));
            ESP_LOGI("TIMER", "Timer initialised.");
            tim_timer_mode_started = true;
            if (operation_mode != TIMER_MODE)
            {
                prev_operation_mode = operation_mode;
            }
            operation_mode = TIMER_MODE;
            gb_params.op_mode = operation_mode;
            if (start_det)
                panelStateChanged_op_mode = true;
        }
        start_det = false;
        start_det_cloud = false;
    }
    // When the timer is stopped
    if (tim_timer_mode_stop)
    {
        tim_timer_mode_stop = false;
        elapsed_secs = 0;
        ESP_LOGI("TIMER", "Timer interrupted. Stopping the timer.");
    }

    if (timer_stop_after_complete)
    {
        elapsed_secs = 0;
        ESP_LOGI("TIMER", "Timer completed. Turning off the heater");
        timer_stop_after_complete = false;
    }
    if (timer_stopped_flag)
    {
        timer_stopped_flag = false;
        if (gb_params.hold == true)
        {
            operation_mode = AUTO_MODE;
            prev_operation_mode = AUTO_MODE;
        }
        else
        {
            operation_mode = prev_operation_mode;
            prev_operation_mode = TIMER_MODE;
        }
        gb_params.op_mode = operation_mode;
        panelStateChanged_op_mode = true;
        set_temperature(&gb_params);
        panelStateChanged_timer = true;
    }
}

// first time only
void update_prev_values()
{
    params_t params;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }

    int32_t nvs_values = 0;
    nvs_get_i32(my_handle, "disp_intens", &nvs_values);
    params.display.intensity = (uint8_t)nvs_values;
    LogInfo(("reading prev values disp_intens %d", params.display.intensity));

    nvs_get_i32(my_handle, "disp_state", &nvs_values);
    params.display.state = (uint8_t)nvs_values;
    LogInfo(("reading prev values disp_state %d", params.display.state));

    nvs_get_i32(my_handle, "disp_tout", &nvs_values);
    params.display.t_out = (uint8_t)nvs_values;
    LogInfo(("reading prev values disp_tout %d", params.display.t_out));

    nvs_get_i32(my_handle, "disp_int_type", &nvs_values);
    params.display.intens_type = (uint8_t)nvs_values;
    LogInfo(("reading prev values disp_int_type %d", params.display.t_out));

    // // storing the pilot light setting into the NVS
    nvs_get_i32(my_handle, "pl_intens", &nvs_values);
    params.pilot_light.intensity = (uint8_t)nvs_values;
    LogInfo(("reading prev values pl_intens %d", params.pilot_light.intensity));

    nvs_get_i32(my_handle, "pl_state", &nvs_values);
    params.pilot_light.state = (uint8_t)nvs_values;
    LogInfo(("reading prev values pl_state %d", params.pilot_light.state));

    // // storing the night light setting into
    nvs_get_i32(my_handle, "nl_state", &nvs_values);
    params.night_light.state = (uint8_t)nvs_values;
    LogInfo(("reading prev values nl_state %d", params.night_light.state));

    nvs_get_i32(my_handle, "nl_intens", &nvs_values);
    params.night_light.intensity = (uint8_t)nvs_values;
    LogInfo(("reading prev values nl_intens %d", params.night_light.intensity));

    nvs_get_i32(my_handle, "nl_red", &nvs_values);
    params.night_light.color.red = (uint8_t)nvs_values;
    LogInfo(("reading prev values nl_red %d", params.night_light.color.red));

    nvs_get_i32(my_handle, "nl_green", &nvs_values);
    params.night_light.color.green = (uint8_t)nvs_values;
    LogInfo(("reading prev values nl_green %d", params.night_light.color.green));

    nvs_get_i32(my_handle, "nl_blue", &nvs_values);
    params.night_light.color.blue = (uint8_t)nvs_values;
    LogInfo(("reading prev values nl_blue %d", params.night_light.color.blue));

    err = nvs_get_i32(my_handle, "s_temp", &nvs_values);
    params.heater.s_temp = (float)nvs_values;
    LogInfo(("reading prev values s_temp %f", params.heater.s_temp));

    params.heater.s_temp_fer = round((nvs_values * 9) / 5 + 32);
    LogInfo(("reading prev values s_temp_fer %f", params.heater.s_temp_fer));

    nvs_get_i32(my_handle, "heater_state", &nvs_values);
    params.heater.state = nvs_values;
    LogInfo(("reading prev values heater_state %d", params.heater.state));
    if (params.heater.state == true)
    {
        heater_on_flag = true;
    }
    else
    {
        heater_on_flag = false;
    }

    nvs_get_i32(my_handle, "t_unit", &nvs_values);
    params.t_unit = nvs_values;
    t_unit_flag = params.t_unit;
    LogInfo(("reading prev values t_unit %d", params.t_unit));

    nvs_get_i32(my_handle, "t_offset", &nvs_values);
    temperature_offset = nvs_values;
    LogInfo(("reading prev values t_offset %d", temperature_offset));

    nvs_get_i32(my_handle, "c_lock", &nvs_values);
    params.c_lock = nvs_values;
    LogInfo(("reading prev values c_lock %d", params.c_lock));

    nvs_get_i32(my_handle, "f_prot", &nvs_values);
    gb_freeze_protection_flag = nvs_values;
    LogInfo(("reading prev values f_prot %d", gb_freeze_protection_flag));

    // err = nvs_set_i32(my_handle, "cali_flag", calibration_flag);
    err = nvs_get_i32(my_handle, "cali_flag", &nvs_values);
    calibration_flag = nvs_values;
    LogInfo(("reading prev values calibration_flag %d", calibration_flag));
    // calibration_flag = false;

    // ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
    ESP_LOGI("Eheat", "Committing updates in NVS ... ");

    err = nvs_commit(my_handle);
    // ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
    // Close
    gb_params = params;
    gb_params.auto_heater.s_temp = gb_params.heater.s_temp;
    gb_params.auto_heater.state = gb_params.heater.state;
    gb_params.heat_timer.t_value = 30;
    gb_params.heat_timer.t_temp = 5;
    gb_params.hold = OFF;

    set_pilot_light(&gb_params);
    set_display(&gb_params);
    set_color(&gb_params);
    set_temperature(&gb_params);
    control_child_lock(&gb_params);

    nvs_close(my_handle);

    retrieve_string("timezone", timezone, sizeof(timezone));
    LogInfo(("reading prev values timezone %s", timezone));
    setenv("TZ", timezone, 1);
    tzset();
    get_schedule_data();
    // obtain_time();
}

void save_curr_values(params_t *params)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    // storing the display info into the nvs
    err = nvs_set_i32(my_handle, "disp_intens", params->display.intensity);
    err = nvs_set_i32(my_handle, "disp_state", params->display.state);
    err = nvs_set_i32(my_handle, "disp_tout", params->display.t_out);
    // storing the pilot light setting into the NVS
    err = nvs_set_i32(my_handle, "pl_intens", params->pilot_light.intensity);
    err = nvs_set_i32(my_handle, "pl_state", params->pilot_light.state);
    // storing the night light setting into
    err = nvs_set_i32(my_handle, "nl_state", params->night_light.state);
    err = nvs_set_i32(my_handle, "nl_intens", params->night_light.intensity);
    err = nvs_set_i32(my_handle, "nl_itype", params->night_light.intens_type);
    err = nvs_set_i32(my_handle, "nl_red", params->night_light.color.red);
    err = nvs_set_i32(my_handle, "nl_green", params->night_light.color.green);
    err = nvs_set_i32(my_handle, "nl_blue", params->night_light.color.blue);

    err = nvs_set_i32(my_handle, "s_temp", params->heater.s_temp);
    err = nvs_set_i32(my_handle, "s_temp_fer", params->heater.s_temp_fer);
    err = nvs_set_i32(my_handle, "heater_state", params->heater.state);
    err = nvs_set_i32(my_handle, "t_unit", params->t_unit);

    // err = nvs_set_i32(my_handle, "t_offset", temperature_offset);

    ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
    ESP_LOGI("Eheat", "Committing updates in NVS ... ");

    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    // Close
    ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
    // gb_params = *params;
}

esp_err_t get_mac_id(char *device_uid)
{
    uint8_t mac[6];
    esp_err_t ret;

    // fetch the MAC ID
    ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Failed to fetch MAC ID (error code %d)\n", ret);
        return ret;
    }

    // print the MAC ID
    sprintf(device_uid, "envi-%02x%02x%02x", mac[3], mac[4], mac[5]);
    // sprintf(mac_id, "envi-%02x%02x%02x", mac[2], mac[4], mac[5]);
    ESP_LOGI("Eheat", "MAC ADDRESS %s", device_uid);
    return ESP_OK;
}

void setting_env(void)
{
    nvs_handle_t my_handle;
    int32_t env_state = 0;
    esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("Eheat", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    err = nvs_get_i32(my_handle, "env_state", &env_state);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI("Eheat", "ENV State = %" PRIu32 "\n", env_state);
        switch (env_state)
        {
        case 0:
            sprintf(&envo, "%s", "dev");
            ESP_LOGI("Eheat", "Selected the Developement Envo %s", envo);
            break;
        case 1:
            sprintf(&envo, "%s", "qa");
            ESP_LOGI("Eheat", "Selected the QA Envo %s", envo);
            break;
        case 2:
            sprintf(&envo, "%s", "stage");
            ESP_LOGI("Eheat", "Selected the STAGE Envo %s", envo);
            break;
        case 3:
            sprintf(&envo, "%s", "prod");
            ESP_LOGI("Eheat", "Selected the PROD Envo %s", envo);
            break;

        default:
            break;
        }
        nvs_close(my_handle);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGW("Eheat", "The value is not initialized yet!\n");
        env_state = 2;
        err = nvs_get_i32(my_handle, "env_state", &env_state);
        sprintf(&envo, "%s", "stage");
        ESP_LOGI("Eheat", "Selected the STAGE Envo %s", envo);
        nvs_close(my_handle);
        break;
    default:
        ESP_LOGW("Eheat", "Error (%s) reading!\n", esp_err_to_name(err));
        nvs_close(my_handle);
    }
}

void control_auto_intensity()
{
    char buffer[10];
    int raw_ldr = 0;
    static uint8_t last_intensity = 0;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &raw_ldr);
    sprintf(buffer, "%d Lx", GET_AUTO_INTENS(raw_ldr));

    if (gb_params.display.intens_type == DISP_AUTO)
    {
        gb_params.display.intensity = (uint8_t)GET_AUTO_INTENS(raw_ldr);
        set_display(&gb_params);
    }
    if (gb_params.night_light.state == AUTO || gb_params.night_light.intens_type == true)
    {
        gb_params.night_light.intensity = (uint8_t)GET_AUTO_INTENS(raw_ldr);
        set_color(&gb_params);
    }
    if (gb_params.pilot_light.state == AUTO)
    {
        gb_params.pilot_light.intensity = (uint8_t)GET_AUTO_INTENS(raw_ldr);

        // adding condition to avoid updating the same values
        if (GET_AUTO_INTENS(raw_ldr) != last_intensity)
        {
            set_pilot_light(&gb_params);
        }
        last_intensity = (uint8_t)GET_AUTO_INTENS(raw_ldr);
    }
}

void callback_button_up(void)
{
    if (gpio_get_level(BUTTON_UP_PIN) == released)
    {
        if (display_timeout_flag == true)
        {
            display_timeout_flag = false;
            disp_prev_time = esp_timer_get_time() / 1000000;
        }
        else
        {
            flag_button_up = true;
            disp_prev_time = esp_timer_get_time() / 1000000;
        }
    }
}

void callback_button_down(void)
{
    if (gpio_get_level(BUTTON_DOWN_PIN) == released)
    {
        if (display_timeout_flag == true)
        {
            display_timeout_flag = false;
            disp_prev_time = esp_timer_get_time() / 1000000;
        }
        else
        {
            flag_button_down = true;
            disp_prev_time = esp_timer_get_time() / 1000000;
        }
    }
}

void callback_button_select(void)
{
    if (gpio_get_level(BUTTON_SELECT_PIN) == pressed)
    {
        // CORNER CASE: if a user is disabling the child lock using panel,
        // and he releases and press the SELECT button only, then no task should be performed
        // untill both buttons are not released once.
        if (flag_child_lock_timer_button_pressed && child_lock_disable_task_start)
        {
            flag_child_lock_select_button_pressed = true;
            return;
        }
        // check if timer button is already pressed for a combination press
        if (flag_timer_button_timer_start)
        {
            // Get elapsed time from SELECT button, check if not more than 0.5 Seconds
            // Discard child lock trigger if pressed time is more than 0.5 seconds
            int32_t timerbutton_timer_elapsed = 0;
            gptimer_get_raw_count(gptimer_timer_button_detect, &timerbutton_timer_elapsed);

            // if the timer button was pressed not before 0.5 seconds, then cancel that button press and start a new timer for combo press
            if (timerbutton_timer_elapsed < 500000)
            {
                // stop individual timer if all conditions are satisfied for child lock trigger
                timer_stop_lp_buttonpress(gptimer_timer_button_detect);
                ESP_ERROR_CHECK(gptimer_disable(gptimer_timer_button_detect));
                ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_combo_button_detect, 0));
            }
            alarm_config_combo_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_combo_button_detect, &alarm_config_combo_button_detect));
            ESP_ERROR_CHECK(gptimer_start(gptimer_combo_button_detect));
            flag_combo_button_timer_start = true;
        }
        else
        {
            // start the long press timer for SELECT button
            ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_select_button_detect, 0));
            alarm_config_select_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_select_button_detect, &alarm_config_select_button_detect));
            ESP_ERROR_CHECK(gptimer_enable(gptimer_select_button_detect));
            ESP_ERROR_CHECK(gptimer_start(gptimer_select_button_detect));
            flag_select_button_timer_start = true;
        }
    }
    if (gpio_get_level(BUTTON_SELECT_PIN) == released)
    {
        if (child_lock_disable_task_start)
        {
            if (flag_child_lock_timer_button_pressed)
            {
                flag_child_lock_select_button_pressed = false;
                return;
            }
            child_lock_disable_task_start = false;
        }
        else
        {
            // Check if child lock timer was started but not completed, stop if true
            if ((flag_lp_combo_button_complete == false) && (flag_combo_button_timer_start == true))
            {
                timer_stop_lp_buttonpress(gptimer_combo_button_detect);
            }

            if (flag_lp_selectbutton_task == false)
            {
                timer_stop_lp_buttonpress(gptimer_select_button_detect);
                ESP_ERROR_CHECK(gptimer_disable(gptimer_select_button_detect));
                if ((longpress_select_button_released == true) && (child_lock_flag == false))
                {
                    select_button_press_type = shortPress;
                }
            }
            else
            {
                flag_lp_selectbutton_task = false;
                longpress_select_button_released = true;
            }
        }
    }
}

void callback_button_timer(void)
{
    if (gpio_get_level(BUTTON_BACK_PIN) == pressed)
    {
        // CORNER CASE: if a user is disabling the child lock using panel,
        // and he releases and press the timer button only, then no task should be performed
        // untill both buttons are not released once.
        if (flag_child_lock_select_button_pressed && child_lock_disable_task_start)
        {
            flag_child_lock_timer_button_pressed = true;
            return;
        }

        // Check if SELECT/OK button is also pressed, testing for child lock
        if (flag_select_button_timer_start)
        {
            // Get elapsed time from SELECT button, check if not more than 0.5 Seconds
            // Discard child lock trigger if pressed time is more than 0.5 seconds
            int32_t selectbutton_timer_elapsed = 0;
            gptimer_get_raw_count(gptimer_select_button_detect, &selectbutton_timer_elapsed);

            // if the timer button was pressed not before 0.5 seconds, then cancel that button press and start a new timer for combo press
            if (selectbutton_timer_elapsed < 500000)
            {
                // stop individual timer if all conditions are satisfied for child lock trigger
                timer_stop_lp_buttonpress(gptimer_select_button_detect);
                ESP_ERROR_CHECK(gptimer_disable(gptimer_select_button_detect));
                ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_combo_button_detect, 0));
            }
            ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_combo_button_detect, 0));
            alarm_config_combo_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_combo_button_detect, &alarm_config_combo_button_detect));
            ESP_ERROR_CHECK(gptimer_start(gptimer_combo_button_detect));
            flag_combo_button_timer_start = true;
        }
        else
        {
            // start the long press timer for timer button
            ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer_timer_button_detect, 0));
            alarm_config_timer_button_detect.alarm_count = BUTTON_LONGPRESS_DURATION * 1000000;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_timer_button_detect, &alarm_config_timer_button_detect));
            ESP_ERROR_CHECK(gptimer_enable(gptimer_timer_button_detect));
            ESP_ERROR_CHECK(gptimer_start(gptimer_timer_button_detect));
            flag_timer_button_timer_start = true;
        }
    }
    if (gpio_get_level(BUTTON_BACK_PIN) == released)
    {
        if (child_lock_disable_task_start)
        {
            if (flag_child_lock_select_button_pressed)
            {
                flag_child_lock_timer_button_pressed = false;
                return;
            }
            child_lock_disable_task_start = false;
        }
        else
        {
            // Check if child lock timer was started but not completed, stop if true

            if ((flag_lp_combo_button_complete == false) && (flag_combo_button_timer_start == true))
            {
                timer_stop_lp_buttonpress(gptimer_combo_button_detect);
            }

            // If the button is released before a long duration, stop the LONGPRESS timer
            if (flag_lp_timerbutton_task == false)
            {
                timer_stop_lp_buttonpress(gptimer_timer_button_detect);
                ESP_ERROR_CHECK(gptimer_disable(gptimer_timer_button_detect));

                if ((longpress_timer_button_released == true) && (child_lock_flag == false))
                {
                    timer_button_press_type = shortPress;
                }
            }
            else
            {
                flag_lp_timerbutton_task = false;
                longpress_timer_button_released = true;
            }
        }
    }
}

void control_child_lock(params_t *params)
{
    if (params->c_lock == true)
    {
        gpio_intr_disable(BUTTON_UP_PIN);
        gpio_intr_disable(BUTTON_DOWN_PIN);
        child_lock_flag = true;
    }
    else if (params->c_lock == false)
    {
        gpio_intr_enable(BUTTON_UP_PIN);
        gpio_intr_enable(BUTTON_DOWN_PIN);
        if (child_lock_flag == true)
        {
            child_lock_disable_task_start = true;
            child_lock_flag = false;
        }
    }
}

void set_adjustment_factor()
{
    // @TODO this function will be called before cycling the heater state
    // This function is used to calculate the adjustment factor i.e. cHeat and cCool
    // These variables should be updated globally so that these can be used by other functions.
    // cHeat: An adjustment factor used to account for prior cycling when heating in seconds
    // cCool: An adjustment factor used to account for prior cycling when cooling in seconds
    if (heater_on_flag)
    {
        // Check if tMax is suitaby larger than tDelta to avoid ln(0)
        // Provide a larger arbitary value to cHeat if difference is zero.
        cHeat = 2147483646;
        if (tMax_c > tDelta)
        {
            cHeat = -tHeat * log((tMax_c - tDelta) / tMax_c);
        }
    }
    else
    {
        // Check if tDelta is not zero. If yes, provide a large arbitary value for cCool
        cCool = 2147483646;
        if (tDelta != 0)
        {
            cCool = -tCool * log(tDelta / tMax_c);
        }
    }
}

// offline mode
void setting_default_values()
{
    update_prev_values();
    gb_params.night_light.color.blue = 255;
    gb_params.night_light.color.red = 255;
    gb_params.night_light.color.green = 255;
    gb_params.night_light.intensity = 100;
    set_color(&gb_params);
    gb_params.heater.s_temp = 5;
    gb_params.heater.s_temp_fer = 40;
    gb_params.heater.state = true;
    set_temperature(&gb_params);
    retrieve_string("timezone", timezone, sizeof(timezone));
    LogInfo(("reading prev values timezone %s", timezone));
    setenv("TZ", timezone, 1);
    tzset();
}

void button_state()
{
    /* ------------------  start Button Acivities ------------------------*/
    if (timer_button_press_type == shortPress)
    {
        ESP_LOGI("DISPLAY TOGGLE", "Timer button pressed for short");
        if (display_timeout_flag == true)
        {
            ESP_LOGI("DISPLAY TOGGLE", "TIMER BUTTON pressed for short when timeoutflag is true");
            display_timeout_flag = false;
            disp_prev_time = esp_timer_get_time() / 1000000;
        }
        else
        {
            disp_prev_time = esp_timer_get_time() / 1000000;
            flag_button_back = true;
        }
    }
    if (current_screen == 1)
    {
        if ((gb_params.heater.state == ON) && (timer_button_press_type == longPress))
        {

            flag_lp_timer_button_complete = false;
            if ((operation_mode == AUTO_MODE) || (operation_mode == GEOFENCE_MODE) || (operation_mode == TIMER_MODE))
            {
                operation_mode = SCHEDULE_MODE;
                gb_params.op_mode = operation_mode;
                gb_schedule_flag = true;
                run_schedule();
            }
            else if (operation_mode == SCHEDULE_MODE)
            {
                operation_mode = AUTO_MODE;
            }
            gb_params.op_mode = operation_mode;
            gb_params.op_mode = operation_mode;
            panelStateChanged_op_mode = true;
            panelStateChanged_heater = true;
            set_temperature(&gb_params);
        }
    }

    if (select_button_press_type == shortPress)
    {
        ESP_LOGI("DISPLAY TOGGLE", "Select button Pressed for short");
        if (display_timeout_flag == false || gb_params.heater.state == false)
        {
            flag_button_select = true;
        }
        display_timeout_flag = false;
        disp_prev_time = esp_timer_get_time() / 1000000;
    }

    if (select_button_press_type == longPress)
    {
        flag_lp_selectbutton_task = true;
        flag_lp_select_button_complete = false;
        current_screen = 2;
        select_button_press_type = not_pressed;
        // ESP_LOGI("_-----------------_", "long press  btton state is: %d", select_button_press_type);
    }

    select_button_press_type = not_pressed; // select button
    timer_button_press_type = not_pressed;  // back button0

    /* ------------------  END Button Acivities ------------------------*/
}

void set_device_time(int hour, int minutes, int seconds)
{
    if (wifi_connect_state == false)
    {
        static struct tm timeinfo;
        currentTime = time(NULL);
        // timeinfo = *(localtime(&currentTime));
        localtime_r(&currentTime, &timeinfo);
        // Update the timeinfo with the provided hours, minutes, and seconds
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minutes;
        timeinfo.tm_sec = seconds;
        timeinfo.tm_mon = 5;
        // timeinfo.tm_wday = 1;
        timeinfo.tm_mday = 13;
        timeinfo.tm_year = 123;
        timeinfo.tm_isdst = day_light_saving_flag;
        // Convert the updated timeinfo back to a time_t value
        manualTime = mktime(&timeinfo);
    }
    else
    {
        get_time_update();
    }
}

void store_string(const char *key, const char *value)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error opening NVS handle!\n");
        return;
    }

    ret = nvs_set_str(nvs_handle, key, value);
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error setting value in NVS!\n");
    }

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

void retrieve_string(const char *key, char *buffer, size_t buffer_size)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error opening NVS handle!\n");
        return;
    }

    size_t required_size;
    ret = nvs_get_str(nvs_handle, key, buffer, &required_size);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI("Eheat", "Error retrieving value from NVS!\n");
    }

    nvs_close(nvs_handle);
}

// Store the schedule data in NVS
void storeScheduleData()
{
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error opening NVS handle: %d\n", err);
        return;
    }

    err = nvs_set_blob(nvsHandle, NVS_KEY, &gb_sch, sizeof(gb_sch));
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error storing schedule data in NVS: %d\n", err);
    }

    nvs_close(nvsHandle);
}

// Retrieve the schedule data from NVS
void retrieveScheduleData()
{
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error opening NVS handle: %d\n", err);
        return;
    }

    size_t requiredSize;
    err = nvs_get_blob(nvsHandle, NVS_KEY, NULL, &requiredSize);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI("Eheat", "Error retrieving schedule data from NVS: %d\n", err);
        nvs_close(nvsHandle);
        return;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI("Eheat", "Schedule data not found in NVS.\n");
        nvs_close(nvsHandle);
        return;
    }

    if (requiredSize != sizeof(gb_sch))
    {
        ESP_LOGI("Eheat", "Invalid schedule data size in NVS.\n");
        nvs_close(nvsHandle);
        return;
    }

    err = nvs_get_blob(nvsHandle, NVS_KEY, &gb_sch, &requiredSize);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error retrieving schedule data from NVS: %d\n", err);
    }

    nvs_close(nvsHandle);
}
void timer_stop_call(void)
{
    gptimer_stop(gptimer);
    ESP_ERROR_CHECK(gptimer_disable(gptimer));
    // resetting last timer count
    gptimer_set_raw_count(gptimer, 0);
    tim_timer_mode_started = false;
    timer_stopped_flag = true;
    // wheever timer stops display shows the last set time value.
    sprintf(timer_set_buffer, "%01lld %02lld", Hours(alarm_config.alarm_count / 1000000), Minutes(alarm_config.alarm_count / 1000000));
    gb_params.heat_timer.state = OFF;
}

void timer_stop_lp_buttonpress(gptimer_handle_t timer)
{
    gptimer_stop(timer);
    gptimer_set_raw_count(timer, 0);
    if (timer == gptimer_select_button_detect)
    {
        flag_select_button_timer_start = false;
    }
    if (timer == gptimer_timer_button_detect)
    {
        flag_timer_button_timer_start = false;
    }
}

void timer_callback(void)
{
    timer_stop_call();
    timer_stop_after_complete = true;
    tim_timer_mode_stop = true;
}

void cb_lp_select_button(void)
{
    timer_stop_lp_buttonpress(gptimer_select_button_detect);
    ESP_ERROR_CHECK(gptimer_disable(gptimer_select_button_detect));
    if (child_lock_flag == false)
    {
        select_button_press_type = longPress;
        flag_lp_select_button_complete = true;
        longpress_select_button_released = false;
    }
}

void cb_lp_timer_button(void)
{
    timer_stop_lp_buttonpress(gptimer_timer_button_detect);
    ESP_ERROR_CHECK(gptimer_disable(gptimer_timer_button_detect));
    if (child_lock_flag == false)
    {
        timer_button_press_type = longPress;
        flag_lp_timer_button_complete = true;
        longpress_timer_button_released = false;
        flag_lp_timerbutton_task = true;
    }
}

void cb_lp_combo_button(void)
{
    timer_stop_lp_buttonpress(gptimer_timer_button_detect);
    ESP_ERROR_CHECK(gptimer_disable(gptimer_timer_button_detect));
    timer_stop_lp_buttonpress(gptimer_select_button_detect);
    ESP_ERROR_CHECK(gptimer_disable(gptimer_select_button_detect));
    flag_lp_combo_button_complete = true;
}

void get_schedule_data()
{
    retrieveScheduleData();

    for (int i = 0; i < WEEK_DAYS; i++)
    {
        ESP_LOGI("SCHEDULE", "Week Day %d:", i);
        for (int j = 0; j < gb_sch[i].num_events; j++)
        {
            ESP_LOGI("SCHEDULE", "Event %d: at %02d:%02d - Tempe: %.2f",
                     j + 1,
                     gb_sch[i].hour[j],
                     gb_sch[i].minute[j],
                     gb_sch[i].temperature[j]);
        }
    }
}

void run_schedule()
{
    if (gb_params.hold == false)
    {
        if ((gb_schedule_flag == true) || (operation_mode == SCHEDULE_MODE))
        {
            for (int j = 0; j < gb_sch[gb_weekday].num_events; j++)
            {
                ESP_LOGI("SCHEDULE", "Event %d: at %02d:%02d - Tempe: %.2f",
                         j + 1,
                         gb_sch[gb_weekday].hour[j],
                         gb_sch[gb_weekday].minute[j],
                         gb_sch[gb_weekday].temperature[j]);

                if ((gb_sch[gb_weekday].hour[j] == gb_hour) && (gb_sch[gb_weekday].minute[j] <= gb_minute))
                {
                    ESP_LOGI("SCHEDULE", "runnning schedule Week Day %d with Temp %f", gb_weekday, gb_sch[gb_weekday].temperature[j]);
                    // copy schedule data from here.
                    gb_params.schedule_heater.s_temp = gb_sch[gb_weekday].temperature[j];
                    prev_operation_mode = operation_mode;
                    operation_mode = SCHEDULE_MODE;
                    set_temperature(&gb_params);
                }
            }
            gb_schedule_flag = false;
        }
    }
}

void addTime(int *hour, int *minute, int add_hour, int add_minute)
{
    *hour += add_hour;
    *minute += add_minute;

    if (*minute >= 60)
    {
        *hour += *minute / 60;
        *minute %= 60;
    }
    else if (*minute < 0)
    {
        *hour -= 1;
        *minute += 60;
    }
}

int getTimezoneOffset(const char *timezone_curr)
{
    int sign = 1;
    int hours = 0;
    int minutes = 0;

    // Skip "UTC" prefix
    timezone_curr += 3;

    if (*timezone_curr == '+')
    {
        sign = -1;
        timezone_curr++;
    }
    else if (*timezone_curr == '-')
    {
        sign = 1;
        timezone_curr++;
    }

    sscanf(timezone_curr, "%d:%d", &hours, &minutes);

    return sign * (hours * 60 + minutes);
}

esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        // ESP_LOGI("Eheat","%.*s", evt->data_len, (char *)evt->data);
        net_connection_flag = true;
        break;
    case HTTP_EVENT_ERROR:
        // ESP_LOGI("Eheat","%.*s", evt->data_len, (char *)evt->data);
        net_connection_flag = false;
        break;
    default:
        break;
    }
    return ESP_OK;
}

bool ping_google()
{
    esp_http_client_config_t config = {
        .url = "http://www.google.com",
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        // ESP_LOGI("Eheat","HTTP GET request completed successfully\n");
        ESP_LOGI("Eheat", "Ping successful!");
    }
    else
    {
        // ESP_LOGI("Eheat","HTTP GET request failed\n");
        ESP_LOGW("Eheat", "Ping Failed");
    }

    esp_http_client_cleanup(client);
    return 0;
}

void check_freeze_protection(double Tc, double Tf)
{
    // do not check for freeze protection if:
    // 1. Freeze protection is off from the app
    // 2. Heater is turned on by the user instead of auto on due to freeze protection
    previous_freeze_protect_state = heater_value;
    if ((gb_freeze_protection_flag == false) || ((gb_params.heater.state == ON) && (heat_on_freeze_protect == false)))
    {
        return;
    }

    if (t_unit_flag == true)
    {
        if ((Tc < FREEZE_PROTECTION_POINT_CEL) && (heater_value != 1))
        {
            // Turn on the heater
            heater_on_flag = ON;
            heater_value = 1;
        }
        if ((Tc > FREEZE_PROTECTION_POINT_CEL + 1) && (heater_value != 2) && heat_on_freeze_protect)
        {
            // turn off the heater
            heater_on_flag = OFF;
            heater_value = 2;
        }
    }
    if (t_unit_flag == false)
    {
        if ((Tf < FREEZE_PROTECTION_POINT_FAH) && (heater_value != 3))
        {
            // Turn on the heater
            heater_on_flag = ON;
            heater_value = 3;
        }
        if ((Tf > FREEZE_PROTECTION_POINT_FAH + FREEZE_OFF_THRESHOLD) && (heater_value != 4) && heat_on_freeze_protect)
        {
            // turn off the heater
            heater_on_flag = OFF;
            heater_value = 4;
        }
    }
    if (previous_freeze_protect_state != heater_value)
    {
        // update the backend about heater's state change
        // Op mode will also change to AUTO mode
        panelStateChanged_heater = true;
        // panelStateChanged_op_mode = true;
        gb_params.heater.state = heater_on_flag;
        gb_params.auto_heater.state = heater_on_flag;
        gb_params.schedule_heater.state = heater_on_flag;
        gb_params.geofence_heater.state = heater_on_flag;
        set_temperature(&gb_params);
        gb_params.heater.state = heater_on_flag;
        heat_on_freeze_protect = heater_on_flag;
        gpio_set_level(GPIO_HEATER_PANEL, heater_on_flag);
    }
}

float verify_temperature_celsius(float value)
{
    float final_value = value;

    if (value > 32.23)
    {
        final_value = (float)32.23;
    }
    if (value < 5)
    {
        if (t_unit_flag == true)
            final_value = 5;
        else
            final_value = 4.5;
    }
    return final_value;
}
