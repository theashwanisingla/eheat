#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"
#include "eheat.h"
#include "icons.h"

u8g2_t u8g2;
uint8_t disp_intensity = 200;
char disp_time[9];
float disp_temperature = 40;
int ambient_temperature = 0;
double report_ambient_temperature = 0;
uint8_t temp_value = 0;
char temperature_buf[15];
char temperature_buf_disp[15];

bool wifi_change_state = false;
volatile bool tim_timer_mode_started;
volatile uint32_t num;
volatile uint32_t num1;
int prev_time = 0;
bool display_timeout_flag = false;
bool display_off_flag = false;

int progress = 0;
int refresh_temperature = (ROOM_TEMP_REFRESH_SEC*10) -10;
params_t gb_params;
static const char *TAG = "Display";
static const unsigned int I2C_TIMEOUT_MS = 1000;

static spi_device_handle_t handle_spi;	// SPI handle.
static i2c_cmd_handle_t handle_i2c;		// I2C handle.
static u8g2_esp32_hal_t u8g2_esp32_hal; // HAL state data.

int button_up_clicked = 0;	   // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0; // same as above
int button_down_clicked = 0;   // same as above
int button_back_clicked = 0;   // same as above

bool flag_button_up = false;
bool flag_button_down = false;
bool flag_button_select = false;
bool flag_button_back = false;

int item_selected = 0; // which item in the menu is selected
int item_sel_previous; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next;	   // next item - used in the menu screen to draw next item after the selected one

bool timer_show_icon = false;

extern volatile bool inc_det;
extern volatile bool dec_det;
extern volatile bool start_det;
extern volatile bool timer_stop;
extern volatile bool timer_stop_after_complete;
uint8_t day_light_saving_flag = false;
char timer_set_buffer[6] = "0 30";
char timezone[10] = "0";
int calibration_flag = false;
// main
int current_screen = 1; // 0 = menu, 1 = screenshot, 2 = qr

typedef enum screens
{
	SET_TIMER_SCREEN = 0,
	HOME_SCREEN = 1,
	MAIN_MENU_SCREEN = 2,
	SUB_MENU = 3,
	SELECTED_SUB_MENU = 4,
} screens_t;

char menu_items[NUM_ITEMS][MAX_ITEM_LENGTH] = { // array with item names
	{"Display"},
	{"Night Light"},
	{"Timer"},
	{"Network"},
	{"Freeze Protect"},
	{"Child Lock"},
	{"Device Settings"},
	{"Power Light"},
	{"Set Schedule"},
	{"Set Time & Day"}};

// Screen Option
int item_selected_screen = 0; // which item in the menu is selected
int item_sel_previous_screen; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_sreen;	  // next item - used in the menu screen to draw next item after the selected one

#define DISP_SETTING_ITEMS 3
char disp_setting[DISP_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	// array for screen options
	{"Auto Brightness"},
	{"Change Brightness"},
	{"Timeout"}};

// Pilot light Option
int item_selected_pilot = 0; // which item in the menu is selected
int item_sel_previous_pilot; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_pilot;	 // next item - used in the menu screen to draw next item after the selected one
#define PILOT_LIGHT_SETTING_ITEMS 3
char pilot_light_options[PILOT_LIGHT_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	// array for screen options
	{"Always On"},
	{"Auto Brightness"},
	{"Change Brightness"}};

// selected light Option
int item_selected_night = 0; // which item in the menu is selected
int item_sel_previous_night; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_night;	 // next item - used in the menu screen to draw next item after the selected one
#define NIGHT_LIGHT_SETTING_ITEMS 3
char night_light_options[NIGHT_LIGHT_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"ON/OFF"}, // array for screen options
	{"Change Brightness"},
	{"Auto"},
};

// selected Timer Option
int item_selected_timer = 0; // which item in the menu is selected
int item_sel_previous_timer; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_timer;	 // next item - used in the menu screen to draw next item after the selected one
#define TIMER_SETTING_ITEMS 3
char timer_options[TIMER_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Timer On/Off"}, // array for screen options
	{"Set Duration"},
	{"Set Temperature"},
};

// selected enable disable Option
int item_selected_enable = 0; // which item in the menu is selected
int item_sel_previous_enable; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_enable;	  // next item - used in the menu screen to draw next item after the selected one
#define ENABLE_SETTING_ITEMS 2
char enable_disable_options[ENABLE_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Enable"}, // array for screen options
	{"Disable"},
};

// selected enable disable Option
int item_selected_unit = 0; // which item in the menu is selected
int item_sel_previous_unit; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_unit;		// next item - used in the menu screen to draw next item after the selected one
// #define ENABLE_SETTING_ITEMS 2
char temperature_unit_options[ENABLE_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Celcius - 'C"}, // array for screen options
	{"Ferhanite - 'F"},
};

// selected Network Option
int item_selected_network = 0; // which item in the menu is selected
int item_sel_previous_network; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_network;	   // next item - used in the menu screen to draw next item after the selected one
#define NETWORK_SETTING_ITEMS 3
char network_options[NETWORK_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Change Network"},
	{"Device IP Address"}, // array for screen options
	{"Wi-Fi Signal"}};

// device settings
int item_selected_device_setting = 0; // which item in the menu is selected
int item_sel_previous_device_setting; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_device_setting;	  // next item - used in the menu screen to draw next item after the selected one
#define DEVICE_SETTING_ITEMS 5
char device_setting_options[DEVICE_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Firmware Version"},
	{"Factory Reset"}, // array for screen options
	{"Device Calibration"},
	{"Temperature Unit"},
	{"About"}};

// set time and day settings
int item_selected_time_day = 0; // which item in the menu is selected
int item_sel_previous_time_day; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_time_day;		// next item - used in the menu screen to draw next item after the selected one
#define TIME_DAY_ITEMS 4
char time_day_options[TIME_DAY_ITEMS][MAX_ITEM_LENGTH] = {
	{"Set Time"},
	{"Set Present Day"}, // array for screen options
	{"Set Daylight Saving"},
	{"Set Timezone"},
	//{"About"}
};

#define ON_OFF_SETTING_ITEMS 2
char on_off_disable_options[ON_OFF_SETTING_ITEMS][MAX_ITEM_LENGTH] = {
	{"Turn On Schedule"},  // array for screen options
	{"Turn Off Schedule"}, // array for screen options
};

// set time and day settings
int item_selected_schedule = 0; // which item in the menu is selected
int item_sel_previous_schedule; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next_schedule;		// next item - used in the menu screen to draw next item after the selected one
#define SCHEDULE_MODE_ITEMS 3
char schedule_options[SCHEDULE_MODE_ITEMS][MAX_ITEM_LENGTH] = {
	{"On/Off Schedule"},
	{"Set 5-1-1 Schedule"},
	// {"Set W-L-R-S"}, // Mode for the Wake-Leave-return-Sleep
	// {"Select Mode"},
	{"Clear Schedule"},
};

#define EVENT_TYPES 4
char event_options[EVENT_TYPES][MAX_ITEM_LENGTH] = {
	{"Wake"},
	{"Leave"},
	{"Return"},
	{"Sleep"},
};

#define EVENT_DAY_TYPE 3
char event_day_options[EVENT_DAY_TYPE][MAX_ITEM_LENGTH] = {
	{"Mo Tu We Th Fr"},
	{"Sat"},
	{"Sun"}};

#define WEEK_DAYS_ITEMS 7
char week_days[WEEK_DAYS_ITEMS][MAX_ITEM_LENGTH] = {
	{"MON"},
	{"TUE"},
	{"WED"},
	{"THU"},
	{"FRI"},
	{"SAT"},
	{"SUN"}};

#define ESP_ERROR_CHECK(x)                         \
	do                                             \
	{                                              \
		esp_err_t rc = (x);                        \
		if (rc != ESP_OK)                          \
		{                                          \
			ESP_LOGE("err", "esp_err_t = %d", rc); \
			assert(0 && #x);                       \
		}                                          \
	} while (0);

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param)
{
	u8g2_esp32_hal = u8g2_esp32_hal_param;
} // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);
	switch (msg)
	{
	case U8X8_MSG_BYTE_SET_DC:
		if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.dc, arg_int);
		}
		break;

	case U8X8_MSG_BYTE_INIT:
	{
		if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED ||
			u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED ||
			u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED)
		{
			break;
		}

		spi_bus_config_t bus_config;
		bus_config.sclk_io_num = u8g2_esp32_hal.clk;  // CLK
		bus_config.mosi_io_num = u8g2_esp32_hal.mosi; // MOSI
		bus_config.miso_io_num = -1;				  // MISO
		bus_config.quadwp_io_num = -1;				  // Not used
		bus_config.quadhd_io_num = -1;				  // Not used
		// ESP_LOGI(TAG, "... Initializing bus.");
		ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));

		spi_device_interface_config_t dev_config;
		dev_config.address_bits = 0;
		dev_config.command_bits = 0;
		dev_config.dummy_bits = 0;
		dev_config.mode = 0;
		dev_config.duty_cycle_pos = 0;
		dev_config.cs_ena_posttrans = 0;
		dev_config.cs_ena_pretrans = 0;
		dev_config.clock_speed_hz = 500000;
		dev_config.spics_io_num = u8g2_esp32_hal.cs;
		dev_config.flags = 0;
		dev_config.queue_size = 200;
		dev_config.pre_cb = NULL;
		dev_config.post_cb = NULL;
		// ESP_LOGI(TAG, "... Adding device bus.");
		ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &handle_spi));

		break;
	}

	case U8X8_MSG_BYTE_SEND:
	{
		spi_transaction_t trans_desc;
		trans_desc.addr = 0;
		trans_desc.cmd = 0;
		trans_desc.flags = 0;
		trans_desc.length = 8 * arg_int; // Number of bits NOT number of bytes.
		trans_desc.rxlength = 0;
		trans_desc.tx_buffer = arg_ptr;
		trans_desc.rx_buffer = NULL;

		// ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
		ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
		break;
	}
	}
	return 0;
} // u8g2_esp32_spi_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle I2C communications.
 */
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

	switch (msg)
	{
	case U8X8_MSG_BYTE_SET_DC:
	{
		if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.dc, arg_int);
		}
		break;
	}

	case U8X8_MSG_BYTE_INIT:
	{
		if (u8g2_esp32_hal.sda == U8G2_ESP32_HAL_UNDEFINED ||
			u8g2_esp32_hal.scl == U8G2_ESP32_HAL_UNDEFINED)
		{
			break;
		}

		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		ESP_LOGI(TAG, "sda_io_num %d", u8g2_esp32_hal.sda);
		conf.sda_io_num = u8g2_esp32_hal.sda;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		ESP_LOGI(TAG, "scl_io_num %d", u8g2_esp32_hal.scl);
		conf.scl_io_num = u8g2_esp32_hal.scl;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		ESP_LOGI(TAG, "clk_speed %d", I2C_MASTER_FREQ_HZ);
		conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
		conf.clk_flags = 0;
		ESP_LOGI(TAG, "i2c_param_config %d", conf.mode);
		ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
		ESP_LOGI(TAG, "i2c_driver_install %d", I2C_MASTER_NUM);
		ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
		break;
	}

	case U8X8_MSG_BYTE_SEND:
	{
		uint8_t *data_ptr = (uint8_t *)arg_ptr;
		ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);

		while (arg_int > 0)
		{
			ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, *data_ptr, ACK_CHECK_EN));
			data_ptr++;
			arg_int--;
		}
		break;
	}

	case U8X8_MSG_BYTE_START_TRANSFER:
	{
		uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
		handle_i2c = i2c_cmd_link_create();
		ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address >> 1);
		ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
		ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
		break;
	}

	case U8X8_MSG_BYTE_END_TRANSFER:
	{
		ESP_LOGD(TAG, "End I2C transfer.");
		ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
		// ESP_ERROR_CHECK(
		i2c_master_cmd_begin(I2C_MASTER_NUM, handle_i2c, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(handle_i2c);
		break;
	}
	default:
		break;
	}
	return 0;
} // u8g2_esp32_i2c_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

	switch (msg)
	{
		// Initialize the GPIO and DELAY HAL functions.  If the pins for DC and RESET have been
		// specified then we define those pins as GPIO outputs.
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
	{
		uint64_t bitmask = 0;
		if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED)
		{
			bitmask = bitmask | (1ull << u8g2_esp32_hal.dc);
		}
		if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED)
		{
			bitmask = bitmask | (1ull << u8g2_esp32_hal.reset);
		}
		if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED)
		{
			bitmask = bitmask | (1ull << u8g2_esp32_hal.cs);
		}

		if (bitmask == 0)
		{
			break;
		}
		gpio_config_t gpioConfig;
		gpioConfig.pin_bit_mask = bitmask;
		gpioConfig.mode = GPIO_MODE_OUTPUT;
		gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
		gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
		gpioConfig.intr_type = GPIO_INTR_DISABLE;
		gpio_config(&gpioConfig);
		break;
	}

		// Set the GPIO reset pin to the value passed in through arg_int.
	case U8X8_MSG_GPIO_RESET:
		if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.reset, arg_int);
		}
		break;
		// Set the GPIO client select pin to the value passed in through arg_int.
	case U8X8_MSG_GPIO_CS:
		if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.cs, arg_int);
		}
		break;
		// Set the Software I²C pin to the value passed in through arg_int.
	case U8X8_MSG_GPIO_I2C_CLOCK:
		if (u8g2_esp32_hal.scl != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.scl, arg_int);
			//				ESP_LOGI("DISPLAY","%c",(arg_int==1?'C':'c'));
		}
		break;
		// Set the Software I²C pin to the value passed in through arg_int.
	case U8X8_MSG_GPIO_I2C_DATA:
		if (u8g2_esp32_hal.sda != U8G2_ESP32_HAL_UNDEFINED)
		{
			gpio_set_level(u8g2_esp32_hal.sda, arg_int);
			//				ESP_LOGI("DISPLAY","%c",(arg_int==1?'D':'d'));
		}
		break;

		// Delay for the number of milliseconds passed in through arg_int.
	case U8X8_MSG_DELAY_MILLI:
		vTaskDelay(arg_int / portTICK_PERIOD_MS);
		break;
	}
	return 0;
} // u8g2_esp32_gpio_and_delay_cb

void display_task(void *args)
{
	esp_err_t err = initialise_display();
	if (err != ESP_OK)
	{
		ESP_LOGI("Eheat", "Error Display Init\n", esp_err_to_name(err));
		return;
	}

	while (1)
	{
		if (display_timeout_flag == false)
		{
			if (calibration_flag == false)
			{
				gb_params.c_lock = false;
				panelStateChanged_child_lock = true;
				control_child_lock(&gb_params);
				item_selected_device_setting = 2;
				item_selected = 6;
				current_screen = 4;
			}

			u8g2_SetPowerSave(&u8g2, 0); // wake up display
			u8g2_SetContrast(&u8g2, disp_intensity);
			// disp_prev_time = esp_timer_get_time() / 1000000;
			if (current_screen != SET_TIMER_SCREEN && current_screen != 4)
			// if (current_screen != 0)
			{
				// gpio_get_level Selection
				if ((flag_button_select == true) && (button_select_clicked == 0))
				{ // select button clicked, jump between screens
					flag_button_select = false;
					button_select_clicked = 1; // set button to clicked to only perform the action once

					if ((current_screen < 4) && (current_screen != 1))
					{
						current_screen = current_screen + 1;
					}

					// Toggling the heater state with SELECT button
					if (current_screen == 1)
					{
						if (gb_params.heater.state == true)
						{
							if (tim_timer_mode_started)
							{
								gb_params.heat_timer.state = false;
								operation_mode = prev_operation_mode;
								operation_mode = AUTO_MODE;
								timer_stop_call();
							}
							gb_params.heater.state = OFF;
							gb_params.auto_heater.state = OFF;
							gb_params.geofence_heater.state = OFF;
							gb_params.schedule_heater.state = OFF;
							heater_on_flag = OFF;
							heater_value = 0;
						}
						else
						{
							gb_params.heater.state = ON;
							gb_params.auto_heater.state = ON;
							gb_params.geofence_heater.state = ON;
							gb_params.schedule_heater.state = ON;
							heater_on_flag = ON;
							heater_value = 0;
							heat_on_freeze_protect = false;
						}

						heater_state_changed = true;

						set_temperature(&gb_params);
						panelStateChanged_heater = true;

						ESP_LOGI("Button State", "button shortpress %d", gb_params.heater.state);
					}
				}

				if ((flag_button_select == false) && (button_select_clicked == 1))
				{ // unclick
					button_select_clicked = 0;
				}

				vTaskDelay(30 / portTICK_PERIOD_MS);

				/****************************************************************************/
				// get back button level
				if ((flag_button_back == true) && (button_back_clicked == 0))
				{ // select button clicked, jump between screens
					flag_button_back = false;
					button_back_clicked = 1; // set button to clicked to only perform the action once
					// current_screen--;
					if (current_screen > 0)
					{
						current_screen = current_screen - 1;
					}
				}
				if ((flag_button_back == false) && (button_back_clicked == 1))
				{ // unclick
					button_back_clicked = 0;
				}
			}

			u8g2_FirstPage(&u8g2); // required for page drawing mode for u8g library
			do
			{
				if (current_screen == -1)
				{
					show_cali();
				}
				if (current_screen == 0)
				{
					show_timer();
					update_timer();
				}
				if (current_screen == 1)
				{
					show_homescreen();
					// vTaskDelay(10 / portTICK_PERIOD_MS);
				}
				/*------------------------------*/
				if (current_screen == 2)
				{
					update_options(&item_selected, &item_sel_previous, &item_sel_next, NUM_ITEMS);
					show_menu();
					// vTaskDelay(10 / portTICK_PERIOD_MS);
				}
				if (current_screen == 3)
				{
					switch (item_selected)
					{
					case 0: /* SCREEN SETTING SCROLL AND UPDATE*/
						update_options(&item_selected_screen, &item_sel_previous_screen, &item_sel_next_sreen, DISP_SETTING_ITEMS);
						update_option_screen(disp_setting, &item_selected_screen, &item_sel_previous_screen, &item_sel_next_sreen, DISP_SETTING_ITEMS);
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 1: /* NIGHT LIGHT SETTING SCROLL AND UPDATE*/
						update_options(&item_selected_night, &item_sel_previous_night, &item_sel_next_night, NIGHT_LIGHT_SETTING_ITEMS);
						update_option_screen(night_light_options, &item_selected_night, &item_sel_previous_night, &item_sel_next_night, NIGHT_LIGHT_SETTING_ITEMS);
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 2: /* TIMER SETTING SCROLL AND UPDATE*/
						update_options(&item_selected_timer, &item_sel_previous_timer, &item_sel_next_timer, TIMER_SETTING_ITEMS);
						update_option_screen(timer_options, &item_selected_timer, &item_sel_previous_timer, &item_sel_next_timer, TIMER_SETTING_ITEMS);
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 3: /* NETWORK SETTING SCROLL AND UPDATE*/
						update_options(&item_selected_network, &item_sel_previous_network, &item_sel_next_network, NETWORK_SETTING_ITEMS);
						update_option_screen(network_options, &item_selected_network, &item_sel_previous_network, &item_sel_next_network, NETWORK_SETTING_ITEMS);
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 4: /* FREEZE PROTECT SETTING SCROLL AND UPDATE*/
						current_screen++;
						break;
					case 5: /* CHILD PROTECT SETTING SCROLL AND UPDATE*/
						current_screen++;
						break;
					case 6: /* DEVICE RESET SETTING SCROLL AND UPDATE*/
						update_options(&item_selected_device_setting, &item_sel_previous_device_setting, &item_sel_next_device_setting, DEVICE_SETTING_ITEMS);
						update_option_screen(device_setting_options, &item_selected_device_setting, &item_sel_previous_device_setting, &item_sel_next_device_setting, DEVICE_SETTING_ITEMS);
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 7: /* PILOT LIGHT SETTIING SCROLL AND UPDATE*/
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						update_options(&item_selected_pilot, &item_sel_previous_pilot, &item_sel_next_pilot, PILOT_LIGHT_SETTING_ITEMS);
						update_option_screen(pilot_light_options, &item_selected_pilot, &item_sel_previous_pilot, &item_sel_next_pilot, PILOT_LIGHT_SETTING_ITEMS);
						break;
					case 8: /* Set the Schedule Mode*/
						update_options(&item_selected_schedule, &item_sel_previous_schedule, &item_sel_next_schedule, SCHEDULE_MODE_ITEMS);
						update_option_screen(schedule_options, &item_selected_schedule, &item_sel_previous_schedule, &item_sel_next_schedule, SCHEDULE_MODE_ITEMS);
						break;
					case 9: /* Set time and date*/
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						update_options(&item_selected_time_day, &item_sel_previous_time_day, &item_sel_next_time_day, TIME_DAY_ITEMS);
						update_option_screen(time_day_options, &item_selected_time_day, &item_sel_previous_time_day, &item_sel_next_time_day, TIME_DAY_ITEMS);
						break;
					default:
						break;
					}
					// ESP_LOGI("DISPLAY", "INSIDE screen 1");
				}
				if (current_screen == 4)
				{
					u8g2_SetPowerSave(&u8g2, 0); // wake up display
					u8g2_SetContrast(&u8g2, disp_intensity);

					switch (item_selected)
					{
						u8g2_SetPowerSave(&u8g2, 0); // wake up display
						u8g2_SetContrast(&u8g2, disp_intensity);
					case 0:
						if (item_selected_screen == 0) // auto brightness
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							update_enable_option(&gb_params.display.intens_type, &item_selected_enable);
							// if the intensity type is 0 then the brightness will variey.
							// ESP_LOGI("DISPLAY", "************** Selected the option %d", gb_params.display.intens_type);
							control_auto_intensity();
						}
						if (item_selected_screen == 1)
						{
							// update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							update_brightness(&gb_params.display.intensity);
							set_display(&gb_params);
							u8g2_SetContrast(&u8g2, disp_intensity);
							show_brightness();
							// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected_screen);
						}
						if (item_selected_screen == 2)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							ESP_LOGI("DISPLAY", "------------ values %d", gb_params.display.state);
							update_enable_option(&gb_params.display.state, &item_selected_enable);
							// vTaskDelay(10 / portTICK_PERIOD_MS);
						}
						break;
					case 1:							  // Night light control
						if (item_selected_night == 0) // on off
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							update_enable_option(&gb_params.night_light.state, &item_selected_enable);
							set_color(&gb_params);
						}
						if (item_selected_night == 1) // change intennsity
						{
							update_brightness(&gb_params.night_light.intensity);
							set_color(&gb_params);
							show_brightness();
						}
						if (item_selected_night == 2) // auto
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							update_enable_option(&gb_params.night_light.intens_type, &item_selected_enable);
							set_color(&gb_params);
						}
						break;
					case 2: // timer setting
						if (item_selected_timer == 0)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							// update_enable_option(&start_det, &item_selected_enable);

							if (gb_params.hold)
							{
								u8g2_SetFont(&u8g2, Dejavu);
								u8g2_DrawStr(&u8g2, 5, 11, "Please disable hold");
							}

							if ((flag_button_select == true) && (button_select_clicked == 0))
							{
								flag_button_select = false;
								button_select_clicked = 1;
								if ((item_selected_enable == 0) && (gb_params.hold == false))
								{
									if (tim_timer_mode_started == false)
									{
										start_det = true;
										timer_schedule();
										gb_params.heat_timer.state = true;
										panelStateChanged_timer = true;
										gb_params.heater.state = true;
										set_temperature(&gb_params);
									}

									current_screen = current_screen - 1;
								}

								if (item_selected_enable == 1)
								{
									timer_stop_call();
									current_screen = current_screen - 1;
								}
							}

							if ((flag_button_select == false) && (button_select_clicked == 1))
							{ // unclick
								button_select_clicked = 0;
							}
							if ((flag_button_back == true) && (button_back_clicked == 0))
							{
								flag_button_back = false;
								button_back_clicked = 1;
								current_screen--;
							}
							if ((flag_button_back == false) && (button_back_clicked == 1))
							{ // unclick
								button_back_clicked = 0;
							}
						}
						if (item_selected_timer == 1)
						{
							show_timer();
							update_timer();
						}
						if (item_selected_timer == 2)
						{
							show_temperature();
						}
						break;
					case 3: /* NETWORK SETTING SCROLL AND UPDATE*/
						if (item_selected_network == 0)
						{
							show_wifi_change();
						}
						if (item_selected_network == 1)
						{
							show_ip_address();
						}
						if (item_selected_network == 2)
						{
							show_wifi_signal();
						}
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 4: // freeze protection
						update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
						show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
						if ((flag_button_select == true) && (button_select_clicked == 0))
						{
							flag_button_select = false;
							button_select_clicked = 1;
							if (item_selected_enable == 0)
							{
								gb_params.f_protection = 1;
								gb_freeze_protection_flag = 1;
								current_screen = current_screen - 2;
								// ESP_LOGI("DISPLAY", "Pilot Auto Enable");
							}
							if (item_selected_enable == 1)
							{
								gb_params.f_protection = 0;
								gb_freeze_protection_flag = 0;
								current_screen = current_screen - 2;
								// ESP_LOGI("DISPLAY", "freeze protection disable ");
							}
						}
						if ((flag_button_select == false) && (button_select_clicked == 1))
						{ // unclick
							button_select_clicked = 0;
						}
						if ((flag_button_back == true) && (button_back_clicked == 0))
						{ // select button clicked, jump between screens
							flag_button_back = false;
							button_back_clicked = 1; // set button to clicked to only perform the action once
							// current_screen--;
							if (current_screen > 0)
							{
								current_screen = current_screen - 2;
							}
						}
						if ((flag_button_back == false) && (button_back_clicked == 1))
						{ // unclick
							button_back_clicked = 0;
						}
						break;
					case 5: // Child Lock button
						update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
						show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
						if ((flag_button_select == true) && (button_select_clicked == 0))
						{
							flag_button_select = false;
							button_select_clicked = 1;
							if (item_selected_enable == 0)
							{
								gb_params.c_lock = true;
								panelStateChanged_child_lock = true;
								current_screen = 1;
							}
							if (item_selected_enable == 1)
							{
								gb_params.c_lock = false;
								panelStateChanged_child_lock = true;
								current_screen = current_screen - 2;
							}
						}
						if ((flag_button_select == false) && (button_select_clicked == 1))
						{ // unclick
							button_select_clicked = 0;
						}
						if ((flag_button_back == true) && (button_back_clicked == 0))
						{ // select button clicked, jump between screens
							flag_button_back = false;
							button_back_clicked = 1; // set button to clicked to only perform the action once
							// current_screen--;
							if (current_screen > 0)
							{
								current_screen = current_screen - 2;
							}
						}
						if ((flag_button_back == false) && (button_back_clicked == 1))
						{ // unclick
							button_back_clicked = 0;
						}
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						control_child_lock(&gb_params);
						break;
					case 6: // device settings.
						if (item_selected_device_setting == 0)
						{
							show_firmware_version();
						}
						if (item_selected_device_setting == 1)
						{
							show_factory_reset();
						}
						if (item_selected_device_setting == 2)
						{
							show_cali();
						}
						if (item_selected_device_setting == 3)
						{
							update_options(&item_selected_unit, &item_sel_previous_unit, &item_sel_next_unit, ENABLE_SETTING_ITEMS);
							show_enable_disable(&temperature_unit_options, &item_selected_unit, &item_sel_previous_unit, &item_sel_next_unit, ENABLE_SETTING_ITEMS);
							update_enable_option(&gb_params.t_unit, &item_selected_unit);
							t_unit_flag = gb_params.t_unit;
							refresh_temperature = (ROOM_TEMP_REFRESH_SEC*10) -10;
							set_temperature(&gb_params);
						}
						if (item_selected_device_setting == 4)
						{
							show_device_about();
						}
						// ESP_LOGI("DISPLAY", "Selected the option %d", item_selected);
						break;
					case 7: // pilot light
						if (item_selected_pilot == 0)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							// ESP_LOGI("DISPLAY", "------------Selected the option %d", item_selected_enable);
							update_enable_option(&gb_params.pilot_light.state, &item_selected_enable);
							control_auto_intensity();
						}
						if (item_selected_pilot == 1)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							// ESP_LOGI("DISPLAY", "------------Selected the option %d", item_selected_enable);
							// update_enable_option(&gb_params.pilot_light.state, &item_selected_enable);
							if ((flag_button_select == true) && (button_select_clicked == 0))
							{
								flag_button_select = false;
								button_select_clicked = 1;
								if (item_selected_enable == 0)
								{
									gb_params.pilot_light.state = 2;
									panelStateChanged_pilot_light = true;
									current_screen = current_screen - 1;
									// ESP_LOGI("DISPLAY", "Pilot Auto Enable");
								}
								if (item_selected_enable == 1)
								{

									panelStateChanged_pilot_light = true;
									gb_params.pilot_light.state = 1;
									current_screen = current_screen - 1;
									// ESP_LOGI("DISPLAY", "pilot auto disable ");
								}
							}

							if ((flag_button_select == false) && (button_select_clicked == 1))
							{ // unclick
								button_select_clicked = 0;
							}
							if ((flag_button_back == true) && (button_back_clicked == 0))
							{ // select button clicked, jump between screens
								flag_button_back = false;
								button_back_clicked = 1; // set button to clicked to only perform the action once
								// current_screen--;
								if (current_screen > 0)
								{
									current_screen--;
								}
							}
							if ((flag_button_back == false) && (button_back_clicked == 1))
							{ // unclick
								button_back_clicked = 0;
							}
							control_auto_intensity();
						}
						if (item_selected_pilot == 2)
						{
							update_brightness(&gb_params.pilot_light.intensity);
							show_brightness();
							set_pilot_light(&gb_params);
						}
						break;
					case 8: // Set schedule for mode
						if (item_selected_schedule == 0)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ON_OFF_SETTING_ITEMS);
							show_enable_disable(on_off_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ON_OFF_SETTING_ITEMS);
							// ESP_LOGI("DISPLAY", "------------Selected the option %d", item_selected_enable);
							update_enable_option(&gb_schedule_flag, &item_selected_enable);
							// control_auto_intensity();
						}
						if (item_selected_schedule == 1)
						{
							show_schedule_with_day();
						}
						if (item_selected_schedule == 2)
						{
							show_schedul_clear();
							// clear schedule
							// for (int i = 0; i < WEEK_DAYS; i++)
							// {
							// 	gb_sch[i] = {0};
							// }
						}
						break;
					case 9:								 // Set time and Day
						if (item_selected_time_day == 0) // Set time
						{
							show_set_time_day();
						}
						if (item_selected_time_day == 1) // set day
						{
							show_present_day();
						}
						if (item_selected_time_day == 2)
						{
							update_options(&item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							show_enable_disable(enable_disable_options, &item_selected_enable, &item_sel_previous_enable, &item_sel_next_enable, ENABLE_SETTING_ITEMS);
							// ESP_LOGI("DISPLAY", "------------Selected the option %d", item_selected_enable);
							update_enable_option(&day_light_saving_flag, &item_selected_enable);
						}
						if (item_selected_time_day == 3) // set day
						{
							show_set_utc_zone();
						}
						break;
					default:
						break;
					}
					vTaskDelay(10 / portTICK_PERIOD_MS);
				}
				vTaskDelay(10 / portTICK_PERIOD_MS);
			} while (u8g2_NextPage(&u8g2)); // required for page drawing mode with u8g library
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		else
		{
			u8g2_SetPowerSave(&u8g2, 1); // sleep display
			// disp_prev_time = esp_timer_get_time() / 1000000;
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}
}

void show_homescreen()
{
	refresh_temperature++;
	if (refresh_temperature == ROOM_TEMP_REFRESH_SEC*10) // around 22 seconds
	{
		refresh_temperature = 0;
		strncpy(temperature_buf_disp, temperature_buf, sizeof(temperature_buf));
	}
	// MENU SCREEN
	u8g2_SetContrast(&u8g2, disp_intensity);
	u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
	// u8g2_SetContrast(&u8g2, disp_intensity);

	u8g2_DrawStr(&u8g2, 0, 10, disp_time);
	u8g2_DrawStr(&u8g2, 13, 10, ":");
	if (wifi_connect_state == true)
		u8g2_DrawBitmap(&u8g2, 33, 1, 16 / 8, 9, wifi_icon);
	else
	{
		u8g2_DrawBitmap(&u8g2, 33, 1, 16 / 8, 9, no_wifi_icon);
	}

	if (ota_update_flag)
	{
		u8g2_DrawBitmap(&u8g2, 71, 1, 16 / 8, 16, ota_icon);
	}

	if (wifi_change_state == true || blufi_state != true)
	{
		if (blufi_state != true)
		{
			if (num1 % 20 == 0)
				u8g2_DrawBitmap(&u8g2, 58, 1, 16 / 8, 11, ble_blink_icon);
			else
			{
				u8g2_DrawBitmap(&u8g2, 58, 1, 16 / 8, 11, ble_icon);
			}
		}
		else
		{
			if (num1 % 10 == 0)
				u8g2_DrawBitmap(&u8g2, 58, 1, 16 / 8, 11, ble_blink_icon);
			else
			{
				u8g2_DrawBitmap(&u8g2, 58, 1, 16 / 8, 11, ble_icon);
			}
		}
		num1++;
	}

	if (gb_params.c_lock == true)
		u8g2_DrawBitmap(&u8g2, 46, 1, 16 / 8, 9, child_lock_icon_new);

	switch (operation_mode)
	{
	case AUTO_MODE:
		u8g2_DrawBitmap(&u8g2, 1, 28, 16 / 8, 11, auto_icon);
		break;
	case TIMER_MODE:
		u8g2_DrawBitmap(&u8g2, 1, 28, 16 / 8, 11, timer_icon);
		break;
	case SCHEDULE_MODE:
		u8g2_DrawBitmap(&u8g2, 1, 28, 16 / 8, 11, schedule_mode_icon);
		break;
	case GEOFENCE_MODE:
		u8g2_DrawBitmap(&u8g2, 1, 28, 16 / 8, 11, geofence_icon);
		break;
	default:
		break;
	}

	u8g2_DrawBitmap(&u8g2, 116, 5, 16 / 8, 11, menu_icon);
	// u8g2_DrawBitmap(&u8g2, 0, 14, 16 / 8, 12, up_arrow);
	// u8g2_DrawBitmap(&u8g2, 0, 52, 16 / 8, 12, down_arrow);

	if (tim_timer_mode_started)
	{

		num++;
		if (num % 5 == 0)
			timer_show_icon = !timer_show_icon;
	}
	else
	{
		num = 0;
		timer_show_icon = true;
	}
	if (timer_show_icon)
		u8g2_DrawBitmap(&u8g2, 117, 42, 16 / 8, 11, timer_icon);

	char buffer[10];
	// sprintf(buffer, "%d", (uint8_t)gb_params.heater.s_temp);
	// ESP_LOGI("-----------", "TEMPE %f", gb_params.heater.s_temp);

	if (gb_params.heater.state)
	{
		if (t_unit_flag == true) // Celsius Mode
		{
			sprintf(buffer, "%d", (uint8_t)round(gb_params.heater.s_temp));
			if ((uint8_t)gb_params.heater.s_temp >= 10 && (uint8_t)gb_params.heater.s_temp < 100)
			{
				u8g2_SetFont(&u8g2, Dejavu);
				u8g2_DrawStr(&u8g2, 89, 24, "o");
				u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
				u8g2_DrawStr(&u8g2, 97, 30, "C");
				u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
				u8g2_DrawStr(&u8g2, 36, 53, buffer);
			}

			if ((uint8_t)gb_params.heater.s_temp < 10 || (uint8_t)gb_params.heater.s_temp == 0)
			{
				u8g2_SetFont(&u8g2, Dejavu);
				u8g2_DrawStr(&u8g2, 79, 24, "o");
				u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
				u8g2_DrawStr(&u8g2, 87, 30, "C");
				u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
				u8g2_DrawStr(&u8g2, 50, 53, buffer);
			}
		}
		else // For fahrenheit values
		{
			sprintf(buffer, "%d", (uint8_t)round(gb_params.heater.s_temp_fer));
			// ESP_LOGI("-----------", "TEMPE %f", gb_params.heater.s_temp_fer);
			if ((uint8_t)gb_params.heater.s_temp_fer >= 10 && (uint8_t)gb_params.heater.s_temp_fer < 100)
			{
				u8g2_SetFont(&u8g2, Dejavu);
				u8g2_DrawStr(&u8g2, 89, 22, "o");
				u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
				u8g2_DrawStr(&u8g2, 97, 28, "F");
				u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
				u8g2_DrawStr(&u8g2, 36, 50, buffer);
			}

			if ((uint8_t)gb_params.heater.s_temp_fer < 10 || (uint8_t)gb_params.heater.s_temp_fer == 0)
			{
				u8g2_SetFont(&u8g2, Dejavu);
				u8g2_DrawStr(&u8g2, 79, 22, "o");
				u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
				u8g2_DrawStr(&u8g2, 87, 28, "F");
				u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
				u8g2_DrawStr(&u8g2, 50, 50, buffer);
			}
		}
	}
	else // if heater state is off
	{
		u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
		u8g2_DrawStr(&u8g2, 48, 47, "--");
	}

	u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
	u8g2_DrawStr(&u8g2, 2, 64, "Room:");
	u8g2_DrawStr(&u8g2, 33, 64, temperature_buf_disp);

	if ((flag_button_up == true) && (button_up_clicked == false))
	{ // up button clicked - jump to previous menu item
		flag_button_up = false;
		button_up_clicked = 1; // set button to clicked to only perform the action once
		if (gb_params.heater.state == false)
		{
			gb_params.heater.state = true;
		}
		if (t_unit_flag == true)
		{
			if (gb_params.heater.s_temp < 32)
			{
				if (operation_mode == TIMER_MODE)
				{
					gb_params.heat_timer.t_temp = gb_params.heat_timer.t_temp + 1;
					gb_params.heater.s_temp = verify_temperature_celsius(gb_params.heat_timer.t_temp);
					panelStateChanged_timer = true;
				}
				else
				{
					if (operation_mode != AUTO_MODE)
					{
						prev_operation_mode = operation_mode;
						operation_mode = AUTO_MODE;
						gb_params.op_mode = AUTO_MODE;
						panelStateChanged_op_mode = true;
					}
					gb_params.heater.s_temp = verify_temperature_celsius(gb_params.heater.s_temp + 1);
					gb_params.auto_heater.s_temp = gb_params.heater.s_temp;
					set_temperature(&gb_params);
					panelStateChanged_heater = true;
				}
			}
		}
		else
		{
			if (gb_params.heater.s_temp_fer < 90)
			{
				if (operation_mode == TIMER_MODE)
				{
					gb_params.heat_timer.t_temp = verify_temperature_celsius(GET_CELSIUS(gb_params.heater.s_temp_fer + 1));
					panelStateChanged_timer = true;
				}
				else
				{
					if (operation_mode != AUTO_MODE)
					{
						prev_operation_mode = operation_mode;
						operation_mode = AUTO_MODE;
						gb_params.op_mode = AUTO_MODE;
						panelStateChanged_op_mode = true;
					}
					gb_params.heater.s_temp_fer = gb_params.heater.s_temp_fer + 1;
					gb_params.auto_heater.s_temp_fer = gb_params.heater.s_temp_fer;
					gb_params.auto_heater.s_temp = GET_CELSIUS(gb_params.auto_heater.s_temp_fer);
					panelStateChanged_heater = true;
				}
				set_temperature(&gb_params);
			}
		}
	}
	if ((flag_button_down == true) && (button_down_clicked == 0))
	{ // down button clicked - jump to next menu item
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		if (gb_params.heater.state == false)
		{
			gb_params.heater.state = true;
		}
		if (t_unit_flag == true)
		{
			if (gb_params.heater.s_temp > 4.4)
			{
				if (operation_mode == TIMER_MODE)
				{
					gb_params.heat_timer.t_temp = gb_params.heat_timer.t_temp - 1;
					gb_params.heater.s_temp = verify_temperature_celsius(gb_params.heat_timer.t_temp);
					panelStateChanged_timer = true;
				}
				else
				{
					if (operation_mode != AUTO_MODE)
					{
						prev_operation_mode = operation_mode;
						operation_mode = AUTO_MODE;
						gb_params.op_mode = AUTO_MODE;
						panelStateChanged_op_mode = true;
					}
					gb_params.heater.s_temp = verify_temperature_celsius(gb_params.heater.s_temp - 1);
					gb_params.auto_heater.s_temp = gb_params.heater.s_temp;
					set_temperature(&gb_params);
					panelStateChanged_heater = true;
				}
			}
		}
		else
		{
			if (gb_params.heater.s_temp_fer > 40)
			{
				if (operation_mode == TIMER_MODE)
				{
					gb_params.heat_timer.t_temp = verify_temperature_celsius(GET_CELSIUS(gb_params.heater.s_temp_fer - 1));
					panelStateChanged_timer = true;
				}
				else
				{
					if (operation_mode != AUTO_MODE)
					{
						prev_operation_mode = operation_mode;
						operation_mode = AUTO_MODE;
						gb_params.op_mode = AUTO_MODE;
						panelStateChanged_op_mode = true;
					}
					gb_params.heater.s_temp_fer = gb_params.heater.s_temp_fer - 1;
					gb_params.auto_heater.s_temp_fer = gb_params.heater.s_temp_fer;
					gb_params.auto_heater.s_temp = GET_CELSIUS(gb_params.auto_heater.s_temp_fer);
					panelStateChanged_heater = true;
				}
				set_temperature(&gb_params);
			}
		}
	}

	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	// selected item background
	//                     x  y   cnt  h
	// u8g2_DrawBitmap(&u8g2, 0, 13, 128 / 8, 1, line);
	// draw previous item as icon + label
	// ESP_LOGI("DISPLAY", "Home screen");
	// u8g2_SendBuffer(&u8g2);
	vTaskDelay(15 / portTICK_PERIOD_MS);
}

void show_menu()
{
	// selected item background
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawBitmap(&u8g2, 0, 23, 128 / 8, 20, bitmap_item_sel_outline);
	// draw previous item as icon + label

	u8g2_DrawStr(&u8g2, 25, 15, menu_items[item_sel_previous]);
	u8g2_DrawBitmap(&u8g2, 4, 2, 16 / 8, 16, bitmap_icons[item_sel_previous]);

	// draw selected item as icon + label in bold font
	u8g2_DrawStr(&u8g2, 25, 15 + 20 + 2, menu_items[item_selected]);
	u8g2_DrawBitmap(&u8g2, 4, 24, 16 / 8, 16, bitmap_icons[item_selected]);

	// ESP_LOGI("DISPLAY", "selected Item %s", menu_items[item_selected]);
	//
	// draw next item as icon + label
	u8g2_DrawStr(&u8g2, 25, 15 + 20 + 20 + 2 + 2, menu_items[item_sel_next]);
	u8g2_DrawBitmap(&u8g2, 4, 46, 16 / 8, 16, bitmap_icons[item_sel_next]);

	// draw scrollbar background
	u8g2_DrawBitmap(&u8g2, 128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);

	// draw scrollbar handle
	u8g2_DrawBox(&u8g2, 125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);

	// ESP_LOGI("DISPLAY", "INSIDE Menu screen");
}

void update_option_screen(char (*menu_list)[30], int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items)
{
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawBitmap(&u8g2, 0, 23, 128 / 8, 20, bitmap_item_sel_outline);
	// draw previous item as icon + label

	u8g2_DrawStr(&u8g2, 5, 15, menu_list[*item_sel_previous]);
	// u8g2_DrawBitmap(&u8g2, 4, 2, 16 / 8, 16, bitmap_icons[item_sel_previous_screen]);

	// draw selected item as icon + label in bold font
	u8g2_DrawStr(&u8g2, 5, 15 + 20 + 2, menu_list[*item_selected]);
	// u8g2_DrawBitmap(&u8g2, 4, 24, 16 / 8, 16, bitmap_icons[item_selected_screen]);
	// ESP_LOGI("DISPLAY", "selected Item %s", menu_list[*item_selected]);

	// draw next item as icon + label
	u8g2_DrawStr(&u8g2, 5, 15 + 20 + 20 + 2 + 2, menu_list[*item_sel_next]);
	// u8g2_DrawBitmap(&u8g2, 4, 46, 16 / 8, 16, bitmap_icons[item_sel_next_sreen]);

	// draw scrollbar background
	u8g2_DrawBitmap(&u8g2, 128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);
	// draw scrollbar handle
	u8g2_DrawBox(&u8g2, 125, 64 / num_items * (*item_selected), 3, 64 / num_items);
}

/// @brief this function update the screen list
void update_options(int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items)
{

	if ((flag_button_up == true) && (button_up_clicked == false))
	{
		flag_button_up = false;				 // up button clicked - jump to previous menu item
		*item_selected = *item_selected - 1; // select previous item
		button_up_clicked = 1;				 // set button to clicked to only perform the action once
		if (*item_selected < 0)
		{ // if first item was selected, jump to last item
			*item_selected = num_items - 1;
		}
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{										 // down button clicked - jump to next menu item
		*item_selected = *item_selected + 1; // select next item
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		if (*item_selected >= num_items)
		{ // last item was selected, jump to first menu item
			*item_selected = 0;
		}
	}

	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	*item_sel_previous = *item_selected - 1;
	if (*item_sel_previous < 0)
	{
		*item_sel_previous = num_items - 1;
	} // previous item would be befalse first = make it the last
	*item_sel_next = *item_selected + 1;
	if (*item_sel_next >= num_items)
	{
		*item_sel_next = 0;
	} // next item would be after last = make it the first
}

void show_brightness()
{
	u8g2_DrawStr(&u8g2, 30, 60, "Brightness");
	u8g2_DrawFrame(&u8g2, 7, 11, 110, 25);
	u8g2_DrawStr(&u8g2, 2, 47, "0%");
	u8g2_DrawStr(&u8g2, 62, 47, "'");
	u8g2_DrawStr(&u8g2, 98, 47, "100%");
	u8g2_DrawBox(&u8g2, 12, 15, progress, 17);
}

void show_timer()
{
	// u8g2_DrawBitmap(&u8g2, 1, 1, 16 / 8, 12, up_arrow);
	// u8g2_DrawBitmap(&u8g2, 1, 53, 16 / 8, 12, down_arrow);
	if (tim_timer_mode_started)
		u8g2_DrawBitmap(&u8g2, 115, 3, 16 / 8, 14, cancel_icon);
	else
		u8g2_DrawBitmap(&u8g2, 115, 6, 16 / 8, 11, ok_icon_timer);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 35, 10, "Set Timer");
	u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
	u8g2_DrawStr(&u8g2, 14, 51, timer_set_buffer);

	if (tim_timer_mode_started)
	{
		num++;
		if (num % 5 == 0)
			timer_show_icon = !timer_show_icon;
	}
	else
	{
		num = 0;
		timer_show_icon = true;
	}
	if (timer_show_icon)
		u8g2_DrawStr(&u8g2, 41, 45, ":");
	u8g2_DrawBitmap(&u8g2, 115, 42, 16 / 8, 11, cancel_icon_timer);

	if (gb_params.hold == true)
	{
		u8g2_SetFont(&u8g2, Dejavu);
		u8g2_DrawStr(&u8g2, 5, 63, "Please disable hold");
	}
}

void show_enable_disable(char (*menu_list)[30], int *item_selected, int *item_sel_previous, int *item_sel_next, int num_items)
{
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawBitmap(&u8g2, 0, 23, 128 / 8, 20, bitmap_item_sel_outline);
	// draw previous item as icon + label

	// draw selected item as icon + label in bold font
	u8g2_DrawStr(&u8g2, 5, 15 + 20 + 2, menu_list[*item_selected]);

	// draw next item as icon + label
	u8g2_DrawStr(&u8g2, 5, 15 + 20 + 20 + 2 + 2, menu_list[*item_sel_next]);

	// draw scrollbar background
	u8g2_DrawBitmap(&u8g2, 128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);
	// draw scrollbar handle
	u8g2_DrawBox(&u8g2, 125, 64 / num_items * (*item_selected), 3, 64 / num_items);
}

void update_enable_option(uint8_t *value, int *enable_selected)
{
	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		if (*enable_selected == 0)
		{
			*value = 1;
			if (item_selected == 0)
			{
				*value = 0;
			}
			current_screen = current_screen - 1;
			ESP_LOGI("DISPLAY", " Changing Enable %d", *value);
		}
		if (*enable_selected == 1)
		{
			*value = 0;
			if (item_selected == 0)
			{
				*value = 1;
			}
			current_screen = current_screen - 1;
			ESP_LOGI("DISPLAY", " Changing Enable %d", *value);
		}
		if (item_selected == 0)
		{
			panelStateChanged_display = true;
		}
		if (item_selected == 1)
		{
			panelStateChanged_night_light = true;
		}
		if (item_selected == 4)
		{
			panelStateChanged_freeze_lock = true;
			current_screen = current_screen - 1;
		}
		if (item_selected == 5)
		{
			panelStateChanged_child_lock = true;
			// current_screen = current_screen - 1;
		}
		if (item_selected == 6)
		{
			nvs_handle_t my_handle;
			esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
			if (err != ESP_OK)
			{
				ESP_LOGI("DISPLAY", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
			}
			// storing the temperature offset info the nvs
			err = nvs_set_i32(my_handle, "t_unit", gb_params.t_unit);
			ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
			ESP_LOGI("Eheat", "Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
			nvs_close(my_handle);
		}
		if (item_selected == 7)
		{
			panelStateChanged_pilot_light = true;
		}
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		// current_screen--;
		if (current_screen > 0)
		{
			current_screen--;
		}
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
}

void update_brightness(uint8_t *value)
{
	// progress = gb_params.night_light.intensity;
	if ((*value % 5) != 0)
	{
		int temp = *value % 5;
		*value = *value - temp;
		// ESP_LOGI("DISPLAY", "Calibrate %d", *value);
	}
	progress = *value;
	if ((flag_button_up == true) && (button_up_clicked == false))
	{ // up button clicked - jump to previous menu item
		flag_button_up = false;
		if (*value < 100)
		{
			*value = *value + 5;
		}
		// ESP_LOGI("DISPLAY", "Value %d", *value);
		// *value = GET_NIGHT_LIGHT_INTENS(*value);
		// set_color(&gb_params);
		button_up_clicked = 1; // set button to clicked to only perform the action once
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{ // down button clicked - jump to next menu item
		flag_button_down = false;
		if ((*value <= 100) && (*value > 0))
		{
			*value = *value - 5;
		}
		// ESP_LOGI("DISPLAY", "Value %d", *value);
		progress = *value;
		// *value = GET_NIGHT_LIGHT_INTENS(*value);
		// set_color(&gb_params);
		button_down_clicked = 1; // set button to clicked to only perform the action once
	}
	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_select = false;
		button_select_clicked = 1; // set button to clicked to only perform the action once
		if (item_selected == 0)
		{
			panelStateChanged_display = true;
		}
		if (item_selected == 1)
		{
			panelStateChanged_night_light = true;
		}
		if (item_selected == 7)
		{
			panelStateChanged_pilot_light = true;
		}
		// current_screen++;
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
}

void update_timer()
{
	if ((flag_button_up == true) && (button_up_clicked == false))
	{ // up button clicked - jump to previous menu item
		flag_button_up = false;
		if (tim_timer_mode_started)
		{
			timer_stop_call();
		}
		inc_det = true;
		timer_schedule();
		button_up_clicked = 1; // set button to clicked to only perform the action once
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{
		flag_button_down = false;
		if (tim_timer_mode_started)
		{
			timer_stop_call();
		}
		dec_det = true;
		timer_schedule();		 // down button clicked - jump to next menu item
		button_down_clicked = 1; // set button to clicked to only perform the action once
	}
	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		if (gb_params.hold == false)
		{
			if (current_screen == 0)
			{
				if (tim_timer_mode_started)
				{
					timer_stop_call();
					gb_params.heat_timer.state = false;
					panelStateChanged_timer = true;
				}
				else
				{
					start_det = true;
					timer_schedule();
					gb_params.heat_timer.state = true;
					panelStateChanged_timer = true;
					gb_params.heater.state = true;
					set_temperature(&gb_params);
				}
				current_screen = 1;
			}
			else
				current_screen--;
		}
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0)
			current_screen = 1;
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	// vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_ip_address()
{

	char buffer[30];
	static uint64_t num = 0;
	static wifi_ap_record_t ap;
	static esp_netif_ip_info_t ip_info;
	static esp_netif_t *netif = NULL;

	if (num % 50 == 0)
	{
		netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
		if (netif == NULL)
			ESP_LOGI("DISPLAY", "Pointer is NULL.\n");
		else
		{
			esp_netif_get_ip_info(netif, &ip_info);
			// ESP_LOGI("DISPLAY","My IP: " IPSTR "\n", IP2STR(&ip_info.ip));
		}
		esp_wifi_sta_get_ap_info(&ap);
		// ESP_LOGI("DISPLAY", "IP ************ %s", ap.ssid);
		num = 0;
	}
	u8g2_DrawStr(&u8g2, 3, 15, "Connected to:");
	sprintf(buffer, "%s", ap.ssid);
	u8g2_DrawStr(&u8g2, 3, 28, buffer);

	u8g2_DrawBitmap(&u8g2, 0, 35, 128 / 8, 1, line);

	u8g2_DrawStr(&u8g2, 3, 48, "Device IP :");
	sprintf(buffer, IPSTR, IP2STR(&ip_info.ip));
	u8g2_DrawStr(&u8g2, 3, 60, buffer);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);
	num = num + 1;

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0)
			current_screen = 1;
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_wifi_signal()
{
	char buffer[20];
	static uint64_t num = 0;
	static int quality = 0;
	wifi_ap_record_t ap;

	if (num % 50 == 0)
	{
		esp_wifi_sta_get_ap_info(&ap);
		// ESP_LOGI("DISPLAY", "IP ************ %s", ap.rssi);
		quality = 2 * (ap.rssi + 100);
		// ESP_LOGI("DISPLAY","%d\n", quality);
		num = 0;
	}
	if (quality > 100)
		quality = 100;
	else if (quality < 0 || wifi_connect_state == false)
		quality = 0;
	sprintf(buffer, "Signal- %d", quality);
	u8g2_DrawStr(&u8g2, 2, 57, buffer);
	u8g2_DrawStr(&u8g2, 65, 57, "%");
	u8g2_DrawBox(&u8g2, 13, 9, quality, 17);
	u8g2_DrawFrame(&u8g2, 8, 5, 110, 25);
	u8g2_DrawStr(&u8g2, 2, 42, "Weak");
	u8g2_DrawStr(&u8g2, 62, 42, "'");
	u8g2_DrawStr(&u8g2, 90, 42, "Strong");
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);
	num = num + 1;

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0)
			current_screen = 1;
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_wifi_change()
{
	// changing the
	u8g2_DrawStr(&u8g2, 3, 12, "To change the");
	u8g2_DrawStr(&u8g2, 3, 24, "network, ensure");
	u8g2_DrawStr(&u8g2, 3, 36, "you have the mobile");
	u8g2_DrawStr(&u8g2, 3, 47, "app handy,");
	u8g2_DrawStr(&u8g2, 3, 59, "want to Continue ?");
	// u8g2_DrawStr(&u8g2, 3, 42, "No");
	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0)
			current_screen = 1;
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen = HOME_SCREEN;
		wifi_change_state = true;
		vTaskSuspend(shadow_taskHandler);
		prev_time = esp_timer_get_time() / 1000000;
		// ESP_LOGI("DISPLAY","Current time (in seconds): %lld\n", current_time);
		initialise_blufi();
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
}

void show_temperature()
{
	u8g2_DrawBitmap(&u8g2, 0, 1, 16 / 8, 12, up_arrow);
	u8g2_DrawBitmap(&u8g2, 0, 52, 16 / 8, 12, down_arrow);
	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_DrawBitmap(&u8g2, 117, 52, 16 / 8, 14, back_arrow);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 30, 10, "Temperature");
	char buffer[10];
	// sprintf(buffer, "%d", disp_temperature);

	if (gb_params.heat_timer.t_temp == 0)
	{
		gb_params.heat_timer.t_temp = 5;
		disp_temperature = 40;
	}
	else
	{
		disp_temperature = ((gb_params.heat_timer.t_temp * 9) / 5 + 32);
	}
	if (t_unit_flag == true) // Celsius Mode
	{
		sprintf(buffer, "%d", (uint8_t)round(gb_params.heat_timer.t_temp));
		if ((uint8_t)gb_params.heat_timer.t_temp >= 10 && (uint8_t)gb_params.heat_timer.t_temp < 100)
		{
			u8g2_SetFont(&u8g2, Dejavu);
			u8g2_DrawStr(&u8g2, 89, 24, "o");
			u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
			u8g2_DrawStr(&u8g2, 97, 30, "C");
			u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
			u8g2_DrawStr(&u8g2, 36, 53, buffer);
		}

		if ((uint8_t)gb_params.heat_timer.t_temp < 10 || (uint8_t)gb_params.heat_timer.t_temp == 0)
		{
			u8g2_SetFont(&u8g2, Dejavu);
			u8g2_DrawStr(&u8g2, 79, 24, "o");
			u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
			u8g2_DrawStr(&u8g2, 87, 30, "C");
			u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
			u8g2_DrawStr(&u8g2, 50, 53, buffer);
		}
	}
	else // For fahrenheit values
	{
		sprintf(buffer, "%d", (uint8_t)round(disp_temperature));
		if (disp_temperature >= 10 && disp_temperature < 100)
		{
			u8g2_SetFont(&u8g2, Dejavu);
			u8g2_DrawStr(&u8g2, 89, 22, "o");
			u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
			u8g2_DrawStr(&u8g2, 97, 28, "F");
			u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
			u8g2_DrawStr(&u8g2, 36, 50, buffer);
		}

		if (disp_temperature < 10 || disp_temperature == 0)
		{
			u8g2_SetFont(&u8g2, Dejavu);
			u8g2_DrawStr(&u8g2, 79, 22, "o");
			u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
			u8g2_DrawStr(&u8g2, 87, 28, "F");
			u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
			u8g2_DrawStr(&u8g2, 50, 50, buffer);
		}
	}

	u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
	// sprintf(buffer, "%.2fF", Tf);
	u8g2_DrawStr(&u8g2, 33, 64, temperature_buf);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}

	if ((flag_button_up == true) && (button_up_clicked == false))
	{ // up button clicked - jump to previous menu item
		flag_button_up = false;
		button_up_clicked = 1; // set button to clicked to only perform the action once
		if (t_unit_flag == true)
		{
			if (gb_params.heat_timer.t_temp < 32)
			{
				gb_params.heat_timer.t_temp = verify_temperature_celsius(gb_params.heat_timer.t_temp + 1);
			}
		}
		else
		{
			if (disp_temperature < 90)
			{
				disp_temperature = disp_temperature + 1;
				gb_params.heat_timer.t_temp = verify_temperature_celsius(((disp_temperature - 32) * 5 / 9));
			}
		}
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{ // down button clicked - jump to next menu item
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		if (t_unit_flag == true)
		{
			if (gb_params.heat_timer.t_temp > 5)
			{
				gb_params.heat_timer.t_temp = verify_temperature_celsius(gb_params.heat_timer.t_temp - 1);
			}
		}
		else
		{
			if (disp_temperature > 40)
			{
				disp_temperature = disp_temperature - 1;
				gb_params.heat_timer.t_temp = verify_temperature_celsius(((disp_temperature - 32) * 5 / 9));
			}
		}
	}

	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		if (tim_timer_mode_started)
		{
			set_temperature(&gb_params);
		}
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	// disp_temperature = (uint8_t)gb_params.heat_timer.t_temp;
	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void update_child_lock()
{
	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen = current_screen - 2;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1;
		current_screen = current_screen - 2;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_firmware_version()
{
	char version[20];
	u8g2_DrawFrame(&u8g2, 6, 8, 115, 40);
	u8g2_DrawStr(&u8g2, 10, 22, "Firmware Version:");
	sprintf(&version, "Eheat-firm-%d.%d.%d", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD);
	u8g2_DrawStr(&u8g2, 10, 40, version);

	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		if (current_screen == 0)
			current_screen = 1;
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_factory_reset()
{
	// changing the
	u8g2_DrawStr(&u8g2, 3, 12, "This setting will");
	u8g2_DrawStr(&u8g2, 3, 24, "reset the device");
	u8g2_DrawStr(&u8g2, 3, 36, "& erase device data");
	u8g2_DrawStr(&u8g2, 3, 47, "including network,");
	u8g2_DrawStr(&u8g2, 3, 59, "want to Continue ?");
	// u8g2_DrawStr(&u8g2, 3, 42, "No");
	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(15 / portTICK_PERIOD_MS);

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen = HOME_SCREEN;
		reset_factory_device();
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
}

void show_device_about()
{
	// changing the
	u8g2_DrawFrame(&u8g2, 6, 8, 115, 40);
	u8g2_DrawStr(&u8g2, 10, 22, "Device ID:");
	// sprintf(buffer, "%s", ap.ssid);
	u8g2_DrawStr(&u8g2, 10, 40, mac_id);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(15 / portTICK_PERIOD_MS);
	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
}

void show_cali()
{
	// u8g2_DrawBitmap(&u8g2, 0, 1, 16 / 8, 12, up_arrow);
	// u8g2_DrawBitmap(&u8g2, 0, 52, 16 / 8, 12, down_arrow);
	u8g2_DrawBitmap(&u8g2, 117, 1, 16 / 8, 11, ok_icon_timer);
	if (calibration_flag == true)
		u8g2_DrawBitmap(&u8g2, 117, 45, 16 / 8, 14, cancel_icon_timer);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 30, 10, "Temperature");
	char buffer[10];
	// static int offset = 0;
	sprintf(buffer, "%d", ambient_temperature);

	if (ambient_temperature > 0 && ambient_temperature < 100)
	{
		u8g2_SetFont(&u8g2, Dejavu);
		u8g2_DrawStr(&u8g2, 87, 22, "o");
		u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
		if (t_unit_flag == true)
			u8g2_DrawStr(&u8g2, 94, 30, "C");
		else
			u8g2_DrawStr(&u8g2, 94, 30, "F");
		u8g2_SetFont(&u8g2, u8g2_font_fur30_tn);
		u8g2_DrawStr(&u8g2, 40, 46, buffer);
	}
	else
	{
		u8g2_SetFont(&u8g2, Dejavu);
		u8g2_DrawStr(&u8g2, 92, 20, "o");
		u8g2_SetFont(&u8g2, u8g2_font_profont22_tf);
		// u8g2_DrawStr(&u8g2, 103, 30, "C");
		if (t_unit_flag == true)
			u8g2_DrawStr(&u8g2, 100, 30, "C");
		else
			u8g2_DrawStr(&u8g2, 100, 30, "F");

		u8g2_SetFont(&u8g2, u8g2_font_fur30_tn);
		u8g2_DrawStr(&u8g2, 19, 46, buffer);
	}

	u8g2_SetFont(&u8g2, Dejavu);

	if (temperature_offset >= 0)
	{
		sprintf(buffer, "calibration:+%0d", temperature_offset);
	}
	else
	{
		sprintf(buffer, "calibration:%0d", temperature_offset);
	}

	u8g2_DrawStr(&u8g2, 28, 60, buffer);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once

		nvs_handle_t my_handle;
		esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
		{
			ESP_LOGI("DISPLAY", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
		}

		// storing the temperature offset info the nvs
		err = nvs_get_i32(my_handle, "t_offset", &temperature_offset);
		// ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
		ESP_LOGI("Eheat", "Committing updates in NVS ... ");
		err = nvs_commit(my_handle);
		nvs_close(my_handle);
		// temperature_offset = 0;
		if (calibration_flag == false)
		{
			current_screen = 1;
			item_selected_device_setting = 0;
			item_selected = 0;
			item_sel_previous = item_selected - 1;
			item_sel_next = item_selected + 1;
			// current_screen = 4;
		}
		else
			current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}

	if ((flag_button_up == true) && (button_up_clicked == false))
	{ // up button clicked - jump to previous menu item
		flag_button_up = false;
		button_up_clicked = 1; // set button to clicked to only perform the action once
		ambient_temperature = ambient_temperature + 1;
		temperature_offset++;
		// set_temperature(&gb_params);
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{ // down button clicked - jump to next menu item
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		ambient_temperature = ambient_temperature - 1;
		temperature_offset--;
		// set_temperature(&gb_params);
	}

	if ((flag_button_up == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		refresh_temperature = (ROOM_TEMP_REFRESH_SEC*10) -10;
		nvs_handle_t my_handle;
		esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
		{
			ESP_LOGI("DISPLAY", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
		}

		// storing the temperature offset info the nvs
		err = nvs_set_i32(my_handle, "t_offset", temperature_offset);
		ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
		// Close
		if (calibration_flag == false)
		{
			calibration_flag = true;
			ESP_LOGI("Eheat", "calibration_flag = %d", calibration_flag);
			err = nvs_set_i32(my_handle, "cali_flag", calibration_flag);
			ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
			current_screen = 1;
		}
		else
			current_screen--;

		ESP_LOGI("Eheat", "Committing updates in NVS ... ");
		err = nvs_commit(my_handle);
		nvs_close(my_handle);
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}

	vTaskDelay(5 / portTICK_PERIOD_MS);
}

void show_set_time_day()
{
	char current_time[9] = "00 00";
	static int hour = 12;
	static int minutes = 0;
	// ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%d", disp_time[0]);
	static bool set_arrow_flag = false;
	static bool set_flag = false;
	// for (int i = 0; i < sizeof(current_time); i++)
	// {
	// 	if (disp_time[i] != ':')
	// 		current_time[i] = disp_time[i];
	// 	else
	// 		current_time[i] = ' ';
	// }

	// u8g2_DrawBitmap(&u8g2, 1, 1, 16 / 8, 12, up_arrow);
	if (set_arrow_flag == true)
	{
		u8g2_DrawBitmap(&u8g2, 90, 53, 16 / 8, 12, up_arrow);
	}

	else if (set_arrow_flag == false)
	{
		u8g2_DrawBitmap(&u8g2, 22, 53, 16 / 8, 12, up_arrow);
	}

	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 35, 11, "Set Time");
	u8g2_SetFont(&u8g2, u8g2_font_fur35_tn);
	sprintf(&current_time, "%02d %02d", hour, minutes);
	// ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%s", current_time);
	u8g2_DrawStr(&u8g2, 1, 51, current_time);

	if (1)
	{
		num++;
		if (num % 5 == 0)
			timer_show_icon = !timer_show_icon;
	}
	else
	{
		num = 0;
		timer_show_icon = true;
	}
	if (timer_show_icon)
		u8g2_DrawStr(&u8g2, 55, 45, ":");
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

	if ((flag_button_up == true) && (button_up_clicked == false))
	{						   // up button clicked - jump to previous menu item
		button_up_clicked = 1; // set button to clicked to only perform the action once
		flag_button_up = false;
		// set_arrow_flag = true;
		if (set_flag == false)
		{
			set_arrow_flag = true;
		}
		if (set_arrow_flag == true && set_flag == true)
		{
			if (minutes >= 0 && minutes < 59)
				minutes++;
		}
		else if (set_arrow_flag == false && set_flag == true)
		{
			if (hour >= 0 && hour < 23)
				hour++;
		}
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		if (set_flag == false)
		{
			set_arrow_flag = false;
		}
		if (set_arrow_flag == true && set_flag == true)
		{
			if (minutes > 0)
				minutes--;
		}
		else if (set_arrow_flag == false && set_flag == true)
		{
			if (hour > 0)
				hour--;
		}
		// set_arrow_flag = false;
	}
	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		if (set_flag == false)
		{
			set_flag = true;
		}
		else
		{
			set_flag = false;
			set_device_time(hour, minutes, 00);
			current_screen--;
		}
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		set_flag = false;
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
}

void show_present_day()
{
	static int selected_week_day = 0;
	u8g2_DrawBitmap(&u8g2, 1, 1, 16 / 8, 12, up_arrow);
	u8g2_DrawBitmap(&u8g2, 1, 53, 16 / 8, 12, down_arrow);
	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 35, 11, "Set Day");
	u8g2_SetFont(&u8g2, u8g2_font_fur30_tf);
	u8g2_DrawStr(&u8g2, 18, 48, week_days[selected_week_day]);

	if ((flag_button_up == true) && (button_up_clicked == false))
	{						   // up button clicked - jump to previous menu item
		button_up_clicked = 1; // set button to clicked to only perform the action once
		flag_button_up = false;
		if (selected_week_day >= 0 && selected_week_day < 6)
			selected_week_day++;
	}

	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once
		if (selected_week_day > 0 && selected_week_day <= 6)
			selected_week_day--;
	}
	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}

	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
}

void show_set_utc_zone()
{
	char current_time[9] = "00 00";
	static int hour = 12;
	static int minutes = 0;
	static int seconds = TIMER_CHANGE_INTERVAL;
	// ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%d", disp_time[0]);
	static bool set_arrow_flag = false;
	static bool set_flag = false;
	int minute = 0;
	// u8g2_DrawBitmap(&u8g2, 1, 1, 16 / 8, 12, up_arrow);
	// if (set_arrow_flag == true)
	// {
	// 	u8g2_DrawBitmap(&u8g2, 90, 53, 16 / 8, 12, up_arrow);
	// }

	// else if (set_arrow_flag == false)
	// {
	// 	u8g2_DrawBitmap(&u8g2, 22, 53, 16 / 8, 12, up_arrow);
	// }

	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_SetFont(&u8g2, Dejavu);
	u8g2_DrawStr(&u8g2, 25, 11, "Set Timezone");
	u8g2_DrawStr(&u8g2, 30, 64, "UTC RANGE");

	// sprintf(&current_time, "%02d %02d", hour, minutes);

	minute = Minutes(seconds);
	hour = Hours(seconds);
	if (minute < 0 || hour < 0)
	{
		sprintf(current_time, " %02d:%02d", (hour < 0) ? -hour : hour, (minute < 0) ? -minute : minute);
		u8g2_SetFont(&u8g2, u8g2_font_fub20_tf);
		u8g2_DrawStr(&u8g2, 5, 40, "-");
	}
	if (minute > 0 || hour > 0)
	{
		sprintf(current_time, " %02d:%02d", (hour < 0) ? -hour : hour, (minute < 0) ? -minute : minute);
		u8g2_SetFont(&u8g2, u8g2_font_fub17_tf);
		u8g2_DrawStr(&u8g2, 0, 37, "+");
	}
	if (minute == 0 && hour == 0)
		sprintf(current_time, " %02d:%02d", (hour < 0) ? -hour : hour, (minute < 0) ? -minute : minute);
	// sprintf(current_time, "%02d:%02d", hour, (minute < 0) ? -minute : minute);

	// sprintf(current_time, "%02d:%02d", Hours(seconds), Minutes(seconds));
	// ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%02d %02d", hour, minutes);

	u8g2_SetFont(&u8g2, u8g2_font_fur30_tn);
	u8g2_DrawStr(&u8g2, 9, 47, current_time);

	// u8g2_DrawStr(&u8g2, 55, 45, ":");
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);
	ESP_LOGI("time seconds->>>>>>>>>>>>>>>>>>>>", "%d", seconds);
	if ((flag_button_up == true) && (button_up_clicked == false))
	{						   // up button clicked - jump to previous menu item
		button_up_clicked = 1; // set button to clicked to only perform the action once
		flag_button_up = false;
		// set_arrow_flag = true;
		if (seconds > -43200)
		{
			seconds -= TIMER_CHANGE_INTERVAL;
		}
	}
	else if ((flag_button_down == true) && (button_down_clicked == 0))
	{
		flag_button_down = false;
		button_down_clicked = 1; // set button to clicked to only perform the action once

		if (seconds < 43200)
		{
			seconds += TIMER_CHANGE_INTERVAL;
		}
	}
	if ((flag_button_down == false) && (button_up_clicked == 1))
	{ // unclick
		button_up_clicked = 0;
	}
	if ((flag_button_down == false) && (button_down_clicked == 1))
	{ // unclick
		button_down_clicked = 0;
	}

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;

		// set_device_time(hour, minutes, 00);
		// if(Hours(seconds))
		// sprintf(timezone, "UTC-%01d:%02d", Hours(seconds), Minutes(seconds));
		if (Hours(seconds) < 0)
			sprintf(timezone, "UTC+%01d:%02d", (hour < 0) ? -hour : hour, (minute < 0) ? -minute : minute);

		// sprintf(timezone, "UTC%01d:%02d", Hours(seconds), (minute < 0) ? -minute : minute);
		else
			sprintf(timezone, "UTC-%01d:%02d", Hours(seconds), (minute < 0) ? -minute : minute);
		// sprintf(timezone, "UTC+%01d:%02d", Hours(seconds), (minute < 0) ? -minute : minute);

		// sprintf(current_time, "%02d %02d", Hours(seconds), Minutes(seconds));
		ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%s", timezone);
		store_string("timezone", timezone);
		setenv("TZ", timezone, 1);
		tzset();
		// obtain_time();
		get_time_update();
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
	/****************************************************************************/
	// get back button level
	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once
		set_flag = false;
		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
}

void show_bootloop(void)
{
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetBitmapMode(&u8g2, true /* solid */);
	u8g2_DrawBitmap(&u8g2, 30, 15, 9, 32, envi_removebg_preview);
	u8g2_SendBuffer(&u8g2);
	u8g2_SetFont(&u8g2, Dejavu);

	for (int K = 0; K < 4; K++)
	{
		for (int i = 10; i < 255; i = i + 20)
		{
			u8g2_SetContrast(&u8g2, i);
			vTaskDelay(35 / portTICK_PERIOD_MS);
		}
		for (int i = 255; i >= 0; i = i - 20)
		{
			u8g2_SetContrast(&u8g2, i);
			vTaskDelay(35 / portTICK_PERIOD_MS);
		}
		// if (K == 0)
		// 	u8g2_DrawStr(&u8g2, 24, 55, "Loading eHeat.");
		// if (K == 1)
		// {
		// 	u8g2_DrawStr(&u8g2, 24, 55, "Loading eHeat..");
		// }
		// if (K == 2)
		// 	u8g2_DrawStr(&u8g2, 24, 55, "Loading eHeat...");
		u8g2_SendBuffer(&u8g2);
	}
	u8g2_SendBuffer(&u8g2);
	u8g2_SetContrast(&u8g2, 0xff);
}

void show_schedule_with_day()
{
	char current_time[9] = "00:00";
	static int hour = 12;
	// static int minutes = 0;

	static int seconds = TIMER_CHANGE_INTERVAL;

	static bool isUpdate = false;
	static bool set_flag = false;
	static bool set_temp_flag = true;
	static bool set_week_flag = true;
	static bool set_event_flag = true;
	static bool set_time_flag = true;
	static bool set_edit_flag = false;
	static uint8_t select_flag = 0;
	static uint8_t Selected_event_option = 0; // Event wake leave return sleep
	static uint8_t Selected_event_days = 0;	  // week

	static float dummy_temp = 30;
	static float dummy_temp_fer = 40;
	static int blink_count = 0;
	int minute = 0;
	if (!blufi_state)
	{
		static sch_t current_schedule[7];

		if (set_edit_flag == true)
		{
			u8g2_DrawBitmap(&u8g2, 117, 1, 16 / 8, 11, ok_icon_timer);
		}
		else
		{
			u8g2_DrawBitmap(&u8g2, 117, 1, 16 / 8, 11, menu_icon);
		}

		u8g2_DrawBitmap(&u8g2, 117, 52, 16 / 8, 14, cancel_icon_timer);
		u8g2_SetFont(&u8g2, Dejavu);
		u8g2_DrawStr(&u8g2, 25, 11, "Set Schedule");

		minute = Minutes(seconds);
		hour = Hours(seconds);

		sprintf(current_time, "%02d:%02d", Hours(seconds), Minutes(seconds));
		// ESP_LOGI("time->>>>>>>>>>>>>>>>>>>>", "%02d %02d", hour, minutes);

		if (blink_count % 5 == 0)
		{
			blink_count = 0;
			switch (select_flag)
			{
			case 0:
				set_temp_flag = true;
				set_time_flag = true;
				set_week_flag = true;
				set_event_flag = !set_event_flag;
				break;
			case 1:
				set_week_flag = true;
				set_time_flag = !set_time_flag;
				set_temp_flag = true;
				set_event_flag = true;
				break;
			case 2:
				set_time_flag = true;
				set_week_flag = true;
				set_temp_flag = !set_temp_flag;
				set_event_flag = true;
				break;
			case 3:
				set_event_flag = true;
				set_week_flag = !set_week_flag;
				set_temp_flag = true;
				set_time_flag = true;
				break;
			default:
				set_event_flag = true;
				set_week_flag = true;
				set_temp_flag = true;
				set_time_flag = true;
				break;
			}
		}

		if (set_event_flag)
		{
			u8g2_DrawStr(&u8g2, 3, 24, event_options[Selected_event_option]);
		}

		if (set_time_flag)
		{
			u8g2_SetFont(&u8g2, u8g2_font_fur17_tn);
			u8g2_DrawStr(&u8g2, 1, 46, current_time);
		}

		if (set_week_flag)
		{
			u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
			u8g2_DrawStr(&u8g2, 1, 64, event_day_options[Selected_event_days]);
		}

		// ESP_LOGI("time seconds->>>>>>>>>>>>>>>>>>>>", "%d", seconds);
		char buffer[10];
		sprintf(buffer, "%d", (uint8_t)gb_params.heater.s_temp);

		// show temperature snippet.
		if (set_temp_flag)
		{
			if (t_unit_flag == true)
			{
				sprintf(buffer, "%d", (uint8_t)dummy_temp);
				if ((uint8_t)dummy_temp > 10 && (uint8_t)dummy_temp <= 32)
				{
					u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
					u8g2_DrawStr(&u8g2, 110, 26, "o");
					u8g2_SetFont(&u8g2, u8g2_font_profont17_tf);
					u8g2_DrawStr(&u8g2, 117, 32, "C");
					u8g2_SetFont(&u8g2, u8g2_font_fur25_tn);
					u8g2_DrawStr(&u8g2, 70, 47, buffer);
				}

				if ((uint8_t)dummy_temp < 10)
				{
					u8g2_SetFont(&u8g2, u8g2_font_mozart_nbp_tf);
					u8g2_DrawStr(&u8g2, 110, 26, "o");
					u8g2_SetFont(&u8g2, u8g2_font_profont17_tf);
					u8g2_DrawStr(&u8g2, 117, 32, "C");
					u8g2_SetFont(&u8g2, u8g2_font_fur25_tn);
					u8g2_DrawStr(&u8g2, 88, 47, buffer);
				}
			}
			else
			{
				sprintf(buffer, "%d", (uint8_t)dummy_temp_fer);
				if ((uint8_t)dummy_temp_fer >= 40 && (uint8_t)dummy_temp_fer <= 90)
				{
					u8g2_SetFont(&u8g2, Dejavu);
					u8g2_DrawStr(&u8g2, 108, 26, "o");
					u8g2_SetFont(&u8g2, u8g2_font_profont17_tf);
					u8g2_DrawStr(&u8g2, 117, 32, "F");
					u8g2_SetFont(&u8g2, u8g2_font_fur25_tn);
					u8g2_DrawStr(&u8g2, 70, 47, buffer);
				}
			}
		}
		// Get button states.
		if ((flag_button_up == true) && (button_up_clicked == false))
		{						   // up button clicked - jump to previous menu item
			button_up_clicked = 1; // set button to clicked to only perform the action once
			flag_button_up = false;
			// set_arrow_flag = true;
			if (set_flag == true && set_edit_flag == false)
			{
				switch (select_flag)
				{
				case 0:
					if (Selected_event_option > 0)
					{
						Selected_event_option -= 1;
					}
					else if (Selected_event_option == 0)
					{
						Selected_event_option = 3;
					}
					break;
				case 1:
					if (seconds > 0)
					{
						seconds -= TIMER_CHANGE_INTERVAL;
						isUpdate = true;
					}
					else if (seconds == 0)
					{
						seconds = 84600;
					}

					break;
				case 2:
					if (t_unit_flag == true)
					{
						if (dummy_temp > 0)
						{
							dummy_temp -= 1;
							isUpdate = true;
						}
						else if (dummy_temp == 0)
						{
							dummy_temp = 32;
							isUpdate = true;
						}
					}
					else
					{
						if (dummy_temp_fer > 40)
						{
							dummy_temp_fer -= 1;
							isUpdate = true;
						}
						else if (dummy_temp_fer == 40)
						{
							dummy_temp_fer = 90;
							isUpdate = true;
						}
						dummy_temp = verify_temperature_celsius((dummy_temp_fer - 32) * 5 / 9);
					}
					break;
				case 3:
					if (Selected_event_days > 0)
					{
						Selected_event_days -= 1;
					}
					else if (Selected_event_days == 0)
					{
						Selected_event_days = 2;
					}
					break;
				default:
					break;
				}
			}

			if (set_edit_flag == true)
			{
				if (select_flag > 0)
				{
					select_flag -= 1;
				}
				else if (select_flag == 0)
				{
					select_flag = 3;
				}
			}
		}
		if ((flag_button_down == true) && (button_down_clicked == 0))
		{
			flag_button_down = false;
			button_down_clicked = 1; // set button to clicked to only perform the action once

			if (set_flag == true && set_edit_flag == false)
			{
				switch (select_flag)
				{
				case 0:
					if (Selected_event_option < 3)
					{
						Selected_event_option += 1;
					}
					else if (Selected_event_option == 3)
					{
						Selected_event_option = 0;
					}
					break;
				case 1:
					if (seconds < 84600)
					{
						seconds += TIMER_CHANGE_INTERVAL;
					}
					else if (seconds == 84600)
					{
						seconds = 0;
					}
					break;
				case 2:
					if (t_unit_flag == true)
					{
						if (dummy_temp < 32)
						{
							dummy_temp += 1;
						}
						else if (dummy_temp == 32)
						{
							dummy_temp = 0;
						}
					}
					else
					{
						if (dummy_temp_fer < 90)
						{
							dummy_temp_fer += 1;
						}
						else if (dummy_temp_fer == 90)
						{
							dummy_temp_fer = 40;
						}
						dummy_temp = verify_temperature_celsius((dummy_temp_fer - 32) * 5 / 9);
					}
					break;
				case 3:
					if (Selected_event_days < 2)
					{
						Selected_event_days += 1;
					}
					else if (Selected_event_days == 2)
					{
						Selected_event_days = 0;
					}
					break;
				default:
					break;
				}
			}

			if (set_edit_flag == true)
			{
				select_flag += 1;
				if (select_flag > 3)
				{
					select_flag = 0;
				}
			}
		}
		if ((flag_button_up == false) && (button_up_clicked == 1))
		{ // unclick
			button_up_clicked = 0;
		}
		if ((flag_button_down == false) && (button_down_clicked == 1))
		{ // unclick
			button_down_clicked = 0;
		}
		if ((flag_button_select == true) && (button_select_clicked == 0))
		{
			flag_button_select = false;
			button_select_clicked = 1;
			if (set_edit_flag == false)
			{
				set_edit_flag = true;
			}
			else
			{
				set_edit_flag = false;
				set_flag = true;
				// select_flag = -1;
			}
			// current_screen--;
		}
		if ((flag_button_select == false) && (button_select_clicked == 1))
		{ // unclick
			button_select_clicked = 0;
		}
		if ((flag_button_back == true) && (button_back_clicked == 0))
		{ // select button clicked, jump between screens
			flag_button_back = false;
			button_back_clicked = 1; // set button to clicked to only perform the action once
			set_flag = false;
			for (int i = 0; i < WEEK_DAYS; i++)
			{
				gb_sch[i] = current_schedule[i];
			}

			// gb_sch[0] = current_schedule[0];
			if (isUpdate == true)
				storeScheduleData();
			current_screen--;
		}
		if ((flag_button_back == false) && (button_back_clicked == 1))
		{ // unclick
			button_back_clicked = 0;
		}

		if (Selected_event_days == 0)
		{
			for (int j = 0; j < EVENT_TYPES + 1; j++)
			{
				current_schedule[j].hour[Selected_event_option] = hour;
				current_schedule[j].minute[Selected_event_option] = minute;
				current_schedule[j].num_events = EVENT_TYPES;
				current_schedule[j].temperature[Selected_event_option] = dummy_temp;
				ESP_LOGI("SCHEDULE", "Day %d: Event %d: at %02d:%02d - Tempe: %.2f",
						 j,
						 Selected_event_option,
						 current_schedule[j].hour[Selected_event_option],
						 current_schedule[j].minute[Selected_event_option],
						 current_schedule[j].temperature[Selected_event_option]);
			}
		}
		else if (Selected_event_days == 1)
		{

			current_schedule[5].hour[Selected_event_option] = hour;
			current_schedule[5].minute[Selected_event_option] = minute;
			current_schedule[5].num_events = EVENT_TYPES;
			current_schedule[5].temperature[Selected_event_option] = dummy_temp;
			ESP_LOGI("SCHEDULE", "Day %d: Event %d: at %02d:%02d - Tempe: %.2f",
					 5,
					 Selected_event_option,
					 current_schedule[5].hour[Selected_event_option],
					 current_schedule[5].minute[Selected_event_option],
					 current_schedule[5].temperature[Selected_event_option]);
		}
		else if (Selected_event_days == 2)
		{

			current_schedule[6].hour[Selected_event_option] = hour;
			current_schedule[6].minute[Selected_event_option] = minute;
			current_schedule[6].num_events = EVENT_TYPES;
			current_schedule[6].temperature[Selected_event_option] = dummy_temp;
			ESP_LOGI("SCHEDULE", "Day %d: Event %d: at %02d:%02d - Tempe: %.2f",
					 6,
					 Selected_event_option,
					 current_schedule[6].hour[Selected_event_option],
					 current_schedule[6].minute[Selected_event_option],
					 current_schedule[6].temperature[Selected_event_option]);
		}
		blink_count++;
	}
	else
	{
		// changing the
		u8g2_DrawStr(&u8g2, 3, 12, "Set Schedule");
		u8g2_DrawStr(&u8g2, 3, 24, "From the Paired");
		u8g2_DrawStr(&u8g2, 3, 36, "App, Schedules will");
		u8g2_DrawStr(&u8g2, 3, 47, "sync with device");
		u8g2_DrawStr(&u8g2, 3, 59, "automatically.");
		// u8g2_DrawStr(&u8g2, 3, 42, "No");

		u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
		u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

		if ((flag_button_back == true) && (button_back_clicked == 0))
		{ // select button clicked, jump between screens
			flag_button_back = false;
			button_back_clicked = 1; // set button to clicked to only perform the action once
			current_screen--;
		}
		if ((flag_button_back == false) && (button_back_clicked == 1))
		{ // unclick
			button_back_clicked = 0;
		}
		vTaskDelay(5 / portTICK_PERIOD_MS);

		if ((flag_button_select == true) && (button_select_clicked == 0))
		{
			flag_button_select = false;
			button_select_clicked = 1;
			current_screen--;
		}

		if ((flag_button_select == false) && (button_select_clicked == 1))
		{ // unclick
			button_select_clicked = 0;
		}
	}
}

void show_schedul_clear()
{
	// changing the
	u8g2_DrawStr(&u8g2, 3, 12, "This Option");
	u8g2_DrawStr(&u8g2, 3, 24, "will clear all");
	u8g2_DrawStr(&u8g2, 3, 36, "Synced Schedules");
	u8g2_DrawStr(&u8g2, 3, 47, "from the App");
	u8g2_DrawStr(&u8g2, 3, 59, "want to Continue ?");
	// u8g2_DrawStr(&u8g2, 3, 42, "No");
	u8g2_DrawBitmap(&u8g2, 113, 1, 16 / 8, 13, right_icon);
	u8g2_DrawBitmap(&u8g2, 115, 52, 16 / 8, 14, back_arrow);

	if ((flag_button_back == true) && (button_back_clicked == 0))
	{ // select button clicked, jump between screens
		flag_button_back = false;
		button_back_clicked = 1; // set button to clicked to only perform the action once

		current_screen--;
	}
	if ((flag_button_back == false) && (button_back_clicked == 1))
	{ // unclick
		button_back_clicked = 0;
	}
	vTaskDelay(5 / portTICK_PERIOD_MS);

	if ((flag_button_select == true) && (button_select_clicked == 0))
	{
		flag_button_select = false;
		button_select_clicked = 1;
		current_screen--;
	}

	if ((flag_button_select == false) && (button_select_clicked == 1))
	{ // unclick
		button_select_clicked = 0;
	}
}
