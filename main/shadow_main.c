/*
 * AWS IoT Device SDK for Embedded C 202103.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.
 * This File is open to edit, reuse, and publish.
 * @author @datta
 */

/* Standard includes. */
#include <assert.h>
#include <stdlib.h>
#include <string.h>

/* POSIX includes. */
#include <unistd.h>
#include <inttypes.h>

/* shadow helpers header. */
#include "shadow_helpers.h"

/* Shadow config include. */
#include "shadow_config.h"

/* SHADOW API header. */
#include "shadow.h"

/* JSON API header. */
#include "core_json.h"
#include "eheat.h"
/* Clock for timer. */
#include "clock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// params_t gb_params;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Format string representing a Shadow document with a "desired" state.
 */
#define NIGHT_LIGHT_SHADOW_REPORTED_JSON \
    "{"                                  \
    "\"state\":{"                        \
    "\"reported\":{"                     \
    "\"night_light\":"                   \
    "%s"                                 \
    "}}}"

#define PANEL_NIGHT_LIGHT_SHADOW_REPORTED_JSON \
    "{\"state\":{\"desired\": {\"night_light\": {\"state\": %d,\"intens\": %d}},\"reported\":{\"night_light\": {\"state\": %d,\"intens\": %d}}}}"

char night_light_report[200];

#define PILOT_LIGHT_SHADOW_REPORTED_JSON \
    "{"                                  \
    "\"state\":{"                        \
    "\"reported\":{"                     \
    "\"pilot_light\":"                   \
    "%s"                                 \
    "}}}"

#define PANEL_PILOT_LIGHT_SHADOW_REPORTED_JSON \
    "{\"state\":{\"desired\": {\"pilot_light\": {\"state\": %d,\"intens\": %d}},\"reported\":{\"pilot_light\": {\"state\": %d,\"intens\": %d}}}}"

char pilot_light_report[200];

#define DISPLAY_SHADOW_REPORTED_JSON \
    "{"                              \
    "\"state\":{"                    \
    "\"reported\":{"                 \
    "\"display\":"                   \
    "%s"                             \
    "}}}"

#define PANEL_DISPLAY_SHADOW_REPORTED_JSON \
    "{\"state\":{\"desired\": {\"display\": {\"state\": %d,\"intens\": %d,\"intens_type\": %d}},\"reported\":{\"pilot_light\": {\"state\": %d,\"intens\": %d,\"intens_type\": %d}}}}"
char display_report[200];

#define HEATER_SHADOW_REPORTED_JSON \
    "{"                             \
    "\"state\":{"                   \
    "\"reported\":{"                \
    "\"heater\":"                   \
    "%s"                            \
    "}}}"

#define PANEL_HEATER_SHADOW_REPORTED_JSON \
    "{\"state\":{\"desired\": {\"heater\": {\"state\": %d,\"s_temp\": %.2f,\"setfrom\": %d}},\"reported\":{\"heater\": {\"state\": %d,\"s_temp\": %.2f,\"setfrom\": %d}}}}"

char heater_report[350];

#define TIMER_SHADOW_REPORTED_JSON \
    "{"                            \
    "\"state\":{"                  \
    "\"reported\":{"               \
    "\"timer\":"                   \
    "%s"                           \
    "}}}"

#define PANEL_TIMER_SHADOW_REPORTED_JSON \
    "{\"state\":{\"desired\": {\"timer\": {\"state\": %d,\"t_value\": %d,\"t_temp\": %.2f,\"setfrom\": %d,\"s_time\": %lld}},\"reported\":{\"timer\": {\"state\": %d,\"t_value\": %d,\"t_temp\": %.2f,\"setfrom\": %d,\"s_time\": %lld}}}}"

char timer_report[250];

static char delete_response[50];
bool delete_dev_flag = false;
bool default_mode = true;
bool nonshadowhandler_firstcall = true;

static char child_lock_response[100];

bool child_lock_msg_flag = false;
bool panelStateChanged_child_lock = false;

#define CHILD_LOCK_REPORT_JSON \
    "{"                        \
    "\"state\":{"              \
    "\"reported\":{"           \
    "\"c_lock\":%01d"          \
    "}}}"
#define PANEL_CHILD_LOCK_REPORT_JSON \
    "{\"state\":{\"desired\": {\"c_lock\": %d},\"reported\": {\"c_lock\": %d}}}"

static char op_mode_response[100];
bool op_mode_msg_flag = false;
bool panelStateChanged_op_mode = false;
#define OP_MODE_REPORT_JSON \
    "{"                     \
    "\"state\":{"           \
    "\"reported\":{"        \
    "\"op_mode\":%01d"      \
    "}}}"

#define PANEL_OP_MODE_REPORT_JSON \
    "{\"state\":{\"desired\": {\"op_mode\": %d},\"reported\": {\"op_mode\": %d}}}"

static char hold_response[100];
bool hold_msg_flag = false;
bool panelStateChanged_hold = false;
#define HOLD_REPORT_JSON \
    "{"                  \
    "\"state\":{"        \
    "\"reported\":{"     \
    "\"is_hold\":%01d"   \
    "}}}"

#define PANEL_HOLD_REPORT_JSON \
    "{\"state\":{\"desired\": {\"is_hold\": %d},\"reported\": {\"is_hold\": %d}}}"

char freeze_protect_response[100];
bool freeze_protect_msg_flag = false;
bool panelStateChanged_freeze_lock = false;
#define FREEZE_PROTECT_REPORT_JSON \
    "{"                            \
    "\"state\":{"                  \
    "\"reported\":{"               \
    "\"f_prot\":%01d"              \
    "}}}"

#define PANEL_FREEZE_PROTECT_REPORT_JSON \
    "{\"state\":{\"desired\": {\"f_prot\": %d},\"reported\": {\"f_prot\": %d}}}"

char t_unit_protect_response[100];
bool t_unit_protect_msg_flag = false;
bool panelStateChanged_t_unit = false;
#define T_UNIT_PROTECT_REPORT_JSON \
    "{"                            \
    "\"state\":{"                  \
    "\"reported\":{"               \
    "\"t_unit\":%01d"              \
    "}}}"

#define PANEL_T_UNIT_PROTECT_REPORT_JSON \
    "{\"state\":{\"desired\": {\"t_unit\": %d},\"reported\": {\"t_unit\": %d}}}"

#define HEALTH_JSON          \
    "{"                      \
    "\"rssi\":%01d"          \
    ","                      \
    "\"a_temp\":%.f"         \
    ","                      \
    "\"f_ver\":\"%d.%d.%d\"" \
    "}"

#define HEALTH_JSON_LENGTH (sizeof(HEALTH_JSON))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------*/
params_t params;
uint8_t gb_freeze_protection_flag = false;
int lastload = true;

/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged_night_light = false;
bool panelStateChanged_night_light = false;

/**
 * @brief The flag to indicate the device current power on state changed.
 */
bool stateChanged_timer = false;
bool panelStateChanged_timer = false;

/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged_pilot_light = false;
bool panelStateChanged_pilot_light = false;

/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged_display = false;
bool panelStateChanged_display = false;
/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged_heater = false;
bool panelStateChanged_heater = false;
/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged_health = false;
/*-----------------------------------------------------------*/

/**
 * @brief This code uses the MQTT library of the AWS IoT Device SDK for
 * Embedded C. This is the prototype of the callback function defined by
 * that library. It will be invoked whenever the MQTT library receives an
 * incoming message.
 *
 * @param[in] pMqttContext MQTT context pointer.
 * @param[in] pPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] pDeserializedInfo Deserialized information from the incoming packet.
 */
static void eventCallback(MQTTContext_t *pMqttContext,
                          MQTTPacketInfo_t *pPacketInfo,
                          MQTTDeserializedInfo_t *pDeserializedInfo);

/**
 * @brief Process payload from /update/delta topic.
 *
 * This handler examines the version number and the powerOn state. If powerOn
 * state has changed, it sets a flag for the main function to take further actions.
 *
 * @param[in] pPublishInfo Deserialized publish info pointer for the incoming
 * packet.
 */
void updateDeltaHandler(MQTTPublishInfo_t *pPublishInfo);

/**
 * @brief Process payload from `/cmd/ehaet/thing_name/rsp` topic.
 *
 * This handler examines the rejected message to look for the reject reason code.
 * If the reject reason code is `404`, an attempt was made to delete a shadow
 * document which was not present yet. This is considered to be success for this
 * application.
 *
 * @param[in] pPublishInfo Deserialized publish info pointer for the incoming
 * packet.
 */
static void nonShadowMessageHandler(MQTTPublishInfo_t *pPublishInfo);

/*-----------------------------------------------------------*/

void updateDeltaHandler(MQTTPublishInfo_t *pPublishInfo)
{
    static uint32_t currentVersion = 0; /* Remember the latestVersion # we've ever received */
    uint32_t version = 0U;
    char *outValue = NULL;
    uint32_t outValueLength = 0U;
    JSONStatus_t result = JSONSuccess;

    assert(pPublishInfo != NULL);
    assert(pPublishInfo->pPayload != NULL);

    if (display_timeout_flag == true)
    {
        ESP_LOGI("DISPLAY TOGGLE", "In Updatedeltahandler");
        display_timeout_flag = false;
        disp_prev_time = esp_timer_get_time() / 1000000;
    }
    // update the state of the device with rssi and ambient temperature.

    LogInfo(("%s/update/delta json payload:%s.", SHADOW_NAME, (const char *)pPublishInfo->pPayload));

    /* Make sure the payload is a valid json document. */
    result = JSON_Validate((const char *)pPublishInfo->pPayload, pPublishInfo->payloadLength);

    if (result == JSONSuccess)
    {
        /* Then we start to get the version value by JSON keyword "version". */
        result = JSON_Search((char *)pPublishInfo->pPayload,
                             pPublishInfo->payloadLength,
                             "version",
                             sizeof("version") - 1,
                             &outValue,
                             (size_t *)&outValueLength);
    }
    else
    {
        LogError(("The json document is invalid!!"));
    }

    if (result == JSONSuccess)
    {
        LogInfo(("version: %.*s", (int)outValueLength, outValue));

        /* Convert the extracted value to an unsigned integer value. */
        version = strtoul(outValue, NULL, 10);
    }
    else
    {
        LogError(("No version in json document!!"));
    }

    LogInfo(("version:%" PRIu32 ", currentVersion:%" PRIu32 " \r\n", version, currentVersion));

    /* When the version is much newer than the on we retained, that means the powerOn
     * state is valid for us. */
    if (version > currentVersion)
    {
        /* Set to received version as the current version. */
        currentVersion = version;
        /* Open NVS to store received update status*/
        nvs_handle_t my_handle;

        esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
        if (err != ESP_OK)
        {
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        }

        cJSON const *root = cJSON_Parse(pPublishInfo->pPayload);
        cJSON const *state = cJSON_GetObjectItem(root, "state");
        cJSON const *night_light = cJSON_GetObjectItem(state, "night_light");

        if (night_light != NULL)
        {
            cJSON const *STATE = cJSON_GetObjectItem(night_light, "state");
            if (STATE != NULL)
            {
                params.night_light.state = STATE->valueint;
                nvs_set_i32(my_handle, "nl_state", params.night_light.state);
                gb_params.night_light.state = params.night_light.state;
            }

            cJSON const *INTENS = cJSON_GetObjectItem(night_light, "intens");
            if (INTENS != NULL)
            {
                params.night_light.intensity = INTENS->valueint;
                nvs_set_i32(my_handle, "nl_intens", params.night_light.intensity);
                gb_params.night_light.intensity = params.night_light.intensity;
            }

            cJSON const *night_light_color = cJSON_GetObjectItem(night_light, "color");
            if (night_light_color != NULL)
            {
                cJSON const *RED = cJSON_GetObjectItem(night_light_color, "r");
                if (RED != NULL)
                {
                    // ESP_LOGI("MESSAGE", "Night Light color - RED %d", RED->valueint);
                    params.night_light.color.red = RED->valueint;
                    gb_params.night_light.color.red = params.night_light.color.red;
                    nvs_set_i32(my_handle, "nl_red", params.night_light.color.red);
                }

                cJSON const *GREEN = cJSON_GetObjectItem(night_light_color, "g");
                if (GREEN != NULL)
                {
                    // ESP_LOGI("MESSAGE", "Night Light color - GREEN %d", GREEN->valueint);
                    params.night_light.color.green = GREEN->valueint;
                    gb_params.night_light.color.green = params.night_light.color.green;
                    nvs_set_i32(my_handle, "nl_green", params.night_light.color.green);
                }

                cJSON const *BLUE = cJSON_GetObjectItem(night_light_color, "b");
                if (BLUE != NULL)
                {
                    params.night_light.color.blue = BLUE->valueint;
                    gb_params.night_light.color.blue = params.night_light.color.blue;
                    nvs_set_i32(my_handle, "nl_blue", params.night_light.color.blue);
                }
            }
            sprintf(night_light_report, NIGHT_LIGHT_SHADOW_REPORTED_JSON, cJSON_Print(night_light));
            stateChanged_night_light = true;

            LogInfo(("Night Light State: %d\tR : %d\tG : %d\t B : %d\t Intensity : %d", params.night_light.state,
                     params.night_light.color.red, params.night_light.color.green, params.night_light.color.blue, params.night_light.intensity));

            ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            ESP_LOGI("Eheat", "Committing updates in NVS ... ");

            set_color(&gb_params);
        }

        cJSON const *pilot = cJSON_GetObjectItem(state, "pilot_light");
        if (pilot != NULL)
        {
            cJSON const *pilot_state = cJSON_GetObjectItem(pilot, "state");
            if (pilot_state != NULL)
            {
                params.pilot_light.state = pilot_state->valueint;
                nvs_set_i32(my_handle, "pl_state", params.pilot_light.state);
                gb_params.pilot_light.state = params.pilot_light.state;
            }

            cJSON const *pilot_intensity = cJSON_GetObjectItem(pilot, "intens");
            if (pilot_intensity != NULL)
            {
                params.pilot_light.intensity = pilot_intensity->valueint;
                nvs_set_i32(my_handle, "pl_intens", params.pilot_light.intensity);
                gb_params.pilot_light.intensity = params.pilot_light.intensity;
            }

            sprintf(pilot_light_report, PILOT_LIGHT_SHADOW_REPORTED_JSON, cJSON_Print(pilot));
            stateChanged_pilot_light = true;
            LogInfo(("Pilot Light State: %d\tIntensity : %d", params.pilot_light.state, params.pilot_light.intensity));
            set_pilot_light(&gb_params);
        }

        cJSON const *display = cJSON_GetObjectItem(state, "display");
        if (display != NULL)
        {
            cJSON const *display_state = cJSON_GetObjectItem(display, "state");
            if (display_state != NULL)
            {
                params.display.state = (uint8_t)display_state->valueint;
                nvs_set_i32(my_handle, "disp_state", params.display.state);
                gb_params.display.state = params.display.state;
            }

            cJSON const *display_intensty = cJSON_GetObjectItem(display, "intens");
            if (display_intensty != NULL)
            {
                params.display.intensity = (uint8_t)display_intensty->valueint;
                nvs_set_i32(my_handle, "disp_intens", params.display.intensity);
                gb_params.display.intensity = params.display.intensity;
            }

            cJSON const *display_intensty_type = cJSON_GetObjectItem(display, "intens_type");
            if (display_intensty_type != NULL)
            {
                params.display.intens_type = (uint8_t)display_intensty_type->valueint;
                nvs_set_i32(my_handle, "disp_int_type", params.display.intens_type);
                gb_params.display.intens_type = params.display.intens_type;
            }

            cJSON const *display_t_out = cJSON_GetObjectItem(display, "t_out");
            if (display_t_out != NULL)
            {
                params.display.t_out = (uint8_t)display_t_out->valueint;
                nvs_set_i32(my_handle, "disp_tout", params.display.t_out);
                gb_params.display.t_out = params.display.t_out;
            }

            sprintf(display_report, DISPLAY_SHADOW_REPORTED_JSON, cJSON_Print(display));
            stateChanged_display = true;
            set_display(&gb_params);
        }

        // Fetch the op_mode
        cJSON const *op_mode = cJSON_GetObjectItem(state, "op_mode");

        // Fetch the permanent hold flag
        cJSON const *hold = cJSON_GetObjectItem(state, "is_hold");

        // Fetch the heater object
        cJSON const *heater = cJSON_GetObjectItem(state, "heater");

        // Fetch the Timer object
        cJSON const *timer = cJSON_GetObjectItem(state, "timer");

        // Check for Hold value. If false, all modes will work as expected
        // If true, the device will go to auto mode and no other mode will be making any changes
        // If any schedule or geofence occurs, these will be stored until hold is disabled again
        // if hold is true and timer was running, stop the timer.
        if (hold != NULL)
        {
            gb_params.hold = (uint8_t)hold->valueint;
            sprintf(hold_response, HOLD_REPORT_JSON, hold->valueint);
            if (gb_params.hold == 1)
            {
                gb_params.auto_heater.s_temp = gb_params.heater.s_temp;
                gb_params.auto_heater.s_temp_fer = gb_params.heater.s_temp_fer;
                if (operation_mode == TIMER_MODE)
                {
                    timer_stop_call();
                }
                else if (gb_params.op_mode != AUTO_MODE)
                {
                    prev_operation_mode = gb_params.op_mode;
                    operation_mode = AUTO_MODE;
                    gb_params.op_mode = operation_mode;
                    panelStateChanged_op_mode = true;
                }
            }
            hold_msg_flag = true;
        }

        // If heater object is present
        if (heater != NULL)
        {
            // Check for state keyword in heater object
            cJSON const *heater_state = cJSON_GetObjectItem(heater, "state");

            // If heater state is present, update the state and status
            if (heater_state != NULL)
            {
                params.heater.state = (uint8_t)heater_state->valueint;
                if (params.heater.state == 1)
                {
                    heat_on_freeze_protect = false;
                }
                gb_params.auto_heater.state = params.heater.state;
                gb_params.schedule_heater.state = params.heater.state;
                gb_params.geofence_heater.state = params.heater.state;
                nvs_set_i32(my_handle, "heater_state", params.heater.state);
                heater_value = 0;
                if (gb_params.heater.state != params.heater.state)
                {
                    heater_state_changed = true;
                }
                heater_on_flag = params.heater.state;
                // If the heater is on due to freeze protection, change the flag value
                if (params.heater.state)
                {
                    heat_on_freeze_protect = OFF;
                }
                else
                {
                    // If timer was started, stop it
                    if (tim_timer_mode_started)
                    {
                        timer_stop_call();
                    }
                }
                // ESP_LOGI("HEATER STATE", "State %d", params.heater.state);
            }

            // Check for the s_temp in heater object. Calculate temperature if present
            cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
            if (heater_stemp != NULL)
            {
                params.heater.s_temp = (float)heater_stemp->valuedouble;
                nvs_set_i32(my_handle, "s_temp", params.heater.s_temp);
                params.heater.s_temp_fer = round(((params.heater.s_temp * 9) / 5 + 32));
            }

            // Preparing reporting packet for device shadow
            sprintf(heater_report, HEATER_SHADOW_REPORTED_JSON, cJSON_Print(heater));
            stateChanged_heater = true;
        }

        int received_opmode = 0;
        // Check if op_mode is present in the received payload
        // If present, update the variables and set accordingly
        if (op_mode != NULL)
        {
            received_opmode = op_mode->valueint;

            // If Current running mode is Timer mode and heater state is on,
            // then hold the values from other modes and update to the previous mode
            if ((operation_mode == TIMER_MODE) && (heater != NULL))
            {
                if (gb_params.heater.state == ON)
                {
                    if (received_opmode != operation_mode)
                    {
                        prev_operation_mode = (uint8_t)received_opmode;
                    }
                }
                else
                {
                    prev_operation_mode = operation_mode;
                    operation_mode = (uint8_t)received_opmode;
                    gb_params.op_mode = operation_mode;
                }
            }

            if (gb_params.hold == 1)
            {
                if (received_opmode != AUTO_MODE)
                {
                    prev_operation_mode = (uint8_t)received_opmode;
                }
            }
            else if ((operation_mode != TIMER_MODE) && (received_opmode != operation_mode))
            {
                prev_operation_mode = operation_mode;
                operation_mode = (uint8_t)received_opmode;
                gb_params.op_mode = operation_mode;
            }
            sprintf(op_mode_response, OP_MODE_REPORT_JSON, received_opmode);
            op_mode_msg_flag = true;
        }
        else
        {
            received_opmode = operation_mode;
        }

        if ((received_opmode != TIMER_MODE) && (heater != NULL))
        {
            // If the heater object is present, check for s_temp keywords.
            if (received_opmode == AUTO_MODE)
            {
                // Fetch the s_temp value if present in the payload
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.auto_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.auto_heater.s_temp_fer = round(((gb_params.auto_heater.s_temp * 9) / 5 + 32));
                }
                set_temperature(&gb_params);
            }

            if (received_opmode == GEOFENCE_MODE)
            {
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.geofence_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.geofence_heater.s_temp_fer = round(((gb_params.geofence_heater.s_temp * 9) / 5 + 32));
                }
                set_temperature(&gb_params);
            }

            if (received_opmode == SCHEDULE_MODE)
            {
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.schedule_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.schedule_heater.s_temp_fer = round(((gb_params.schedule_heater.s_temp * 9) / 5 + 32));
                }
                gb_schedule_flag = true;
                set_temperature(&gb_params);
            }
            stateChanged_heater = true;
        }

        if (timer != NULL)
        {
            cJSON const *timer_state = cJSON_GetObjectItem(timer, "state");
            if (timer_state != NULL)
            {
                gb_params.heat_timer.state = (uint8_t)timer_state->valueint;
                nvs_set_i32(my_handle, "t_state", gb_params.heat_timer.state);
                if (gb_params.heat_timer.state == 1)
                {
                    heat_on_freeze_protect = false;
                }
            }

            cJSON const *timer_t_value = cJSON_GetObjectItem(timer, "t_value"); // t_value is the timer duration
            if (timer_t_value != NULL && gb_params.heat_timer.state != false)
            {
                gb_params.heat_timer.t_value = (uint8_t)timer_t_value->valueint;
                nvs_set_i32(my_handle, "t_value", gb_params.heat_timer.t_value);
            }

            cJSON const *timer_s_temp = cJSON_GetObjectItem(timer, "t_temp"); // set temperature for timer
            if (timer_s_temp != NULL)
            {
                if (timer_s_temp->valuedouble != 0)
                {
                    gb_params.heat_timer.t_temp = (float)timer_s_temp->valuedouble;
                    nvs_set_i32(my_handle, "t_temp", gb_params.heat_timer.t_temp);
                }
            }

            cJSON const *timer_action = cJSON_GetObjectItem(timer, "t_action"); // not in use as of now
            if (timer_action != NULL)
            {
                gb_params.heat_timer.t_action = (uint8_t)timer_action->valueint;
                nvs_set_i32(my_handle, "t_action", gb_params.heat_timer.t_action);
            }
            cJSON const *timer_s_time = cJSON_GetObjectItem(timer, "s_time"); // timestamp when timer was triggered from the app
            if (timer_s_time != NULL)
            {
                gb_params.heat_timer.s_time = timer_s_time->valueint;
                nvs_set_i32(my_handle, "s_time", gb_params.heat_timer.s_time);
            }
            sprintf(timer_report, TIMER_SHADOW_REPORTED_JSON, cJSON_Print(timer));
            stateChanged_timer = true;

            set_timer(&gb_params);
        }

        cJSON const *c_lock = cJSON_GetObjectItem(state, "c_lock");
        if (c_lock != NULL)
        {
            // ESP_LOGI("MESSAGE", "child lock Settings %d", c_lock->valueint);
            params.c_lock = (uint8_t)c_lock->valueint;
            gb_params.c_lock = params.c_lock;
            nvs_set_i32(my_handle, "c_lock", params.c_lock);
            sprintf(child_lock_response, CHILD_LOCK_REPORT_JSON, c_lock->valueint);
            child_lock_msg_flag = true;
            control_child_lock(&gb_params);
        }

        cJSON const *f_prot = cJSON_GetObjectItem(state, "f_prot");
        if (f_prot != NULL)
        {
            sprintf(freeze_protect_response, FREEZE_PROTECT_REPORT_JSON, f_prot->valueint);
            gb_freeze_protection_flag = (uint8_t)f_prot->valueint;
            nvs_set_i32(my_handle, "f_prot", params.c_lock);
            freeze_protect_msg_flag = true;
            // ESP_LOGI("MESSAGE", "freeze protection Settings %d", gb_freeze_protection_flag);
        }

        cJSON const *t_unit = cJSON_GetObjectItem(state, "t_unit");
        if (t_unit != NULL)
        {
            // params.f_protection = f_prot->valueint;`
            sprintf(t_unit_protect_response, T_UNIT_PROTECT_REPORT_JSON, t_unit->valueint);
            gb_params.t_unit = (uint8_t)t_unit->valueint;
            t_unit_flag = t_unit->valueint;
            nvs_set_i32(my_handle, "t_unit", gb_params.t_unit);
            t_unit_protect_msg_flag = true;
            refresh_temperature = (ROOM_TEMP_REFRESH_SEC * 10) - 10;
        }

        size_t heapUsedAtStart = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI("MESSAGE", "Heap free at start %d.\n", heapUsedAtStart);

        nvs_commit(my_handle);
        nvs_close(my_handle);

        cJSON_Delete(root);
        save_curr_values(&gb_params);

        heapUsedAtStart = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI("MESSAGE", "Heap free at end %d.\n", heapUsedAtStart);
    }
    else
    {
        ESP_LOGI("SHADOW", "The received version is smaller than current one");
    }
}

/*-----------------------------------------------------------*/

/* This is the callback function invoked by the MQTT stack when it receives
 * incoming messages. This function demonstrates how to use the Shadow_MatchTopicString
 * function to determine whether the incoming message is a device shadow message
 * or not. If it is, it handles the message depending on the message type.
 */

void eventCallback(MQTTContext_t *pMqttContext,
                   MQTTPacketInfo_t *pPacketInfo,
                   MQTTDeserializedInfo_t *pDeserializedInfo)
{
    ShadowMessageType_t messageType = ShadowMessageTypeMaxNum;
    const char *pThingName = NULL;
    uint8_t thingNameLength = 0U;
    const char *pShadowName = NULL;
    uint8_t shadowNameLength = 0U;
    uint16_t packetIdentifier;

    (void)pMqttContext;

    assert(pDeserializedInfo != NULL);
    assert(pMqttContext != NULL);
    assert(pPacketInfo != NULL);

    packetIdentifier = pDeserializedInfo->packetIdentifier;

    /* Handle incoming publish. The lower 4 bits of the publish packet
     * type is used for the dup, QoS, and retain flags. Hence masking
     * out the lower bits to check if the packet is publish. */

    if ((pPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
    {
        assert(pDeserializedInfo->pPublishInfo != NULL);
        if (SHADOW_SUCCESS == Shadow_MatchTopicString(pDeserializedInfo->pPublishInfo->pTopicName,
                                                      pDeserializedInfo->pPublishInfo->topicNameLength,
                                                      &messageType,
                                                      &pThingName,
                                                      &thingNameLength,
                                                      &pShadowName,
                                                      &shadowNameLength))
        {
            /* Upon successful return, the messageType has been filled in. */
            if (messageType == ShadowMessageTypeUpdateDelta)
            {
                /* Handler function to process payload. */
                updateDeltaHandler(pDeserializedInfo->pPublishInfo);
            }
            else if (messageType == ShadowMessageTypeUpdateAccepted)
            {
                /* Handler function to process payload. */
                // updateAcceptedHandler(pDeserializedInfo->pPublishInfo);
            }
            else if (messageType == ShadowMessageTypeUpdateDocuments)
            {
                LogInfo(("/update/documents json payload:%s.", (const char *)pDeserializedInfo->pPublishInfo->pPayload));
            }
            else if (messageType == ShadowMessageTypeUpdateRejected)
            {
                LogInfo(("/update/rejected json payload:%s.", (const char *)pDeserializedInfo->pPublishInfo->pPayload));
            }
            else if (messageType == ShadowMessageTypeDeleteAccepted)
            {
                LogInfo(("Received an MQTT incoming publish on /delete/accepted topic."));
                // shadowDeleted = true;
                // deleteResponseReceived = true;
            }
            else if (messageType == ShadowMessageTypeGetAccepted)
            {
                /* Handler function to process payload. */
                nonShadowMessageHandler(pDeserializedInfo->pPublishInfo);
            }
            else
            {
                LogInfo(("Other message type:%d !!", messageType));
                // LogInfo(("Other Message Type : Message: %s", pDeserializedInfo->pPublishInfo->pPayload));
                nonShadowMessageHandler(pDeserializedInfo->pPublishInfo);
            }
        }
        else
        {
            // recievig the messages for deleting the device and factory device.
            // LogInfo(("Custm Topic Message ------ :%s", pDeserializedInfo->pPublishInfo->pTopicName));

            // { get_health = 1}
            // {del_devic : 1, del_ts = 1}
            // ESP_LOGI("MESSAGE", "-------------- %s", pDeserializedInfo->pPublishInfo->pPayload);
            cJSON const *message = cJSON_Parse(pDeserializedInfo->pPublishInfo->pPayload);
            if (message != NULL)
            {
                cJSON const *health_req = cJSON_GetObjectItem(message, "get_health");
                if (health_req != NULL)
                {
                    ESP_LOGI("MESSAGE", "-------------- %s", pDeserializedInfo->pPublishInfo->pPayload);
                    if (health_req->valueint == true)
                        stateChanged_health = true;
                    else
                        stateChanged_health = false;
                }
                cJSON const *delete_req = cJSON_GetObjectItem(message, "del_device");
                if (delete_req != NULL)
                {
                    ESP_LOGI("MESSAGE", "-------------- %s", pDeserializedInfo->pPublishInfo->pPayload);
                    memcpy(delete_response, pDeserializedInfo->pPublishInfo->pPayload, pDeserializedInfo->pPublishInfo->payloadLength);
                    if (delete_req->valueint == true)
                        delete_dev_flag = true;
                    else
                        delete_dev_flag = false;
                }
                cJSON const *events = cJSON_GetObjectItem(message, "events");
                if (events != NULL)
                {
                    // ESP_LOGI("MESSAGE", "-------------- %s", pDeserializedInfo->pPublishInfo->pPayload);
                    //  char *message = malloc(pDeserializedInfo->pPublishInfo->pPayload);
                    //  mem
                    // process_schedule(pDeserializedInfo->pPublishInfo->pPayload);

                    // cJSON const *events = cJSON_GetObjectItem(root, "events");
                    if (events != NULL && cJSON_IsObject(events))
                    {
                        for (int i = 0; i < WEEK_DAYS; i++)
                        {
                            char object_key[2];
                            sprintf(object_key, "%d", i);

                            cJSON const *array = cJSON_GetObjectItem(events, object_key);
                            if (array != NULL && cJSON_IsArray(array))
                            {
                                // printf("Week Day %d:\n", i);
                                int array_size = cJSON_GetArraySize(array);
                                for (int j = 0; j < array_size; j++)
                                {
                                    // printf("Array Num %d\t", j);
                                    cJSON const *item = cJSON_GetArrayItem(array, j);
                                    if (item != NULL && cJSON_IsObject(item))
                                    {
                                        cJSON const *hour = cJSON_GetObjectItem(item, "h");
                                        cJSON const *minute = cJSON_GetObjectItem(item, "m");
                                        cJSON const *temperature = cJSON_GetObjectItem(item, "t");
                                        if (hour != NULL && minute != NULL && temperature != NULL)
                                        {
                                            int timezone_offset = getTimezoneOffset(timezone);
                                            addTime(&hour->valueint, &minute->valueint, timezone_offset / 60, timezone_offset % 60);

                                            gb_sch[i].hour[j] = hour->valueint;
                                            gb_sch[i].minute[j] = minute->valueint;
                                            gb_sch[i].temperature[j] = (float)temperature->valuedouble;
                                            gb_sch[i].num_events = j + 1;
                                            // printf("Hour: %d, Minute: %d, Temperature: %.2f\n", hour->valueint, minute->valueint, temperature->valuedouble);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    cJSON_Delete(message);
                    storeScheduleData();
                    get_schedule_data();

                } // event loop
            }
        }
        memset(pDeserializedInfo->pPublishInfo->pPayload, 0, pDeserializedInfo->pPublishInfo->payloadLength);
    }
    else
    {
        HandleOtherIncomingPacket(pPacketInfo, packetIdentifier);
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Entry point of shadow.
 *
 */
void aws_iot_shadow_main(void)
{
    int returnStatus = EXIT_SUCCESS;
    int firstConnected = EXIT_SUCCESS;
    // $aws/things/envi-c08445/shadow/name/dev/get
    get_mac_id(&mac_id);
    static int ping = 0;
    char shadow_topic_delta[55];
    sprintf(&shadow_topic_delta, "%s%s%s%s%s%s", SHADOW_PREFIX, mac_id, SHADOW_NAMED_ROOT, envo, SHADOW_OP_UPDATE, SHADOW_SUFFIX_DELTA);
    ESP_LOGI("Eheat", "TOPIC %s %d", shadow_topic_delta, strlen(shadow_topic_delta));
    ESP_LOGI("Eheat", "-------------------------MAC  %s", mac_id);
    // $aws/things/envi-abcdee/shadow/get/accepted
    char shadow_topic_get_accepted[55];
    sprintf(&shadow_topic_get_accepted, "%s%s%s%s%s%s", SHADOW_PREFIX, mac_id, SHADOW_NAMED_ROOT, envo, SHADOW_OP_GET, SHADOW_SUFFIX_ACCEPTED);
    ESP_LOGI("Eheat", "TOPIC %s %d", shadow_topic_get_accepted, strlen(shadow_topic_get_accepted));
    // $aws/things/envi-abcdee/shadow/get
    char shadow_topic_get[55];
    sprintf(&shadow_topic_get, "%s%s%s%s%s", SHADOW_PREFIX, mac_id, SHADOW_NAMED_ROOT, envo, SHADOW_OP_GET);
    ESP_LOGI("Eheat", "TOPIC %s %d", shadow_topic_get, strlen(shadow_topic_get));

    char shadow_topic_update[55];
    sprintf(&shadow_topic_update, "%s%s%s%s%s", SHADOW_PREFIX, mac_id, SHADOW_NAMED_ROOT, envo, SHADOW_OP_UPDATE);
    ESP_LOGI("Eheat", "TOPIC %s %d", shadow_topic_update, strlen(shadow_topic_update));
    // cmd/eheat/envi-0a0a3a/dev/req
    char custom_topic_request[50];
    sprintf(&custom_topic_request, "/cmd/eheat/%s/%s/req", envo, mac_id);
    ESP_LOGI("Eheat", "TOPIC %s %d", custom_topic_request, strlen(custom_topic_request));

    char custom_topic_response[50];
    sprintf(&custom_topic_response, "/cmd/eheat/%s/%s/res", envo, mac_id);
    ESP_LOGI("Eheat", "TOPIC %s %d", custom_topic_response, strlen(custom_topic_response));

    char custom_topic_schedule[50];
    sprintf(&custom_topic_schedule, "/cmd/eheat/%s/%s/sch", envo, mac_id);
    ESP_LOGI("Eheat", "TOPIC %s %d", custom_topic_schedule, strlen(custom_topic_schedule));

    if (wifi_connect_state == true && net_connection_flag == false)
    {
        returnStatus = EstablishMqttSession(eventCallback);
        returnStatus = SubscribeToTopic(shadow_topic_delta, strlen(shadow_topic_delta));
        returnStatus = SubscribeToTopic(custom_topic_request, strlen(custom_topic_request));
        returnStatus = SubscribeToTopic(custom_topic_schedule, strlen(custom_topic_schedule));
        if (returnStatus == EXIT_SUCCESS)
        {
            firstConnected = EXIT_SUCCESS;
        }
        else
        {
            firstConnected = EXIT_FAILURE;
        }
    }

    while (1)
    {
        if (wifi_connect_state == true && net_connection_flag == true)
        {
            if (returnStatus == EXIT_FAILURE)
            {
                if (firstConnected == EXIT_SUCCESS)
                {
                    returnStatus = DisconnectMqttSession();
                }
                ESP_LOGI("state", "MQTTloopProcess Timeout,Conecting to MQTT..");
                returnStatus = EstablishMqttSession(eventCallback);
                returnStatus = SubscribeToTopic(shadow_topic_delta, strlen(shadow_topic_delta));
                returnStatus = SubscribeToTopic(custom_topic_request, strlen(custom_topic_request));
                returnStatus = SubscribeToTopic(custom_topic_schedule, strlen(custom_topic_schedule));
            }

            if (returnStatus == EXIT_SUCCESS && firstConnected == EXIT_SUCCESS)
            {
                if (lastload == true)
                {
                    char updateDocument[3] = "{}";
                    snprintf(updateDocument, sizeof(updateDocument), "{}");
                    returnStatus = PublishToTopic(shadow_topic_get,
                                                  strlen(shadow_topic_get),
                                                  updateDocument,
                                                  (sizeof(updateDocument)));
                    returnStatus = SubscribeToTopic(shadow_topic_get_accepted, strlen(shadow_topic_get_accepted));
                    stateChanged_health = true;
                }
                ESP_LOGI("------------------------", "return %d", returnStatus);
                // if (ping % 8 == 0)
                {
                    // ping = 0;
                    waitForPacketAck_2();
                    // ping_google();
                    // if (net_connection_flag)
                    // {
                    //     ESP_LOGW("DEVICE SHADOW", "Ping successful!\n");
                    // }
                    // else
                    // {

                    //     ESP_LOGW("DEVICE SHADOW", "Ping failed.\n");
                    // }
                }

                if (stateChanged_health == true)
                {
                    stateChanged_health = false;
                    /*start of - to post the rssi as device health*/
                    char postHealth[100] = {0};
                    static int quality = 0;
                    wifi_ap_record_t ap;
                    esp_wifi_sta_get_ap_info(&ap);
                    quality = 2 * (ap.rssi + 100);
                    if (quality > 100)
                    {
                        quality = 100;
                    }

                    snprintf(postHealth,
                             HEALTH_JSON_LENGTH + 1,
                             HEALTH_JSON,
                             quality, report_ambient_temperature, APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD);

                    returnStatus = PublishToTopic(&custom_topic_response,
                                                  (strlen(custom_topic_response)),
                                                  postHealth,
                                                  strlen(postHealth));
                    /*end of - to post the rssi as device health*/
                }
                if (stateChanged_night_light == true || panelStateChanged_night_light == true)
                {
                    if (panelStateChanged_night_light == true)
                    {
                        panelStateChanged_night_light = false;
                        snprintf(night_light_report,
                                 sizeof(PANEL_NIGHT_LIGHT_SHADOW_REPORTED_JSON) + 1,
                                 PANEL_NIGHT_LIGHT_SHADOW_REPORTED_JSON,
                                 gb_params.night_light.state, gb_params.night_light.intensity,
                                 gb_params.night_light.state, gb_params.night_light.intensity);
                    }
                    else
                    {
                        stateChanged_night_light = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  night_light_report,
                                                  strlen(night_light_report));
                }
                if (stateChanged_pilot_light == true || panelStateChanged_pilot_light == true)
                {
                    if (panelStateChanged_pilot_light == true)
                    {
                        panelStateChanged_pilot_light = false;
                        snprintf(pilot_light_report,
                                 sizeof(PANEL_PILOT_LIGHT_SHADOW_REPORTED_JSON) + 1,
                                 PANEL_PILOT_LIGHT_SHADOW_REPORTED_JSON,
                                 gb_params.pilot_light.state, gb_params.pilot_light.intensity,
                                 gb_params.pilot_light.state, gb_params.pilot_light.intensity);
                    }
                    else
                    {
                        stateChanged_pilot_light = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  pilot_light_report,
                                                  strlen(pilot_light_report));
                }
                if (stateChanged_display == true || panelStateChanged_display == true)
                {
                    if (panelStateChanged_display == true)
                    {
                        panelStateChanged_display = false;
                        snprintf(display_report,
                                 sizeof(PANEL_DISPLAY_SHADOW_REPORTED_JSON) + 1,
                                 PANEL_DISPLAY_SHADOW_REPORTED_JSON,
                                 gb_params.display.state, gb_params.display.intensity, gb_params.display.intens_type,
                                 gb_params.display.state, gb_params.display.intensity, gb_params.display.intens_type);
                    }
                    else
                    {
                        stateChanged_display = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  display_report,
                                                  strlen(display_report));
                }
                if (stateChanged_heater == true || panelStateChanged_heater == true)
                {
                    stateChanged_health = true;
                    if (panelStateChanged_heater == true)
                    {
                        static int setfrom = 1;
                        panelStateChanged_heater = false;
                        if (t_unit_flag == true)
                        {
                            snprintf(heater_report,
                                     sizeof(PANEL_HEATER_SHADOW_REPORTED_JSON) + 1,
                                     PANEL_HEATER_SHADOW_REPORTED_JSON,
                                     gb_params.heater.state, gb_params.heater.s_temp, setfrom,
                                     gb_params.heater.state, gb_params.heater.s_temp, setfrom);
                        }
                        else
                        {
                            // (22°F − 32) × 5/9
                            gb_params.heater.s_temp = verify_temperature_celsius((gb_params.heater.s_temp_fer - 32) * 5 / 9);
                            snprintf(heater_report,
                                     sizeof(PANEL_HEATER_SHADOW_REPORTED_JSON) + 1,
                                     PANEL_HEATER_SHADOW_REPORTED_JSON,
                                     gb_params.heater.state, gb_params.heater.s_temp, setfrom,
                                     gb_params.heater.state, gb_params.heater.s_temp, setfrom);
                        }
                        setfrom++;
                        if (setfrom == 10)
                        {
                            setfrom = 0;
                        }
                    }
                    else
                    {
                        stateChanged_heater = false;
                    }

                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  heater_report,
                                                  strlen(heater_report));
                }
                if (stateChanged_timer == true || panelStateChanged_timer == true)
                {
                    static int setfrom = 1;
                    uint32_t elapsed_timer_sec = 0;
                    gptimer_get_raw_count(gptimer, &elapsed_timer_sec);
                    get_time_update();
                    uint64_t timer_start_epoch = current_epoch - (elapsed_timer_sec / 1000000);
                    ESP_LOGW("TIMe", "%lld", elapsed_timer_sec);

                    if (panelStateChanged_timer == true)
                    {
                        memset(timer_report, 0, sizeof(timer_report));
                        panelStateChanged_timer = false;
                        if (gb_params.heat_timer.state == OFF)
                        {
                            snprintf(timer_report,
                                     sizeof(PANEL_TIMER_SHADOW_REPORTED_JSON) + 1,
                                     PANEL_TIMER_SHADOW_REPORTED_JSON,
                                     gb_params.heat_timer.state, 0, gb_params.heat_timer.t_temp, setfrom, (uint64_t)0, // setfrom 0-> app, 1 -> panel
                                     gb_params.heat_timer.state, 0, gb_params.heat_timer.t_temp, setfrom, (uint64_t)0);
                        }
                        else
                        {
                            snprintf(timer_report,
                                     sizeof(PANEL_TIMER_SHADOW_REPORTED_JSON) + 15,
                                     PANEL_TIMER_SHADOW_REPORTED_JSON,
                                     gb_params.heat_timer.state, gb_params.heat_timer.t_value, gb_params.heat_timer.t_temp, setfrom, timer_start_epoch,
                                     gb_params.heat_timer.state, gb_params.heat_timer.t_value, gb_params.heat_timer.t_temp, setfrom, timer_start_epoch);
                        }
                        setfrom++;
                        if (setfrom == 10)
                        {
                            setfrom = 0;
                        }
                    }
                    else
                    {
                        stateChanged_timer = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  timer_report,
                                                  strlen(timer_report));
                }
                if (child_lock_msg_flag == true || panelStateChanged_child_lock == true)
                {
                    if (panelStateChanged_child_lock == true)
                    {
                        panelStateChanged_child_lock = false;
                        snprintf(child_lock_response,
                                 sizeof(PANEL_CHILD_LOCK_REPORT_JSON) + 1,
                                 PANEL_CHILD_LOCK_REPORT_JSON,
                                 gb_params.c_lock, gb_params.c_lock);
                    }
                    else
                    {
                        child_lock_msg_flag = false;
                    }

                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  child_lock_response,
                                                  strlen(child_lock_response));
                }
                if (op_mode_msg_flag == true || panelStateChanged_op_mode == true)
                {
                    if (panelStateChanged_op_mode == true)
                    {
                        panelStateChanged_op_mode = false;
                        snprintf(op_mode_response,
                                 sizeof(PANEL_OP_MODE_REPORT_JSON) + 1,
                                 PANEL_OP_MODE_REPORT_JSON,
                                 gb_params.op_mode, gb_params.op_mode);
                    }
                    else
                    {
                        op_mode_msg_flag = false;
                    }

                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  op_mode_response,
                                                  strlen(op_mode_response));
                }

                if (hold_msg_flag == true || panelStateChanged_hold == true)
                {
                    if (panelStateChanged_hold == true)
                    {
                        panelStateChanged_hold = false;
                        snprintf(hold_response,
                                 sizeof(PANEL_HOLD_REPORT_JSON) + 1,
                                 PANEL_HOLD_REPORT_JSON,
                                 gb_params.hold, gb_params.hold);
                    }
                    else
                    {
                        hold_msg_flag = false;
                    }

                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  hold_response,
                                                  strlen(hold_response));
                }

                if (freeze_protect_msg_flag == true || panelStateChanged_freeze_lock == true)
                {
                    if (panelStateChanged_freeze_lock == true)
                    {
                        panelStateChanged_freeze_lock = false;
                        snprintf(freeze_protect_response,
                                 sizeof(PANEL_FREEZE_PROTECT_REPORT_JSON) + 1,
                                 PANEL_FREEZE_PROTECT_REPORT_JSON,
                                 gb_params.f_protection, gb_params.f_protection);
                    }
                    else
                    {
                        freeze_protect_msg_flag = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  freeze_protect_response,
                                                  strlen(freeze_protect_response));
                }
                if (t_unit_protect_msg_flag == true || panelStateChanged_t_unit == true)
                {
                    if (panelStateChanged_t_unit == true)
                    {
                        panelStateChanged_t_unit = false;
                        snprintf(t_unit_protect_response,
                                 sizeof(PANEL_T_UNIT_PROTECT_REPORT_JSON) + 1,
                                 PANEL_T_UNIT_PROTECT_REPORT_JSON,
                                 gb_params.t_unit, gb_params.t_unit);
                    }
                    else
                    {
                        t_unit_protect_msg_flag = false;
                    }
                    returnStatus = PublishToTopic(shadow_topic_update,
                                                  (strlen(shadow_topic_update)),
                                                  t_unit_protect_response,
                                                  strlen(freeze_protect_response));
                }
                if (delete_dev_flag == true)
                {
                    delete_dev_flag = false;
                    returnStatus = PublishToTopic(custom_topic_response,
                                                  (strlen(custom_topic_response)),
                                                  delete_response,
                                                  strlen(delete_response));
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    if (returnStatus == EXIT_SUCCESS)
                        reset_factory_device();
                }
            }

            vTaskDelay(30 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI("------------------------", "return %d", returnStatus);
            returnStatus = EXIT_FAILURE;
            ESP_LOGE("DEVICE SHADOW", " WIFI CONNECTION DISCONNECTED.....");
            vTaskDelay(15000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/*-----------------------------------------------------------*/
/**
 * @brief Entry point of Non shadow Message Handler
 *
 */
void nonShadowMessageHandler(MQTTPublishInfo_t *pPublishInfo)
{
    static uint32_t currentVersion = 0; /* Remember the latestVersion # we've ever received */
    uint32_t version = 0U;
    char *outValue = NULL;
    uint32_t outValueLength = 0U;
    JSONStatus_t result = JSONSuccess;

    assert(pPublishInfo != NULL);
    assert(pPublishInfo->pPayload != NULL);

    LogInfo(("%s shadow/get json payload:%s.", SHADOW_NAME, (const char *)pPublishInfo->pPayload));

    /* Make sure the payload is a valid json document. */
    result = JSON_Validate((const char *)pPublishInfo->pPayload, pPublishInfo->payloadLength);

    if (result == JSONSuccess)
    {
        /* Then we start to get the version value by JSON keyword "version". */
        result = JSON_Search((char *)pPublishInfo->pPayload,
                             pPublishInfo->payloadLength,
                             "version",
                             sizeof("version") - 1,
                             &outValue,
                             (size_t *)&outValueLength);
    }
    else
    {
        LogError(("The json document is invalid!!"));
    }

    if (result == JSONSuccess)
    {
        LogInfo(("version: %.*s", (int)outValueLength, outValue));

        /* Convert the extracted value to an unsigned integer value. */
        version = strtoul(outValue, NULL, 10);
    }
    else
    {
        LogError(("No version in json document!!"));
    }

    LogInfo(("version:%" PRIu32 ", currentVersion:%" PRIu32 " \r\n", version, currentVersion));

    /* When the version is much newer than the on we retained, that means the powerOn
     * state is valid for us. */
    if (version > currentVersion)
    {
        /* Set to received version as the current version. */
        currentVersion = version;
        /* Open NVS to store received update status*/
        nvs_handle_t my_handle;

        esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
        if (err != ESP_OK)
        {
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        }

        /*======================================================================================================================================*/

        /*======================================================================================================================================*/

        cJSON const *root = cJSON_Parse(pPublishInfo->pPayload);
        cJSON const *state1 = cJSON_GetObjectItem(root, "state");
        cJSON const *state = cJSON_GetObjectItem(state1, "desired");
        lastload = false;

        cJSON const *night_light = cJSON_GetObjectItem(state, "night_light");
        if (night_light != NULL)
        {
            cJSON const *STATE = cJSON_GetObjectItem(night_light, "state");
            if (STATE != NULL)
            {
                params.night_light.state = STATE->valueint;
                nvs_set_i32(my_handle, "nl_state", params.night_light.state);
                gb_params.night_light.state = params.night_light.state;
            }

            cJSON const *INTENS = cJSON_GetObjectItem(night_light, "intens");
            if (INTENS != NULL)
            {
                params.night_light.intensity = INTENS->valueint;
                nvs_set_i32(my_handle, "nl_intens", params.night_light.intensity);
                gb_params.night_light.intensity = params.night_light.intensity;
            }

            cJSON const *night_light_color = cJSON_GetObjectItem(night_light, "color");
            if (night_light_color != NULL)
            {
                cJSON const *RED = cJSON_GetObjectItem(night_light_color, "r");
                if (RED != NULL)
                {
                    // ESP_LOGI("MESSAGE", "Night Light color - RED %d", RED->valueint);
                    params.night_light.color.red = RED->valueint;
                    gb_params.night_light.color.red = params.night_light.color.red;
                    nvs_set_i32(my_handle, "nl_red", params.night_light.color.red);
                }

                cJSON const *GREEN = cJSON_GetObjectItem(night_light_color, "g");
                if (GREEN != NULL)
                {
                    // ESP_LOGI("MESSAGE", "Night Light color - GREEN %d", GREEN->valueint);
                    params.night_light.color.green = GREEN->valueint;
                    gb_params.night_light.color.green = params.night_light.color.green;
                    nvs_set_i32(my_handle, "nl_green", params.night_light.color.green);
                }

                cJSON const *BLUE = cJSON_GetObjectItem(night_light_color, "b");
                if (BLUE != NULL)
                {
                    params.night_light.color.blue = BLUE->valueint;
                    gb_params.night_light.color.blue = params.night_light.color.blue;
                    nvs_set_i32(my_handle, "nl_blue", params.night_light.color.blue);
                }
            }
            sprintf(night_light_report, NIGHT_LIGHT_SHADOW_REPORTED_JSON, cJSON_Print(night_light));
            // stateChanged_night_light = true;

            LogInfo(("Night Light State: %d\tR : %d\tG : %d\t B : %d\t Intensity : %d", params.night_light.state,
                     params.night_light.color.red, params.night_light.color.green, params.night_light.color.blue, params.night_light.intensity));

            ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
            ESP_LOGI("Eheat", "Committing updates in NVS ... ");

            set_color(&gb_params);
        }

        cJSON const *pilot = cJSON_GetObjectItem(state, "pilot_light");
        if (pilot != NULL)
        {
            cJSON const *pilot_state = cJSON_GetObjectItem(pilot, "state");
            if (pilot_state != NULL)
            {
                params.pilot_light.state = pilot_state->valueint;
                nvs_set_i32(my_handle, "pl_state", params.pilot_light.state);
                gb_params.pilot_light.state = params.pilot_light.state;
            }

            cJSON const *pilot_intensity = cJSON_GetObjectItem(pilot, "intens");
            if (pilot_intensity != NULL)
            {
                params.pilot_light.intensity = pilot_intensity->valueint;
                nvs_set_i32(my_handle, "pl_intens", params.pilot_light.intensity);
                gb_params.pilot_light.intensity = params.pilot_light.intensity;
            }

            sprintf(pilot_light_report, PILOT_LIGHT_SHADOW_REPORTED_JSON, cJSON_Print(pilot));
            // stateChanged_pilot_light = true;
            LogInfo(("Pilot Light State: %d\tIntensity : %d", params.pilot_light.state, params.pilot_light.intensity));
            set_pilot_light(&gb_params);
        }

        cJSON const *display = cJSON_GetObjectItem(state, "display");
        if (display != NULL)
        {
            cJSON const *display_state = cJSON_GetObjectItem(display, "state");
            if (display_state != NULL)
            {
                params.display.state = (uint8_t)display_state->valueint;
                nvs_set_i32(my_handle, "disp_state", params.display.state);
                gb_params.display.state = params.display.state;
            }

            cJSON const *display_intensty = cJSON_GetObjectItem(display, "intens");
            if (display_intensty != NULL)
            {
                params.display.intensity = (uint8_t)display_intensty->valueint;
                nvs_set_i32(my_handle, "disp_intens", params.display.intensity);
                gb_params.display.intensity = params.display.intensity;
            }

            cJSON const *display_intensty_type = cJSON_GetObjectItem(display, "intens_type");
            if (display_intensty_type != NULL)
            {
                params.display.intens_type = (uint8_t)display_intensty_type->valueint;
                nvs_set_i32(my_handle, "disp_int_type", params.display.intens_type);
                gb_params.display.intens_type = params.display.intens_type;
            }

            cJSON const *display_t_out = cJSON_GetObjectItem(display, "t_out");
            if (display_t_out != NULL)
            {
                params.display.t_out = (uint8_t)display_t_out->valueint;
                nvs_set_i32(my_handle, "disp_tout", params.display.t_out);
                gb_params.display.t_out = params.display.t_out;
            }

            sprintf(display_report, DISPLAY_SHADOW_REPORTED_JSON, cJSON_Print(display));
            // stateChanged_display = true;
            set_display(&gb_params);
        }

        // Fetch the op_mode
        cJSON const *op_mode = cJSON_GetObjectItem(state, "op_mode");

        // Fetch the permanent hold flag
        cJSON const *hold = cJSON_GetObjectItem(state, "is_hold");

        // Fetch the heater object
        cJSON const *heater = cJSON_GetObjectItem(state, "heater");

        // Fetch the Timer object
        cJSON const *timer = cJSON_GetObjectItem(state, "timer");

        // Check for Hold value. If false, all modes will work as expected
        // If true, the device will go to auto mode and no other mode will be making any changes
        // If any schedule or geofence occurs, these will be stored until hold is disabled again
        // if hold is true and timer was running, stop the timer.
        if (hold != NULL)
        {
            gb_params.hold = (uint8_t)hold->valueint;

            sprintf(hold_response, HOLD_REPORT_JSON, hold->valueint);
            if (gb_params.hold == 1)
            {
                if (nonshadowhandler_firstcall == true)
                {
                    operation_mode = AUTO_MODE;
                    gb_params.op_mode = AUTO_MODE;
                    nonshadowhandler_firstcall = false;
                }
                else
                {
                    gb_params.auto_heater.s_temp = gb_params.heater.s_temp;
                    gb_params.auto_heater.s_temp_fer = gb_params.heater.s_temp_fer;
                    if (operation_mode == TIMER_MODE)
                    {
                        timer_stop_call();
                    }
                    else if (gb_params.op_mode != AUTO_MODE)
                    {
                        prev_operation_mode = gb_params.op_mode;
                        operation_mode = AUTO_MODE;
                        gb_params.op_mode = operation_mode;
                        panelStateChanged_op_mode = true;
                    }
                }
            }
        }

        // If heater object is present
        if (heater != NULL)
        {
            // Check for state keyword in heater object
            cJSON const *heater_state = cJSON_GetObjectItem(heater, "state");

            // If heater state is present, update the state and status
            if (heater_state != NULL)
            {
                params.heater.state = (uint8_t)heater_state->valueint;
                if (params.heater.state == 1)
                {
                    heat_on_freeze_protect = false;
                }
                gb_params.auto_heater.state = params.heater.state;
                gb_params.schedule_heater.state = params.heater.state;
                gb_params.geofence_heater.state = params.heater.state;
                nvs_set_i32(my_handle, "heater_state", params.heater.state);
                heater_value = 0;
                if (gb_params.heater.state != params.heater.state)
                {
                    heater_state_changed = true;
                }
                heater_on_flag = params.heater.state;
                // If the heater is on due to freeze protection, change the flag value
                if (params.heater.state)
                {
                    heat_on_freeze_protect = OFF;
                }
                else
                {
                    // If timer was started, stop it
                    if (tim_timer_mode_started)
                    {
                        timer_stop_call();
                    }
                }
            }

            // Check for the s_temp in heater object. Calculate temperature if present
            cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
            if (heater_stemp != NULL)
            {
                params.heater.s_temp = (float)heater_stemp->valuedouble;
                gb_params.auto_heater.s_temp = (float)heater_stemp->valuedouble;
                nvs_set_i32(my_handle, "s_temp", params.heater.s_temp);
                params.heater.s_temp_fer = round(((params.heater.s_temp * 9) / 5 + 32));
            }

            // Preparing reporting packet for device shadow
            sprintf(heater_report, HEATER_SHADOW_REPORTED_JSON, cJSON_Print(heater));
        }

        int received_opmode = 0;
        // Check if op_mode is present in the received payload
        // If present, update the variables and set accordingly
        if (op_mode != NULL)
        {
            received_opmode = op_mode->valueint;

            // If Current running mode is Timer mode and heater state is on,
            // then hold the values from other modes and update to the previous mode
            if ((operation_mode == TIMER_MODE) && (heater != NULL))
            {
                if (params.heater.state == ON)
                {
                    if (received_opmode != operation_mode)
                    {
                        prev_operation_mode = (uint8_t)received_opmode;
                    }
                }
                else
                {
                    prev_operation_mode = operation_mode;
                    operation_mode = (uint8_t)received_opmode;
                    gb_params.op_mode = operation_mode;
                }
            }
            if (gb_params.hold == 1)
            {
                if (received_opmode != AUTO_MODE)
                {
                    prev_operation_mode = (uint8_t)received_opmode;
                }
            }
            else if ((operation_mode != TIMER_MODE) && (received_opmode != operation_mode))
            {
                prev_operation_mode = operation_mode;
                operation_mode = (uint8_t)received_opmode;
                gb_params.op_mode = operation_mode;
            }
            sprintf(op_mode_response, OP_MODE_REPORT_JSON, received_opmode);
        }
        else
        {
            received_opmode = operation_mode;
        }

        if ((received_opmode != TIMER_MODE) && (heater != NULL))
        {
            // If the heater object is present, check for s_temp keywords.
            if (received_opmode == AUTO_MODE)
            {
                // Fetch the s_temp value if present in the payload
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.auto_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.auto_heater.s_temp_fer = round(((gb_params.auto_heater.s_temp * 9) / 5 + 32));
                }
                set_temperature(&gb_params);
            }

            if (received_opmode == GEOFENCE_MODE)
            {
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.geofence_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.geofence_heater.s_temp_fer = round(((gb_params.geofence_heater.s_temp * 9) / 5 + 32));
                }
                set_temperature(&gb_params);
            }

            if (received_opmode == SCHEDULE_MODE)
            {
                cJSON const *heater_stemp = cJSON_GetObjectItem(heater, "s_temp");
                if (heater_stemp != NULL)
                {
                    gb_params.schedule_heater.s_temp = (float)heater_stemp->valuedouble;
                    gb_params.schedule_heater.s_temp_fer = round(((gb_params.schedule_heater.s_temp * 9) / 5 + 32));
                }
                gb_schedule_flag = true;
                set_temperature(&gb_params);
            }
            stateChanged_heater = true;
        }

        if (timer != NULL)
        {
            cJSON const *timer_state = cJSON_GetObjectItem(timer, "state");
            if (timer_state != NULL)
            {
                gb_params.heat_timer.state = (uint8_t)timer_state->valueint;
                nvs_set_i32(my_handle, "t_state", gb_params.heat_timer.state);
                if (gb_params.heat_timer.state == 1)
                {
                    heat_on_freeze_protect = false;
                }
            }

            cJSON const *timer_t_value = cJSON_GetObjectItem(timer, "t_value"); // t_value is the timer duration
            if (timer_t_value != NULL && gb_params.heat_timer.state != false)
            {
                gb_params.heat_timer.t_value = (uint8_t)timer_t_value->valueint;
                nvs_set_i32(my_handle, "t_value", gb_params.heat_timer.t_value);
            }

            cJSON const *timer_s_temp = cJSON_GetObjectItem(timer, "t_temp"); // set temperature for timer
            if (timer_s_temp != NULL)
            {
                if (timer_s_temp->valuedouble != 0)
                {
                    gb_params.heat_timer.t_temp = (float)timer_s_temp->valuedouble;
                    nvs_set_i32(my_handle, "t_temp", gb_params.heat_timer.t_temp);
                }
            }

            cJSON const *timer_action = cJSON_GetObjectItem(timer, "t_action"); // not in use as of now
            if (timer_action != NULL)
            {
                gb_params.heat_timer.t_action = (uint8_t)timer_action->valueint;
                nvs_set_i32(my_handle, "t_action", gb_params.heat_timer.t_action);
            }
            cJSON const *timer_s_time = cJSON_GetObjectItem(timer, "s_time"); // timestamp when timer was triggered from the app
            if (timer_s_time != NULL)
            {
                gb_params.heat_timer.s_time = timer_s_time->valueint;
                nvs_set_i32(my_handle, "s_time", gb_params.heat_timer.s_time);
            }
            sprintf(timer_report, TIMER_SHADOW_REPORTED_JSON, cJSON_Print(timer));
            // stateChanged_timer = true;

            set_timer(&gb_params);
        }

        cJSON const *c_lock = cJSON_GetObjectItem(state, "c_lock");
        if (c_lock != NULL)
        {
            params.c_lock = (uint8_t)c_lock->valueint;
            gb_params.c_lock = params.c_lock;
            nvs_set_i32(my_handle, "c_lock", params.c_lock);
            sprintf(child_lock_response, CHILD_LOCK_REPORT_JSON, c_lock->valueint);
            // child_lock_msg_flag = true;
            control_child_lock(&gb_params);
        }

        cJSON const *f_prot = cJSON_GetObjectItem(state, "f_prot");
        if (f_prot != NULL)
        {
            sprintf(freeze_protect_response, FREEZE_PROTECT_REPORT_JSON, f_prot->valueint);
            gb_freeze_protection_flag = (uint8_t)f_prot->valueint;
            nvs_set_i32(my_handle, "f_prot", params.c_lock);
        }

        cJSON const *t_unit = cJSON_GetObjectItem(state, "t_unit");
        if (t_unit != NULL)
        {
            sprintf(t_unit_protect_response, T_UNIT_PROTECT_REPORT_JSON, t_unit->valueint);
            gb_params.t_unit = (uint8_t)t_unit->valueint;
            t_unit_flag = t_unit->valueint;
            nvs_set_i32(my_handle, "t_unit", gb_params.t_unit);
            refresh_temperature = (ROOM_TEMP_REFRESH_SEC * 10) - 10;
        }

        size_t heapUsedAtStart = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI("MESSAGE", "Heap free at start %d.\n", heapUsedAtStart);

        nvs_commit(my_handle);
        nvs_close(my_handle);
        cJSON_Delete(root);

        heapUsedAtStart = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI("MESSAGE", "Heap free at start %d.\n", heapUsedAtStart);
    }
    else
    {
        LogWarn(("The received version is smaller than current one!!"));
    }
}

/*-----------------------------------------------------------*/
void reset_factory_device(void)
{
    // Find the Partition
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);
    // Erase entire partition
    ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));
    // Erase entire partition
    const esp_partition_t *partition1 = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "certs");
    assert(partition1 != NULL);
    ESP_ERROR_CHECK(esp_partition_erase_range(partition1, 0, partition1->size));

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("nvs", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI("Eheat", "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    // storing the display info into the nvs
    err = nvs_set_i32(my_handle, "disp_intens", 0);
    err = nvs_set_i32(my_handle, "disp_state", 0);
    err = nvs_set_i32(my_handle, "disp_tout", 0);
    // storing the pilot light setting into the NVS
    err = nvs_set_i32(my_handle, "pl_intens", 0);
    err = nvs_set_i32(my_handle, "pl_state", 0);
    // storing the night light setting into
    err = nvs_set_i32(my_handle, "nl_state", 0);
    err = nvs_set_i32(my_handle, "nl_intens", 0);
    err = nvs_set_i32(my_handle, "nl_itype", 0);
    err = nvs_set_i32(my_handle, "nl_red", 0);
    err = nvs_set_i32(my_handle, "nl_green", 0);
    err = nvs_set_i32(my_handle, "nl_blue", 0);

    err = nvs_set_i32(my_handle, "s_temp", 0);
    err = nvs_set_i32(my_handle, "s_temp_fer", 0);
    err = nvs_set_i32(my_handle, "heater_state", 0);
    err = nvs_set_i32(my_handle, "t_unit", 0);
    err = nvs_set_i32(my_handle, "blufi_state", 0);

    ESP_LOGI("Eheat", "%s", (err != ESP_OK) ? "Failed!\n" : "Done\n");
    ESP_LOGI("Eheat", "resseting the values in NVS ... ");

    err = nvs_commit(my_handle);
    nvs_close(my_handle);

    err = esp_wifi_restore();
    if (err == ESP_OK)
    {
        ESP_LOGW("Eheat", "Wi-Fi credentials cleared successfully.\n");
    }
    else
    {
        ESP_LOGW("Eheat", "Failed to clear Wi-Fi credentials. Error code: %d\n", err);
    }

    // Close
    esp_restart();
}