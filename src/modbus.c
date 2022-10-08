/*
 *  Open HR20
 *
 *  target:     ATmega169 @ 4 MHz in Honnywell Rondostat HR20E
 *
 *  compiler:   WinAVR-20071221
 *              avr-libc 1.6.0
 *              GCC 4.2.2
 *
 *  copyright:  2008 Jiri Dobry (jdobry-at-centrum-dot-cz)
 *
 *  license:    This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU Library General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later version.
 *
 *              This program is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              GNU General Public License for more details.
 *
 *              You should have received a copy of the GNU General Public License
 *              along with this program. If not, see http:*www.gnu.org/licenses
 */

/*!
 * \file       com.c
 * \brief      comunication
 * \author     Jiri Dobry <jdobry-at-centrum-dot-cz>
 * \date       $Date$
 * $Rev$
 */

#if defined(MODBUS) && MODBUS == 1

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/wdt.h>


#include "config.h"
#include "modbus.h"
#include "common/avr_uart_i.h"
#include "main.h"
#include "common/rtc.h"
#include "adc.h"
#include "task.h"
#include "watch.h"
#include "eeprom.h"
#include "controller.h"
#include "menu.h"
#include "common/wireless.h"
#include "debug.h"


enum modbus_function_codes {
    MODBUS_FUNC_GET_VERSION,
    MODBUS_FUNC_GET_FEATURES,
    MODBUS_FUNC_GET_DEBUG,
    MODBUS_FUNC_GET_CURRENT_TEMPERATURE,
    MODBUS_FUNC_SET_VALVE,
    MODBUS_FUNC_GET_INTERFACE_VALUES,
    MODBUS_FUNC_GET_EVENTS,
    MODBUS_FUNC_SET_TARGET_TEMPERATURE,
    MODBUS_FUNC_SET_VALVE_ON_TEMPERATURE,
    MODBUS_FUNC_SET_VALVE_OFF_TEMPERATURE,
    MODBUS_FUNC_SET_TIME,
    MODBUS_FUNC_SET_DATE,
    MODBUS_FUNC_SET_SCHEDULE_TIMES,
    MODBUS_FUNC_SET_SCHEDULE_TEMPERATURES,
    MODBUS_FUNC_GET_TEMPERATURE_HISTORY,
    MODBUS_FUNC_REBOOT,
    MODBUS_FUNC_FWUPDATE,
};

struct __attribute__((packed)) modbus_get_version_request_data {

};

struct __attribute__((packed)) modbus_get_version_reply_data {

};



struct __attribute__((packed)) modbus_get_debug_request_data {

};

struct __attribute__((packed)) modbus_get_debug_reply_data {

};



struct __attribute__((packed)) modbus_get_current_temperature_request_data {

};

struct __attribute__((packed)) modbus_get_current_temperature_reply_data {

};



struct __attribute__((packed)) modbus_set_valve_request_data {

};

struct __attribute__((packed)) modbus_set_valve_reply_data {

};



struct __attribute__((packed)) modbus_get_interface_values_request_data {

};

struct __attribute__((packed)) modbus_get_interface_values_reply_data {

};



struct __attribute__((packed)) modbus_get_events_request_data {

};

struct __attribute__((packed)) modbus_get_events_reply_data {

};



struct __attribute__((packed)) modbus_set_target_temperature_request_data {

};

struct __attribute__((packed)) modbus_set_target_temperature_reply_data {

};



struct __attribute__((packed)) modbus_set_valve_on_temperature_request_data {

};

struct __attribute__((packed)) modbus_set_valve_on_temperature_reply_data {

};



struct __attribute__((packed)) modbus_set_valve_off_temperature_request_data {

};

struct __attribute__((packed)) modbus_set_valve_off_temperature_reply_data {

};



struct __attribute__((packed)) modbus_set_time_request_data {

};

struct __attribute__((packed)) modbus_set_time_reply_data {

};



struct __attribute__((packed)) modbus_set_date_request_data {

};

struct __attribute__((packed)) modbus_set_date_reply_data {

};



struct __attribute__((packed)) modbus_set_schedule_times_request_data {

};

struct __attribute__((packed)) modbus_set_schedule_times_reply_data {

};



struct __attribute__((packed)) modbus_set_schedule_temperatures_request_data {

};

struct __attribute__((packed)) modbus_set_schedule_temperatures_reply_data {

};



struct __attribute__((packed)) modbus_get_temperature_history_request_data {

};

struct __attribute__((packed)) modbus_get_temperature_history_reply_data {

};



struct __attribute__((packed)) modbus_reboot_request_data {

};

struct __attribute__((packed)) modbus_reboot_reply_data {

};



struct __attribute__((packed)) modbus_fwupdate_request_data {

};

struct __attribute__((packed)) modbus_fwupdate_reply_data {

};


union __attribute__((packed)) modbus_device_request_data {
    struct modbus_get_version_request_data get_version;
    struct modbus_get_debug_request_data get_debug;
    struct modbus_get_current_temperature_request_data get_current_temperature;
    struct modbus_set_valve_request_data set_valve;
    struct modbus_get_interface_values_request_data get_interface_values;
    struct modbus_get_events_request_data get_events;
    struct modbus_set_target_temperature_request_data set_target_temperature;
    struct modbus_set_valve_on_temperature_request_data set_valve_on_temperature;
    struct modbus_set_valve_off_temperature_request_data set_valve_off_temperature;
    struct modbus_set_time_request_data set_time;
    struct modbus_set_date_request_data set_date;
    struct modbus_set_schedule_times_request_data set_schedule_times;
    struct modbus_set_schedule_temperatures_request_data set_schedule_temperatures;
    struct modbus_get_temperature_history_request_data get_temperature_history;
    struct modbus_reboot_request_data reboot;
    struct modbus_fwupdate_request_data fwupdate;
};


union __attribute__((packed)) modbus_device_reply_data {
    struct modbus_get_version_reply_data get_version;
    struct modbus_get_debug_reply_data get_debug;
    struct modbus_get_current_temperature_reply_data get_current_temperature;
    struct modbus_set_valve_reply_data set_valve;
    struct modbus_get_interface_values_reply_data get_interface_values;
    struct modbus_get_events_reply_data get_events;
    struct modbus_set_target_temperature_reply_data set_target_temperature;
    struct modbus_set_valve_on_temperature_reply_data set_valve_on_temperature;
    struct modbus_set_valve_off_temperature_reply_data set_valve_off_temperature;
    struct modbus_set_time_reply_data set_time;
    struct modbus_set_date_reply_data set_date;
    struct modbus_set_schedule_times_reply_data set_schedule_times;
    struct modbus_set_schedule_temperatures_reply_data set_schedule_temperatures;
    struct modbus_get_temperature_history_reply_data get_temperature_history;
    struct modbus_reboot_reply_data reboot;
    struct modbus_fwupdate_reply_data fwupdate;
};

struct __attribute__((packed)) modbus_device_request_pdu {
    uint8_t function_code;
    union modbus_device_request_data;

};

struct __attribute__((packed)) modbus_device_reply_pdu {
    uint8_t function_code;
    union modbus_device_reply_data;

};

struct __attribute__((packed)) modbus_broadcast_pdu {
    uint8_t function_code;
};



/**/
/**/

typedef uint8_t modbus_addr_t;
typedef uint16_t modbus_checksum_t;

struct __attribute__((packed)) modbus_serial_request_pdu {
    modbus_addr_t addr;
    union {
        struct modbus_device_request_pdu dev_pdu;
        struct modbus_broadcast_pdu broadcast_pdu;
    };
    modbus_checksum_t checksum;
};

struct __attribute__((packed)) modbus_serial_reply_pdu {
    modbus_addr_t addr;
    struct modbus_device_reply_pdu dev_pdu;
    modbus_checksum_t checksum;
};


static void check_command_termination(uint8_t c) {
    if (c == '\n') {
        task |= TASK_COM;
    }
}

/*!
 *******************************************************************************
 *  \brief init communication
 *
 *  \note
 ******************************************************************************/
void modbus_init(void)
{
#ifdef COM_UART
	uart0_init();
	uart0_sethook(check_command_termination);
#endif
}




void modbus_handle_command(void)
{
}

#endif /* defined(MODBUS) && MODBUS == 1 */
