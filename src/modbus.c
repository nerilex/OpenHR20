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


#define TX_BUFF_SIZE 128
#define RX_BUFF_SIZE 32

#define ENABLE_LOCAL_COMMANDS 1



void COM_print_debug(uint8_t type);

#if RFM == 1
void COM_wireless_command_parse(uint8_t *rfm_framebuf, uint8_t rfm_framepos);
#endif

void COM_debug_print_motor(int8_t dir, uint16_t m, uint8_t pwm);
void COM_debug_print_temperature(uint16_t t);

#if DEBUG_DUMP_RFM
void COM_dump_packet(uint8_t *d, uint8_t len, bool mac_ok);
// void COM_mac_ok(void);
#else
#define COM_dump_packet(d, len, mac_ok)
// #define COM_mac_ok() ()
#endif
#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
void COM_print_time(uint8_t c);
#else
#define COM_print_time(c)
#endif

static void COM_putchar(char c);
static void COM_flush(void);
void COM_printStr16(const char *s, uint16_t x);

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

/*!
 *******************************************************************************
 *  \brief transmit bytes
 *
 *  \note
 ******************************************************************************/
static
void COM_putchar(char c)
{
	uart0_putc(c);
}


/*!
 *******************************************************************************
 *  \brief receive bytes
 *
 *  \note
 ******************************************************************************/
static
char COM_getchar(void)
{
	char c;

	c = uart0_getc();
	return c;
}

/*!
 *******************************************************************************
 *  \brief flush output buffer
 *
 *  \note
 ******************************************************************************/
static
void COM_flush(void)
{
	uart0_flush();
}


/*!
 *******************************************************************************
 *  \brief helper function print 2 digit dec number
 *
 *  \note only unsigned numbers
 ******************************************************************************/
static void print_decXX(uint8_t i)
{
	if (i >= 100)
	{
		COM_putchar(i / 100 + '0');
		i %= 100;
	}
	COM_putchar(i / 10 + '0');
	COM_putchar(i % 10 + '0');
}

/*!
 *******************************************************************************
 *  \brief helper function print 4 digit dec number
 *
 *  \note only unsigned numbers
 ******************************************************************************/
static void print_decXXXX(uint16_t i)
{
	print_decXX(i / 100);
	print_decXX(i % 100);
}

/*!
 *******************************************************************************
 *  \brief helper function print 2 digit dec number
 *
 *  \note only unsigned numbers
 ******************************************************************************/
static void print_hexXX(uint8_t i)
{
	uint8_t x = i >> 4;

	if (x >= 10)
	{
		COM_putchar(x + 'a' - 10);
	}
	else
	{
		COM_putchar(x + '0');
	}
	x = i & 0xf;
	if (x >= 10)
	{
		COM_putchar(x + 'a' - 10);
	}
	else
	{
		COM_putchar(x + '0');
	}
}

/*!
 *******************************************************************************
 *  \brief helper function print 4 digit dec number
 *
 *  \note
 ******************************************************************************/
static void print_hexXXXX(uint16_t i)
{
	print_hexXX(i >> 8);
	print_hexXX(i & 0xff);
}

/*!
 *******************************************************************************
 *  \brief helper function print string without \n2 digit dec number
 *
 *  \note
 ******************************************************************************/
static void print_s_p(const char *s)
{
	char c;

	for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
	{
		COM_putchar(c);
	}
}

/*!
 *******************************************************************************
 *  \brief helper function print version string
 *
 *  \note
 ******************************************************************************/
static void print_version(bool sync)
{
	const char *s = (PSTR(VERSION_STRING "\n"));

	COM_putchar('V');
	char c;
	for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
	{
		COM_putchar(c);
#if RFM == 1
		if (sync)
		{
			wireless_putchar(c);
		}
#endif
	}
}

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
    print_version(false);
	COM_flush();
}


/*!
 *******************************************************************************
 *  \brief Print debug line
 *
 *  \note
 ******************************************************************************/
void COM_print_debug(uint8_t type)
{
	print_s_p(PSTR("D: "));
	print_hexXX(RTC_GetDayOfWeek() + 0xd0);
	COM_putchar(' ');
	print_decXX(RTC_GetDay());
	COM_putchar('.');
	print_decXX(RTC_GetMonth());
	COM_putchar('.');
	print_decXX(RTC_GetYearYY());
	COM_putchar(' ');
	print_decXX(RTC_GetHour());
	COM_putchar(':');
	print_decXX(RTC_GetMinute());
	COM_putchar(':');
	print_decXX(RTC_GetSecond());
	COM_putchar(' ');
	COM_putchar((CTL_mode_auto) ? (CTL_test_auto() ? 'A' : '-') : 'M');
	print_s_p(PSTR(" V: "));
	print_decXX(valve_wanted);
	print_s_p(PSTR(" I: "));
	print_decXXXX(temp_average);
	print_s_p(PSTR(" S: "));
	if (CTL_temp_wanted_last > TEMP_MAX + 1)
	{
		print_s_p(PSTR("BOOT"));
	}
	else
	{
		print_decXXXX(calc_temp(CTL_temp_wanted_last));
	}
	print_s_p(PSTR(" B: "));
	print_decXXXX(bat_average);
#if DEBUG_PRINT_I_SUM
	print_s_p(PSTR(" Is: "));
	print_hexXXXX(sumError >> 16);
	print_hexXXXX(sumError);
	print_s_p(PSTR(" Ib: "));       //jr
	print_hexXX(CTL_integratorBlock);
	print_s_p(PSTR(" Ic: "));       //jr
	print_hexXX(CTL_interatorCredit);
	print_s_p(PSTR(" Ie: "));       //jr
	print_hexXX(CTL_creditExpiration);
#endif
	if (CTL_error != 0)
	{
		print_s_p(PSTR(" E:"));
		print_hexXX(CTL_error);
	}
	if (type > 0)
	{
		print_s_p(PSTR(" X"));
	}
	if (mode_window())
	{
		print_s_p(PSTR(" W"));
	}
	if (menu_locked)
	{
		print_s_p(PSTR(" L"));
	}
	COM_putchar('\n');
	COM_flush();
#if (RFM == 1)
	bool sync = (type == 2);
	if (!sync)
	{
		wireless_buf_ptr = 0;
		wireless_async = true;
		wireless_putchar('D');
	}
	wireless_putchar(
		RTC_GetMinute()
		| (CTL_test_auto() ? 0x40 : 0)
		| ((CTL_mode_auto) ? 0x80 : 0));
	wireless_putchar(
		RTC_GetSecond()
		| ((mode_window()) ? 0x40 : 0)
		| ((menu_locked) ? 0x80 : 0));
	wireless_putchar(CTL_error);
	wireless_putchar(temp_average >> 8);    // current temp
	wireless_putchar(temp_average & 0xff);
	wireless_putchar(bat_average >> 8);     // current temp
	wireless_putchar(bat_average & 0xff);
	wireless_putchar(CTL_temp_wanted);      // wanted temp
	wireless_putchar(valve_wanted);         // valve pos
	wireless_async = false;
	rfm_start_tx();
#endif
}

/*!
 *  \note dirty trick with shared array for \ref COM_hex_parse and \ref COM_commad_parse
 *  code size optimalization
 */
static uint8_t com_hex[3];


/*!
 *******************************************************************************
 *  \brief parse hex number (helper function)
 *
 *	\note hex numbers use ONLY lowcase chars, upcase is reserved for commands
 *
 ******************************************************************************/
static char COM_hex_parse(uint8_t n)
{
	uint8_t i;

	for (i = 0; i < n; i++)
	{
		uint8_t c = COM_getchar() - '0';
		if (c > 9)   // chars < '0' overload var c
		{
			if ((c >= ('a' - '0')) && (c <= ('f' - '0')))
			{
				c -= (('a' - '0') - 10);
			}
			else
			{
				return c + '0';
			}
		}
		if (i & 1)
		{
			com_hex[i >> 1] += c;
		}
		else
		{
			com_hex[i >> 1] = (uint8_t)c << 4;
		}
	}
	{
		char c;
		if ((c = COM_getchar()) != '\n')
		{
			return c;
		}
	}
	return '\0';
}

/*!
 *******************************************************************************
 *  \brief print X[xx]=
 *
 ******************************************************************************/
static void print_idx(char t, uint8_t i)
{
	COM_putchar(t);
	COM_putchar('[');
	print_hexXX(i);
	COM_putchar(']');
	COM_putchar('=');
}



/*!
 *******************************************************************************
 *  \brief parse command
 *
 *  \note commands have FIXED format
 *  \note command X.....\n    - X is upcase char as commad name, \n is termination char
 *  \note hex numbers use ONLY lowcase chars, upcase is reserved for commands
 *  \note   V\n - print version information
 *  \note   D\n - print status line
 *  \note   Taa\n - print watched variable aa (return 2 or 4 hex numbers) see to \ref watch.c
 *  \note   Gaa\n - get configuration byte with hex address aa see to \ref eeprom.h 0xff address returns EEPROM layout version
 *  \note   Saadd\n - set configuration byte aa to value dd (hex)
 *  \note   Rab\n - get timer for day a slot b, return cddd=(timermode c time ddd) (hex)
 *  \note   Wabcddd\n - set timer  for day a slot b timermode c time ddd (hex)
 *  \note   B1324\n - reboot, 1324 is password (fixed at this moment)
 *  \note   Yyymmdd\n - set, year yy, month mm, day dd; HEX values!!!
 *  \note   Hhhmmss\n - set, hour hh, minute mm, second ss; HEX values!!!
 *  \note   Axx\n - set wanted temperature [unit 0.5C]
 *  \note   Mxx\n - set mode and close window (00=manu 01=auto fd=nochange/close window only)
 *      \note	Lxx\n - Lock keys, and return lock status (00=unlock, 01=lock, 02=status only)
 *
 ******************************************************************************/
void modbus_handle_command(void)
{
	char c;

	while (uart0_dataavail())
	{
		switch (c = COM_getchar())
		{
		case 'V':
			if (COM_getchar() == '\n')
			{
				print_version(false);
			}
			c = '\0';
			break;
#if ENABLE_LOCAL_COMMANDS
		case 'D':
			if (COM_getchar() == '\n')
			{
				COM_print_debug(1);
			}
			c = '\0';
			break;
		case 'T':
		{
			if (COM_hex_parse(1 * 2) != '\0')
			{
				break;
			}
			print_idx(c, com_hex[0]);
			print_hexXXXX(watch(com_hex[0]));
		}
		break;
		case 'G':
		case 'S':
			if (c == 'G')
			{
				if (COM_hex_parse(1 * 2) != '\0')
				{
					break;
				}
			}
			else
			{
				if (COM_hex_parse(2 * 2) != '\0')
				{
					break;
				}
				if (com_hex[0] < CONFIG_RAW_SIZE)
				{
					config_raw[com_hex[0]] = (uint8_t)(com_hex[1]);
					eeprom_config_save(com_hex[0]);
				}
			}
			print_idx(c, com_hex[0]);
			if (com_hex[0] == 0xff)
			{
				print_hexXX(EE_LAYOUT);
			}
			else
			{
				print_hexXX(config_raw[com_hex[0]]);
			}
			break;
		case 'R':
		case 'W':
			if (c == 'R')
			{
				if (COM_hex_parse(1 * 2) != '\0')
				{
					break;
				}
			}
			else
			{
				if (COM_hex_parse(3 * 2) != '\0')
				{
					break;
				}
				RTC_DowTimerSet(
					com_hex[0] >> 4,
					com_hex[0] & 0xf,
					(((uint16_t)(com_hex[1]) & 0xf) << 8) + (uint16_t)(com_hex[2]),
					(com_hex[1]) >> 4);
				CTL_update_temp_auto();
			}
			print_idx(c, com_hex[0]);
			print_hexXXXX(eeprom_timers_read_raw(
					      timers_get_raw_index((com_hex[0] >> 4), (com_hex[0] & 0xf))));
			break;
		case 'Y':
			if (COM_hex_parse(3 * 2) != '\0')
			{
				break;
			}
			RTC_SetDate(com_hex[2], com_hex[1], com_hex[0]);
			COM_print_debug(1);
			c = '\0';
			break;
		case 'H':
			if (COM_hex_parse(3 * 2) != '\0')
			{
				break;
			}
			RTC_SetHour(com_hex[0]);
			RTC_SetMinute(com_hex[1]);
			RTC_SetSecond(com_hex[2]);
			COM_print_debug(1);
			c = '\0';
			break;
		case 'B':
		{
			if (COM_hex_parse(2 * 2) != '\0')
			{
				break;
			}
			if ((com_hex[0] == 0x13) && (com_hex[1] == 0x24))
			{
				cli();
				wdt_enable(WDTO_15MS);  //wd on,15ms
				while (1)
				{
					;               //loop till reset
				}
			}
		}
		break;
		case 'M':
			if (COM_hex_parse(1 * 2) != '\0')
			{
				break;
			}
			CTL_change_mode(com_hex[0]);
			COM_print_debug(1);
			break;
		case 'A':
			if (COM_hex_parse(1 * 2) != '\0')
			{
				break;
			}
			if (com_hex[0] < TEMP_MIN - 1)
			{
				break;
			}
			if (com_hex[0] > TEMP_MAX + 1)
			{
				break;
			}
			CTL_set_temp(com_hex[0]);
			COM_print_debug(1);
			break;
		case 'L':
			if (COM_hex_parse(1 * 2) != '\0')
			{
				break;
			}
			if (com_hex[0] <= 1)
			{
				menu_locked = com_hex[0];
			}
			print_hexXX(menu_locked);
			break;
#endif
		//case '\n':
		//case '\0':
		default:
			c = '\0';
			break;
		}
		if (c != '\0')
		{
			COM_putchar('\n');
		}
		COM_flush();
#if (RFM == 1)
		rfm_start_tx();
#endif
	}
}

#if RFM == 1
static void COM_wireless_word(uint16_t w)
{
	wireless_putchar(w >> 8);
	wireless_putchar(w & 0xff);
}

/*!
 *******************************************************************************
 *  \brief parse command from wireless
 *******************************************************************************
 */
void COM_wireless_command_parse(uint8_t *rfm_framebuf, uint8_t rfm_framepos)
{
	uint8_t pos = 0;

	while (rfm_framepos > pos)
	{
		uint8_t c = rfm_framebuf[pos++];
		wireless_putchar(c | 0x80);
		switch (c)
		{
		case 'V':
			print_version(true);
			break;
		case 'D':
			COM_print_debug(2);
			break;
		case 'T':
			wireless_putchar(rfm_framebuf[pos]);
			COM_wireless_word(watch(rfm_framebuf[pos]));
			pos++;
			break;
		case 'G':
		case 'S':
			if (c == 'S')
			{
				if (rfm_framebuf[pos] < CONFIG_RAW_SIZE)
				{
					config_raw[rfm_framebuf[pos]] = (uint8_t)(rfm_framebuf[pos + 1]);
					eeprom_config_save(rfm_framebuf[pos]);
				}
			}
			wireless_putchar(rfm_framebuf[pos]);
			if (rfm_framebuf[pos] == 0xff)
			{
				wireless_putchar(EE_LAYOUT);
			}
			else
			{
				wireless_putchar(config_raw[rfm_framebuf[pos]]);
			}
			if (c == 'S')
			{
				pos++;
			}
			pos++;
			break;
		case 'R':
		case 'W':
			if (c == 'W')
			{
				RTC_DowTimerSet(
					rfm_framebuf[pos] >> 4,
					rfm_framebuf[pos] & 0xf,
					(((uint16_t)(rfm_framebuf[pos + 1]) & 0xf) << 8) + (uint16_t)(rfm_framebuf[pos + 2]),
					(rfm_framebuf[pos + 1]) >> 4);
				CTL_update_temp_auto();
			}
			wireless_putchar(rfm_framebuf[pos]);
			COM_wireless_word(eeprom_timers_read_raw(
						  timers_get_raw_index((rfm_framebuf[pos] >> 4), (rfm_framebuf[pos] & 0xf))));
			if (c == 'W')
			{
				pos += 2;
			}
			pos++;
			break;
		case 'B':
			if ((rfm_framebuf[pos] == 0x13) && (rfm_framebuf[pos + 1] == 0x24))
			{
				reboot = true;
			}
			wireless_putchar(rfm_framebuf[pos]);
			wireless_putchar(rfm_framebuf[pos + 1]);
			pos += 2;
			break;
		case 'M':
			CTL_change_mode(rfm_framebuf[pos++]);
			COM_print_debug(2);
			break;
		case 'A':
			if (rfm_framebuf[pos] < TEMP_MIN - 1)
			{
				break;
			}
			if (rfm_framebuf[pos] > TEMP_MAX + 1)
			{
				break;
			}
			CTL_set_temp(rfm_framebuf[pos++]);
			COM_print_debug(2);
			break;
		case 'L':
			if (rfm_framebuf[pos] <= 1)
			{
				menu_locked = rfm_framebuf[pos];
			}
			wireless_putchar(menu_locked);
			pos++;
			break;
		default:
			break;
		}
	}
}
#endif

#if DEBUG_PRINT_MOTOR
void COM_debug_print_motor(int8_t dir, uint16_t m, uint8_t pwm)
{
	if (dir > 0)
	{
		COM_putchar('+');
	}
	else if (dir < 0)
	{
		COM_putchar('-');
	}
	COM_putchar(' ');
	print_hexXXXX(m);
	COM_putchar(' ');
	print_hexXX(pwm);

	COM_putchar('\n');
	COM_flush();
}
#endif

#if DEBUG_PRINT_MEASURE
void COM_debug_print_temperature(uint16_t t)
{
	print_s_p(PSTR("T: "));
	print_decXXXX(t);
	COM_putchar('\n');
	COM_flush();
}
#endif


#if DEBUG_DUMP_RFM
/*!
 *******************************************************************************
 *  \brief dump data from *d length len
 *
 *  \note
 ******************************************************************************/
static uint16_t seq = 0;
void COM_dump_packet(uint8_t *d, uint8_t len, bool mac_ok)
{
	uint8_t type;

	print_decXX((task & TASK_RTC) ? RTC_GetSecond() + 1 : RTC_GetSecond());
	COM_putchar('.');
	print_hexXX(RTC_s256);
	if (mac_ok && (len >= (2 + 4)))
	{
		print_s_p(PSTR(" PKT"));
		len -= 4; // mac is correct and not needed
	}
	else
	{
		print_s_p(PSTR(" ERR"));
	}
	print_hexXXXX(seq++);
	COM_putchar(':');
	bool dots = false;
	if (len > 10)
	{
		len = 10; // debug output limitation
		dots = true;
	}
	while ((len--) > 0)
	{
		COM_putchar(' ');
		print_hexXX(*(d++));
	}
	if (dots)
	{
		print_s_p(PSTR("..."));
	}
	COM_putchar('\n');
	COM_flush();
}
#endif

#if DEBUG_PRINT_ADDITIONAL_TIMESTAMPS
void COM_print_time(uint8_t c)
{
	print_decXX((task & TASK_RTC) ? RTC_GetSecond() + 1 : RTC_GetSecond());
	COM_putchar('.');
	print_hexXX(RTC_s256);
	COM_putchar('-');
	COM_putchar(c);
	COM_putchar('\n');
	COM_flush();
}

#endif

#if DEBUG_PRINT_STR_16
void COM_printStr16(const char *s, uint16_t x)
{
	print_s_p(s);
	print_hexXXXX(x);
	COM_putchar('\n');
}
#endif

#endif /* defined(MODBUS) && MODBUS == 1 */
