#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "matrix_drive.h"
//#include SSID_H
#include "pendulum.h"
#include "settings.h"
#include "wifi.h"


/*

	
*/

static String ap_name;
static String ap_pass;
static ip_addr_settings_t ip_addr_settings;

static void wifi_init_settings();
void wifi_setup()
{
	// check settings fs
	wifi_init_settings();

	// first, disconnect wifi
	WiFi.mode(WIFI_OFF);
	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	WiFi.setAutoReconnect(true);

	// try to connect
	WiFi.mode(WIFI_STA);
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	wifi_start();
}

extern "C" {
uint8 wifi_get_channel(void);
}

//! watch wifi channel and ser EMI reduction mode
static void wifi_check_proc()
{
	static pendulum_t pendulum(&wifi_check_proc, 2); // check even if the Arduino main loop is not working 

	static int last_chann = -1;
	int now_chann = wifi_get_channel();
	if(last_chann != now_chann)
	{
		last_chann = now_chann;

		led_set_interval_mode_from_channel(now_chann);
	}
}


static int wifi_last_status = -1;
void wifi_check()
{
	// check wifi status
	wifi_check_proc();

	// output to console
	int st = WiFi.status();
	if(wifi_last_status != st)
	{
		wifi_last_status = st;
		Serial.print('\r');
		Serial.print('\n');
		String m = wifi_get_connection_info_string();
		Serial.print(m.c_str());
		Serial.print('\r');
		Serial.print('\n');
	}
}

ip_addr_settings_t::ip_addr_settings_t()
{
	clear();
}

void ip_addr_settings_t::clear()
{
	ip_addr_settings.ip_addr =
	ip_addr_settings.ip_gateway =
	ip_addr_settings.ip_mask =
	ip_addr_settings.dns1 =
	ip_addr_settings.dns2 = F("0.0.0.0");
}


/**
 * Begins WPS configuration.
 * This function does not return until WPS config ends (either successfully or not)
 */
void wifi_wps()
{
	auto old_mode = led_get_interval_mode();
	led_set_interval_mode(LIM_PWM_OFF); // stop PWM clock to reduce EMI during WPS
	WiFi.beginWPSConfig();
	led_set_interval_mode(old_mode);

	// read WPS configuration regardless of WPS success or not
	ap_name = WiFi.SSID();
	ap_pass = WiFi.psk();

	// WPS client should be have automatic DHCP
	ip_addr_settings.clear();

	// write current configuration
	wifi_write_settings();
}

/**
 * Start WiFi connection using configured parameters
 */
void wifi_start()
{
	WiFi.mode(WIFI_OFF);
	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	WiFi.setAutoReconnect(true);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ap_name.c_str(), ap_pass.c_str());

	IPAddress i_ip_addr;    i_ip_addr   .fromString(ip_addr_settings.ip_addr);
	IPAddress i_ip_gateway; i_ip_gateway.fromString(ip_addr_settings.ip_gateway);
	IPAddress i_ip_mask;    i_ip_mask   .fromString(ip_addr_settings.ip_mask);
	IPAddress i_dns1;       i_dns1      .fromString(ip_addr_settings.dns1);
	IPAddress i_dns2;       i_dns2      .fromString(ip_addr_settings.dns2);

	WiFi.config(i_ip_addr, i_ip_gateway, i_ip_mask, i_dns1, i_dns2);
}

/**
 * Read settings. Initialize settings to factory state, if the settings key is invalid
 */
static void wifi_init_settings()
{
	settings_write(F("ap_name"), F(""), SETTINGS_NO_OVERWRITE);
	settings_write(F("ap_pass"), F(""), SETTINGS_NO_OVERWRITE);
	settings_write(F("ip_addr"), F("0.0.0.0"), SETTINGS_NO_OVERWRITE); // automatic ip configuration
	settings_write(F("ip_gateway"), F("0.0.0.0"), SETTINGS_NO_OVERWRITE);
	settings_write(F("ip_mask"), F("0.0.0.0"), SETTINGS_NO_OVERWRITE);
	settings_write(F("dns_1"), F("0.0.0.0"), SETTINGS_NO_OVERWRITE);
	settings_write(F("dns_2"), F("0.0.0.0"), SETTINGS_NO_OVERWRITE);

	settings_read(F("ap_name"), ap_name);
	settings_read(F("ap_pass"), ap_pass);
	settings_read(F("ip_addr"),    ip_addr_settings.ip_addr);
	settings_read(F("ip_gateway"), ip_addr_settings.ip_gateway);
	settings_read(F("ip_mask"),    ip_addr_settings.ip_mask);
	settings_read(F("dns_1"),      ip_addr_settings.dns1);
	settings_read(F("dns_2"),      ip_addr_settings.dns1);
}

/**
 * Write settings
 */
void wifi_write_settings()
{
	settings_write(F("ap_name"), ap_name);
	settings_write(F("ap_pass"), ap_pass);
	settings_write(F("ip_addr"),    ip_addr_settings.ip_addr);
	settings_write(F("ip_gateway"), ip_addr_settings.ip_gateway);
	settings_write(F("ip_mask"),    ip_addr_settings.ip_mask);
	settings_write(F("dns_1"),      ip_addr_settings.dns1);
	settings_write(F("dns_2"),      ip_addr_settings.dns1);
}

const String & wifi_get_ap_name()
{
	return ap_name;
}

const String & wifi_get_ap_pass()
{
	return ap_pass;
}

ip_addr_settings_t wifi_get_ip_addr_settings(bool use_current_config)
{
	// if use_current_config is true, returns current connection information
	// instead of manually configured settings
	if(!use_current_config)
		return ip_addr_settings;

	if(WiFi.status() != WL_CONNECTED)
		return ip_addr_settings;

	ip_addr_settings_t s;

	s.ip_addr = WiFi.localIP().toString();
	s.ip_gateway = WiFi.gatewayIP().toString();
	s.ip_mask = WiFi.subnetMask().toString();
	s.dns1 = WiFi.dnsIP(0).toString();
	s.dns2 = WiFi.dnsIP(1).toString();
	return s;
}

void wifi_set_ap_info(const String &_ap_name, const String &_ap_pass)
{
	ap_name = _ap_name;
	ap_pass = _ap_pass;

	wifi_write_settings();
	wifi_start();
}

void wifi_set_ap_info(const String &_ap_name, const String &_ap_pass,
	const ip_addr_settings_t & ip)
{
	ap_name = _ap_name;
	ap_pass = _ap_pass;

	wifi_manual_ip_info(ip);
}

void wifi_manual_ip_info(const ip_addr_settings_t & ip)
{
	ip_addr_settings = ip;

	wifi_write_settings();

	wifi_start();
}

String wifi_get_connection_info_string()
{
	String m;
	switch(WiFi.status())
	{
	case WL_CONNECTED:
		m = String(F("Connected to \"")) + wifi_get_ap_name() + F("\"");
		if((uint32_t)WiFi.localIP() == 0)
			m += String(F(", but no IP address got."));
		else
			m += String(F(", IP address is ")) + WiFi.localIP().toString() + F(" .");
		break;

	case WL_NO_SSID_AVAIL:
		m = String(F("AP \"")) + wifi_get_ap_name() + F("\" not available.");
		break;

	case WL_CONNECT_FAILED:
		m = String(F("Connection to \"")) + wifi_get_ap_name() + F("\" failed.");
		break;

	case WL_IDLE_STATUS: // ?? WTF ??
		m = String(F("Connection to \"")) + wifi_get_ap_name() + F("\" is idling.");
		break;

	case WL_DISCONNECTED:
		m = String(F("Disconnected from \"")) + wifi_get_ap_name() + F("\".");
		break;
	}
	return m;
}

