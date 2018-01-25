/******************************************************************************************
 *
 * Description: Source code for single-channel LoRaWAN Gateway based on ESP8266 and SX1276
 * Version    : 0.8.1
 * Date       : 2018-01-24
 * Software   : https://github.com/SandboxElectronics/LoRaGoDOCK-Gateway
 * Hardware   : LoRaGo DOCK – http://sandboxelectronics.com/?product=lorago-dock-single-channel-lorawan-gateway
 * 
 * Copyright (c) 2016, 2017 Maarten Westenberg
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the MIT License
 * which accompanies this distribution, and is available at
 * https://opensource.org/licenses/mit-license.php
 *
 *****************************************************************************************/
 
// This file contains a number of compile-time settings that can be set on (=1) or off (=0)
// The disadvantage of compile time is minor compared to the memory gain of not having
// too much code compiled and loaded on your ESP8266.
//
// ----------------------------------------------------------------------------------------

#define VERSION "0.8.1"

// This value of DEBUG determines whether some parts of code get compiled.
// Also this is the initial value of debug parameter. 
// The value can be changed using the admin webserver
// For operational use, set initial DEBUG vaulue 0
#define DEBUG 1

// BAND can be defined as 868/915 in config.h file
#define BAND 868

// _CH is the index in the frequency array freqs[] defined in loraModem.h
#define _CH 0

// The spreading factor is the most important parameter to set for a single channel
// gateway. It specifies the speed/datarate in which the gateway and node communicate.
// As the name says, in principle the single channel gateway listens to one channel/frequency
// and to one spreading factor only.
// This parameters contains the default value of SF, the actual version can be set with
// the webserver and it will be stored in SPIFF
// NOTE: The frequency is set in the loraModem.h file and is default 868.100000 MHz.
#define _SPREADING SF8

// Channel Activity Detection
// This function will scan for valid LoRa headers and determine the Spreading 
// factor accordingly. If set to 1 we will use this function which means the 
// 1-channel gateway will become even more versatile. If set to 0 we will use the
// continuous listen mode.
// Using this function means that we HAVE to use more dio pins on the RFM95/sx1276
// device and also connect enable dio1 to detect this state. 
#define _CAD 1

// Definitions for the admin webserver.
// A_SERVER determines whether or not the admin webpage is included in the sketch.
// Normally, leave it in!
#define A_SERVER 1				// Define local WebServer only if this define is set
#define A_REFRESH 1				// Will the webserver refresh or not?
#define A_SERVERPORT 80			// local webserver port
#define A_MAXBUFSIZE 192		// Must be larger than 128, but small enough to work

// Definitions for over the air updates. At the moment we support OTA with IDE
// Make sure that you have installed Python version 2.7 and have Bonjour in your network.
// Bonjour is included in iTunes (which is free) and OTA is recommended to install 
// the firmware on your router witout having to be really close to the gateway and 
// connect with USB.
#define A_OTA 0

// Gather statistics on sensor and Wifi status
// 0= No statistics
// 1= Keep track of messages statistics, number determined by MAX_STAT
// 2= See 1 + Keep track of messages received PER SF
#define STATISTICS 2
// Maximum number of statistics records gathered. 20 is a good maximum (memory intensive)
#define MAX_STAT 20

// Single channel gateways if they behave strict should only use one frequency 
// channel and one spreading factor. However, the TTN backend replies on RX2 
// timeslot for spreading factors SF9-SF12. 
// Also, the server will respond with SF12 in the RX2 timeslot.
// If the 1ch gateway is working in and for nodes that ONLY transmit and receive on the set
// and agreed frequency and spreading factor. make sure to set STRICT to 1.
// In this case, the frequency and spreading factor for downlink messages is adapted by this
// gateway
// NOTE: If your node has only one frequency enabled and one SF, you must set this to 1
//		in order to receive downlink messages
#define _STRICT_1CH	0

// Allows configuration through WifiManager AP setup. Must be 0 or 1					
#define WIFIMANAGER 0

// Define the name of the accesspoint if the gateway is in accesspoint mode (is
// getting WiFi SSID and password using WiFiManager)
#define AP_NAME "LoRaGo"
#define AP_PASSWD "DOCK"


// Defines whether the gateway will also report sensor/status value on MQTT
// after all, a gateway can be a node to the system as well
// Set its LoRa address and key below in this file
// See spec. para 4.3.2
#define GATEWAYNODE 1
#define _CHECK_MIC 0


// Define whether we want to manage the gateway over UDP (next to management 
// thru webinterface).
// This will allow us to send messages over the UDP connection to manage the gateway 
// and its parameters. Sometimes the gateway is not accesible from remote, 
// in this case we would allow it to use the SERVER UDP connection to receive 
// messages as well.
// NOTE: Be aware that these messages are NOT LoRa and NOT LoRa Gateway spec compliant.
//	However that should not interfere with regular gateway operation but instead offer 
//	functions to set/reset certain parameters from remote.
#define GATEWAYMGT 0

// Name of he configfile in SPIFFs	filesystem
// In this file we store the configuration and other relevant info that should
// survive a reboot of the gateway		
#define CONFIGFILE "/gwayConfig.txt"

// Set the Server Settings (IMPORTANT)
#define _LOCUDPPORT 1700					// UDP port of gateway! Often 1700 or 1701 is used for upstream comms

// Timing
#define _PULL_INTERVAL 30					// PULL_DATA messages to server to get downstream
#define _STAT_INTERVAL 120					// Send a 'stat' message to server
#define _NTP_INTERVAL 3600					// How often doe we want time NTP synchronization
#define _WWW_INTERVAL	60					// Number of seconds before we refresh the web page

// MQTT definitions, these settings should be standard for TTN
// and need not changing
#define _TTNPORT 1700						// Standard port for TTN
#define _TTNSERVER "router.eu.thethings.network"

// If you have a second back-end server defined such as Semtech or loriot.io
// your can define _THINGPORT and _THINGSERVER with your own value.
// If not, make sure that you do not defined these.
// Port is UDP port in this program

//#define _THINGPORT 1700
//#define _THINGSERVER "yourserver.com"		// Server URL of the LoRa-udp.js handler

// Gateway Ident definitions
#define _DESCRIPTION "LoRaGo DOCK – Single-Channel LoRaWAN Gateway"
#define _EMAIL "info@sandboxelectronics.com"
#define _PLATFORM "LoRaGo DOCK"
#define _LAT 52.00
#define _LON 5.800
#define _ALT 14

// ntp
#define NTP_TIMESERVER "nl.pool.ntp.org"	// Country and region specific
#define NTP_TIMEZONES	1		// How far is our Timezone from UTC (excl daylight saving/summer time)
#define SECS_PER_HOUR	3600
#define NTP_INTR 0							// Do NTP processing with interrupts or in loop();

#if GATEWAYNODE==1
#define _DEVADDR { 0x26, 0x01, 0x15, 0x3D }
#define _APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define _NWKSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define _SENSOR_INTERVAL 300
#endif

// Define the correct radio type that you are using
#define CFG_sx1276_radio		
//#define CFG_sx1272_radio

// Serial Port speed
#define _BAUDRATE 115200					// Works for debug messages to serial momitor

// if OLED Display is connected to i2c
#define OLED 0								// Make define 1 on line if you have an OLED display connected
#if OLED==1
#define OLED_SCL 5							// GPIO5 / D1
#define OLED_SDA 4							// GPIO4 / D2
#endif


// Wifi definitions
// WPA is an array with SSID and password records. Set WPA size to number of entries in array
// When using the WiFiManager, we will overwrite the first entry with the 
// accesspoint we last connected to with WifiManager
// NOTE: Structure needs at least one (empty) entry.
//		So WPASIZE must be >= 1
struct wpas {
	char login[32];								// Maximum Buffer Size (and allocated memory)
	char passw[64];
};

// Please fill in at least ONE SSID and password from your own WiFI network
// below. This is needed to get the gateway working
// Note: DO NOT use the first and the last line of the stucture, these should be empty strings and
// the first line in te struct is reserved for WifiManager.
//
wpas wpa[] = {
	{ "", "" },								    // Reserved for WiFi Manager
    { "Sandbox", "stq18641189588wp18641189168" },
//	{ "your_router_1", "1st_password" },
	{ "", ""}									// spare line
};	

// For asserting and testing the following defines are used.
//
#if !defined(CFG_noassert)
#define ASSERT(cond) if(!(cond)) gway_failed(__FILE__, __LINE__)
#else
#define ASSERT(cond) /**/
#endif
