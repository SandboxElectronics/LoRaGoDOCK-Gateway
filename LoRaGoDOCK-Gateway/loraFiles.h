/******************************************************************************************
 *
 * Description: Source code for single-channel LoRaWAN Gateway based on ESP8266 and SX1276
 * Version    : 0.8.2
 * Date       : 2018-11-26
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



// Definition of the configuration record that is read at startup and written
// when settings are changed.
struct espGwayConfig {
	uint16_t fcnt;				// =0 as init value	XXX Could be 32 bit in size
	uint16_t boots;				// Number of restarts made by the gateway after reset
	uint16_t resets;			// Number of statistics resets
	uint16_t views;				// Number of sendWebPage() calls
	uint16_t wifis;				// Number of WiFi Setups
	uint16_t reents;			// Number of re-entrant interrupt handler calls
	uint16_t ntpErr;			// Number of UTP requests that failed
	uint16_t ntps;
	
	uint8_t ch;					// index to freqs array, freqs[ifreq]=868100000 default
	uint8_t sf;					// range from SF7 to SF12
	uint8_t debug;				// range 0 to 4
	
	bool cad;					// is CAD enabled?
	bool hop;					// Is HOP enabled (Note: SHould be disabled)
	bool node;					// Is gateway node enabled
	bool refresh;				// Is WWW browser refresh enabled
	
	String ssid;				// SSID of the last connected WiFi Network
	String pass;				// Password
} gwayConfig;


