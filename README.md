# LoRaGo DOCK - Single Channel LoRaWAN Gateway based on ESP8266 and SX1276

Version : 0.8.1  
Date    : 2018-01-25  
Software: https://github.com/SandboxElectronics/LoRaGoDOCK-Gateway  
Hardware: LoRaGo DOCK – http://sandboxelectronics.com/?product=lorago-dock-single-channel-lorawan-gateway  

Originally designed by Maarten Westenberg (mw12554@hotmail.com)  
Adopted by Sandbox Electronics (http://sandboxelectronics.com)  

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

# Description

This repository contains a proof-of-concept implementation of a single channel LoRaWAN gateway for the ESP8266.
The software implements a standard LoRa gateway with the following exceptions on changes:

- This LoRa gateway is not a full gateway but it implements just a one-channel/one frequency gateway.
The minimum amount of frequencies supported by a full gateway is 3, most support 9 or more frequencies.
This software started as a proof-of-concept to prove that a single low-cost RRFM95 chip which was present in almost every
LoRa node in Europe could be used as a cheap alternative to the far more expensive full gateways that were
making use of the SX1301 chip.

- As the software of this gateway will often be used during the development phase of a project or in demo situations,
the software is flexible and can be easily configured according to environment or customer requirements.
There are two ways of interacting with the software: 1. Modifying the config.h file at compile time allows the
administrator to set almost all parameters. 2. Using the web interface (http://<gateway_IP>) will allow
administrators to set and reset several of the parameters at runtime.


# Getting Started

It is recommended to compile and start the single channel gateway with as little modificatons as possible. This means that you should use the default settings in the configuration files as much as possible and only change the SSID/Password for your WiFi setup and the pins of your ESP8266 that you use in loraModem.h. This section describes the minimum of configuration necessary to get a working gateway which than can be configured further using the webpage.

1. Unpack the source code including the libraries in a separate folder.
1. Install depending libraries. Check "Dependencies" section below.
1. Install ESP8266 board from with Boards Manager if necessary.
1. Select Board and related settings:
   - Board        : "SparkFun ESP8266 Thing Dev"
   - CPU Frequency: "80 MHz"
   - Upload Speed : "115200"
1. Connect the gateway to a serial port of your computer, and configure that port in the IDE.
1. Edit the config.h file and adapt the "wpas" structure. Make sure that the first line of this structure remains empty and put the SSID and Password of your router on the second line of the array.
1. Compile the code and doenload the executable over USB to the gateway. If all is right, you should see the gateway starting up on the Serial Monitor.
1. Note the IP address that the device receives from your router. Use that IP address in a browser on your computer to connect to the gateway with the browser.

Now your gateway should be running. Use the webpage to set "debug" to 1 and you should be able to see packages
coming in on the Serial monitor.


# Configuration

There are two ways of changing the configuration of the single channel gateway:

1. Changing the config.h file at compile-time
2. Run the http://<gateway-IP> web interface to change setting at complie time.


## Editing the config.h file

The config.h file contains all the user configurable settings. All have their definitions set through #define statements. In general, setting a #define to 1 will enable the function and setting it to 0 will disable it.

Also, some settings can be initialized by setting their value with a #define but can be changed at runtime in the web interface. For some settings, disabling the function with a #define will remove the function from the webserver as well.

NOTE regarding memory usage: The ESP8266 has an enormous amount of memory available for program space and SPIFFS file system. However the memory available for heap and variables is limited to about 80K bytes (For the ESP-32 this is higher). The user is advised to turn off functions not used in order to save on memory usage. If the heap drops below 18 KBytes some functions may not behave as expected (in extreme case the program may crash).


### Debug

The user can set the initial value of the DEBUG parameter.
Setting this parameter will also determine some settings of the webserver.

`#define DEBUG 1`


### Setting Spreading Factor

Set the `_SPREADING` factor to the desired SF7, SF8 - SF12 value.
Please note that this value is closely related to teh value used for `_CAD`.
If `_CAD` is enabled, the value of `_SPREADING` is not used by the gateway as it has all spreading factors enabled.

`#define _SPREADING SF9`

Please note that the default frequency used is 868.1 MHz which can be changed in the loraModem.h file.
The user is advised NOT to change this setting and only use the default 868.1 MHz frequency.


### Channel Activity Detection

Channel Activity Detection (CAD) is a function of the LoRa RFM95 chip to detect incoming messages (activity).
These incoming messages might arrive on any of the well know spreading factors SF7-SF12.
By enabling CAD, the gateway can receive messages of any of the spreading factors.

Actually it is used in normal operation to tell the receiver that another signal is using the
channel already.

The CAD functionality comes at a (little) price: The chip will not be able to receive very weak signals as
the CAD function will use the RSSI register setting of the chip to determine whether or not it received a
signal (or just noise). As a result, very weak signals are not received which means that the range of the
gateway will be reduced in CAD mode.

`#define _CAD 1`


### Over the Air Updates (OTA)

As from version 4.0.6 the gateway allows over the air updated if the setting A_OTA is on.
The over the air software requires once setting of the 4.0.6 version over USB to the gateway,
after which the software is (default) enabled for use.

The first release only supports OTA function using the IDE which in practice means the IDE has to
be on the same network segment as the gateway.

Note: You have to use Bonjour software (Apple) on your network somewhere. A version is available
for most platforms (shipped with iTunes for windows for example). The Bonjour software enables the
gateway to use mDNS to resolve the gateway ID set by OTA after which download ports show up in the IDE.

Todo: The OTA software has not (yet) been tested in conjunction with the WiFiManager software.

`#define A_OTA 1`


### Enable Webserver

This setting enables the webserver. Although the webserver itself takes a lot of memory, it greatly helps
to configure the gateway at run-time and inspects its behavior. It also provides statistics of last messages received.
The A_REFRESH parameter defines whether the webserver should renew every X seconds.

`#define A_SERVER 1         // Define local WebServer only if this define is set`  
`#define A_REFRESH 0        // Will the webserver refresh or not?`  
`#define A_SERVERPORT 80    // local webserver port`  
`#define A_MAXBUFSIZE 192   // Must be larger than 128, but small enough to work`  

 The `A_REFRESH` parameter determines the refresh frequency of the webserver.  

### Strict LoRa behaviour

In order to have the gateway send downlink messages on the pre-set spreading factor and on the default frequency,
you have to set the `_STRICT_1Ch` parameter to 1. Note that when it is not set to 1, the gateway will respond to
downlink requests with the frequency and spreading factor set by the backend server. And at the moment TTN
responds to downlink messages for SF9-SF12 in the RX2 timeslot and with frequency 869.525MHz and on SF12
(according to the LoRa standard when sending in the RX2 timeslot).

`#define _STRICT_1CH 0`

You are advised not to change the default setting of this parameter.

### Enable OLED panel

By setting the OLED you configure the system to work with OLED panels over I2C.
Some panels work by both SPI and I2C where I2c is slower. However, since SPI is use for RFM95 transceiver
communication you are strongly discouraged to use one of these as they will not work with this software.
Instead choose a OLED solution that works over I2C.

`#define OLED 1`

Setting the I2C SDA/SCL pins is done in the config.h file right after the #define of OLED.

### Setting TTN server

The gateway allows to connect to 2 servers at the same time (as most LoRa gateways do BTW).
You have to connect to at least one standard LoRa router, in case you use The Things Network (TTN)
than make sure that you set:

`#define _TTNSERVER "router.eu.thethings.network"`  
`#define _TTNPORT 1700`  

In case you setup your own server, you can specify as follows using your own router URL and your own port:

`#define _THINGSERVER "your_server.com"			// Server URL of the LoRa udp.js server program`  
`#define _THINGPORT 1701							      // Your UDP server should listen to this port`  


### Gateway Identity
Set the identity parameters for your gateway:   

`#define _DESCRIPTION "LoRaGo DOCK – Single-Channel LoRaWAN Gateway"`  
`#define _EMAIL "info@sandboxelectronics.com"`  
`#define _PLATFORM "LoRaGo DOCK"`  
`#define _LAT 52.00`  
`#define _LON 5.800`  
`#define _ALT 14`  


### Using the gateway as a sensor node

It is possible to use the gateway as a node. This way, local/internal sensor values are reported.
This is a cpu and memory intensive function as making a sensor message involves EAS and CMAC functions.

`#define GATEWAYNODE 0`  

Further below in the configuration file, it is possible to set the address and other  LoRa information of the gateway node.

`#if GATEWAYNODE==1`  
`#define _DEVADDR { 0x26, 0x01, 0x15, 0x3D }`  
`#define _APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }`  
`#define _NWKSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }`  
`#define _SENSOR_INTERVAL 300`  
`#endif`  


### Connect to WiFi with WiFiManager

The easiest way to configure the Gateway on WiFi is by using the WiFimanager function. This function works out of the box.
WiFiManager will put the gateway in accesspoint mode so that you can connect to it as a WiFi accesspoint.

`#define WIFIMANAGER 0`  

If Wifi Manager is enabled, make sure to define the name of the accesspoint if the gateway is in accesspoint
mode and the password.

`#define AP_NAME "LoRaGo"`  
`#define AP_PASSWD "DOCK"`  

The standard access point name used by the gateway is "LoRaGo" and its password is "DOCK".
After binding to the access point with your mobile phone or computer, go to http://192.168.4.1 in a browser and tell the gateway to which WiFi network you want it to connect, and specify the password.

The gateway will then reset and bind to the given network. If all goes well you are now set and the ESP8266 will remember the network that it must connect to. NOTE: As long as the accesspoint that the gateway is bound to is present, the gateway will not any longer work with the wpa list of known access points.
If necessary, you can delete the current access point in the webserver and power cycle the gateway to force it to read the wpa array again.


### Other Settings

- `static char *wpa[WPASIZE][2]` contains the array of known WiFi access points the Gateway will connect to. Make sure that the dimensions of the array are correctly defined in the WPASIZE settings.
Note: When the WiFiManager software is enabled (it is by default) there must at least be one entry in the wpa file, wpa[0] is used for storing WiFiManager information.


## Webserver

The built-in webserver can be used to display status and debugging information. Also the webserver allows the user to change certain settings at run-time such as the debug level or switch on
and off the CAD function. It can be accessed with the following URL: http://<YourGatewayIP>:80 where <YourGatewayIP> is the IP given by the router to the ESP8266 at startup. It is probably something like 192.168.1.XX. The webserver shows various configuration settings as well as providing functions to set parameters.

The following parameters can be set using the webServer.
- Debug Level (0-4)
- CAD mode on or off (STD mode)
- Switch frequency hopping on and off (Set to OFF)
- When frequency Hopping is off: Select the frequency the gateway will work with.
NOTE: Frequency hopping is experimental and does not work correctly.
- When CAD mode is off: Select the Spreading Factor (SF) the gateway will work with


# Dependencies

The software is dependent on several pieces of software, the Arduino IDE for ESP8266 being the most important. Several other libraries are also used by this program, make sure you install those libraries with the IDE:

- gBase64 library, The gBase library is actually a base64 library made by Adam Rudd (url=https://github.com/adamvr/arduino-base64). I changed the name because I had another base64 library installed on my system and they did not coexist well.
- Time library (http://playground.arduino.cc/code/time)
- Arduino JSON; Needed to decode downstream messages
- SimpleTimer; Not yet used, but reserved for interrupt and timing
- WiFiManager
- ESP8266 Web Server
- Streaming library, used in the wwwServer part
- AES library (taken from ideetron.nl) for downstream messages
- Time

For convenience, the libraries are also found in this github repository in the libraries directory. Please note that they are NOT part of the ESP 1channel gateway and may have their own licensing. However, these libraries are not part of the single-channel Gateway software.

# License

The source files of the gateway sketch in this repository is made available under the MIT license. The libraries included in this repository are included for convenience only and all have their own license, and are not part of the gateway source code.
