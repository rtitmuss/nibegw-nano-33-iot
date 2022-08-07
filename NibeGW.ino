/**
 * Copyright (c) 2010-2022 Contributors to the openHAB project
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * ----------------------------------------------------------------------------
 *
 *  Author: pauli.anttila@gmail.com
 *
 *
 *  2.11.2013   v1.00   Initial version.
 *  3.11.2013   v1.01
 *  27.6.2014   v1.02   Fixed compile error and added Ethernet initialization delay.
 *  29.6.2015   v2.00   Bidirectional support.
 *  18.2.2017   v3.00   Redesigned.
 *  14.3.2021   v3.01   Fix Prodino build + fixed UDP issue + debug improvements.
 */

// ######### CONFIGURATION #######################

#define VERSION                 "3.01"

// Enable debug printouts
#define ENABLE_DEBUG

// Enable UDP debug printouts, listen printouts e.g. via netcat (nc -l -u 50000)
//#define ENABLE_UDP_DEBUG

#define VERBOSE_LEVEL           1

#define BOARD_NAME              "Arduino NibeGW"
#define BOARD_MAC               { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }
#define BOARD_IP                { 10, 46, 1, 37 }
#define GATEWAY_IP              { 192, 168, 1, 1 }
#define NETWORK_MASK            { 255, 255, 255, 0 }
#define INCOMING_PORT_READCMDS  9999
#define INCOMING_PORT_WRITECMDS 10000

//#define TARGET_IP               10, 46, 1, 45
#define TARGET_IP               10, 46, 1, 26
#define TARGET_PORT             9999
#define TARGET_DEBUG_PORT       50000


// Delay before initialize ethernet on startup in seconds
#define ETH_INIT_DELAY          5

// Used serial port and direction change pin for RS-485 port
// Note! Select if Serial is SW or HW serial port in NibeGw.h
#define RS485_PORT              Serial1
#define RS485_DIRECTION_PIN     2

#define ACK_MODBUS40            true
#define ACK_SMS40               false
#define ACK_RMU40               false
#define SEND_ACK                true

#define DEBUG_BUFFER_SIZE       80

// ######### INCLUDES #######################

#include <SPI.h>
#include <WiFiNINA.h>
#include <wdt_samd21.h>
#include "uptime_formatter.h"

#include "NibeGw.h"
#include "ArduinoSecrets.h"

// ######### VARIABLES #######################

// The media access control (ethernet hardware) address for the shield
byte mac[] = BOARD_MAC;

//The IP address for the shield
byte ip[] = BOARD_IP;

//The IP address of the gateway
byte gw[] = GATEWAY_IP;

//The network mask
byte mask[] = NETWORK_MASK;

// The Wifi SSID
const char *ssid = WIFI_SSID;

// The Wifi password
const char *pass = WIFI_PASS;

unsigned long wifi_delay = ETH_INIT_DELAY;

int WIFI_LED_PIN =  LED_BUILTIN;

IPAddress boardIp(BOARD_IP);

boolean ethernetInitialized = false;

// Target IP address and port where Nibe UDP packets are send
IPAddress targetIp(TARGET_IP);
WiFiUDP udp;
WiFiUDP udp4writeCmnds;

NibeGw nibegw(&RS485_PORT, RS485_DIRECTION_PIN);

// ######### DEBUG #######################

#define DEBUG_BUFFER_SIZE       80

#ifdef ENABLE_DEBUG
 #define DEBUG_PRINT(level, message) if (verbose >= level) { debugPrint(message); }
 #define DEBUG_PRINTDATA(level, message, data) if (verbose >= level) { sprintf(debugBuf, message, data); debugPrint(debugBuf); }
 #define DEBUG_PRINTARRAY(level, data, len) if (verbose >= level) { for (int i = 0; i < len; i++) { sprintf(debugBuf, "%02X", data[i]); debugPrint(debugBuf); }}
#else
 #define DEBUG_PRINT(level, message)
 #define DEBUG_PRINTDATA(level, message, data)
 #define DEBUG_PRINTARRAY(level, data, len)
#endif

#ifdef ENABLE_DEBUG
char verbose = VERBOSE_LEVEL;
char debugBuf[DEBUG_BUFFER_SIZE];

void debugPrint(char* data)
{
#ifdef ENABLE_UDP_DEBUG
  if (ethernetInitialized)
  {
    udp.beginPacket(targetIp, TARGET_DEBUG_PORT);
    udp.write(data);
    udp.endPacket();
  }
#endif

  Serial.print(data);
}
#endif

// ######### FUNCTION DEFINITION ######################

void nibeCallbackMsgReceived(const byte* const data, int len);
int nibeCallbackTokenReceived(eTokenType token, byte* data);
void sendUdpPacket(const byte * const data, int len);


// ######### SETUP #######################

void setup()
{
  // Start watchdog
  wdt_init ( WDT_CONFIG_PER_4K );

  nibegw.setCallback(nibeCallbackMsgReceived, nibeCallbackTokenReceived);
  nibegw.setAckModbus40Address(ACK_MODBUS40);
  nibegw.setAckSms40Address(ACK_SMS40);
  nibegw.setAckRmu40Address(ACK_RMU40);
  nibegw.setSendAcknowledge(SEND_ACK);

#ifdef ENABLE_NIBE_DEBUG
  nibegw.setDebugCallback(nibeDebugCallback);
  nibegw.setVerboseLevel(VERBOSE_LEVEL);
#endif

  Serial.begin(115200, SERIAL_8N1);
  //while ( !Serial );
  
  pinMode( WIFI_LED_PIN, OUTPUT );

  DEBUG_PRINTDATA(0, "%s ", BOARD_NAME);
  DEBUG_PRINTDATA(0, "version %s\n", VERSION);
  DEBUG_PRINT(0, "Started\n");
}

// ######### MAIN LOOP #######################

void loop()
{
  wdt_reset();

  if (!nibegw.connected()) {
    nibegw.connect();
  }
  else {
    do {
      nibegw.loop();
    } while (nibegw.messageStillOnProgress());
  }

  wifiTask();
  loopAnalysis();
}

// ######### FUNCTIONS #######################

void loopAnalysis()
{
  static unsigned long previousMillis = 0;
  static unsigned long lastMillis = 0;
  static unsigned long minLoopTime = 0xFFFFFFFF;
  static unsigned long maxLoopTime = 0;
  static unsigned long totalMaxLoopTime = 0;
  static unsigned long loopCounter = 0;

#define INTERVAL 1000

  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > INTERVAL )
  {
    Serial.println("up: " + uptime_formatter::getUptime());
    Serial.print( "Loops: " );
    Serial.print( loopCounter );
    Serial.print( " ( " );
    Serial.print( minLoopTime );
    Serial.print( " / " );
    Serial.print( maxLoopTime );
    Serial.print( " / " );
    Serial.print( totalMaxLoopTime );
    Serial.println( " )" );
    previousMillis = currentMillis;
    loopCounter = 0;
    minLoopTime = 0xFFFFFFFF;
    maxLoopTime = 0;
  }
  loopCounter++;
  unsigned long loopTime = currentMillis - lastMillis;
  lastMillis = currentMillis;
  if ( loopTime < minLoopTime )
  {
    minLoopTime = loopTime;
  }
  if ( loopTime > maxLoopTime )
  {
    maxLoopTime = loopTime;
  }
  if ( loopTime > totalMaxLoopTime )
  {
    totalMaxLoopTime = loopTime;
  }
}

void wifiTask( void )
{
  enum WIFI_STATE_TYPE {
    WIFI_CONNECT,
    WIFI_VERIFY,
    WIFI_RESTART = 255
  };
  
  static WIFI_STATE_TYPE state = WIFI_CONNECT;
  static int wifiConnectTry = 0;
  static unsigned long previousMillis = 0;

#define WIFI_CONNECT_TIMEOUT 10000
  
  switch ( state )
  {
    case WIFI_CONNECT: {
      if ( millis() - previousMillis < WIFI_CONNECT_TIMEOUT && wifiConnectTry > 0 )
      {
        // just continue with rest of program for now
        break;
      }
      if ( wifiConnectTry > 10 )
      {
        // could not connect, clear everything and try again later
        state = WIFI_RESTART;
        break;
      }
      
      WiFi.config(boardIp);
      int wifiStatus = WiFi.begin( ssid, pass );
      previousMillis = millis();
      wifiConnectTry++;
      
      if ( wifiStatus == WL_CONNECTED )
      {
        state = WIFI_VERIFY;
        ethernetInitialized = true;
        digitalWrite( WIFI_LED_PIN, HIGH );
        DEBUG_PRINT(0, "WiFi connected" );

        udp.begin(INCOMING_PORT_READCMDS); 
        udp4writeCmnds.begin(INCOMING_PORT_WRITECMDS);
        break;
      }

      DEBUG_PRINTDATA(0, "Try: %d\n", wifiConnectTry);
      DEBUG_PRINTDATA(0, "Status: %d\n", wifiStatus);
      break;
    }
    
    case WIFI_VERIFY:
      if (WiFi.status() != WL_CONNECTED)
      {
        state = WIFI_RESTART;
      }
      break;
      
    default:
      state = WIFI_CONNECT;
      ethernetInitialized = false;
      wifiConnectTry = 0;
      WiFi.disconnect();
      WiFi.end();
      digitalWrite( WIFI_LED_PIN, LOW );
      DEBUG_PRINT(0, "WiFi restart" );
      break;
  }
}

void nibeCallbackMsgReceived(const byte* const data, int len)
{
  if (ethernetInitialized)
  {
    sendUdpPacket(data, len);
  }
}

int nibeCallbackTokenReceived(eTokenType token, byte* data)
{
  int len = 0;
  if (ethernetInitialized)
  {
    if (token == READ_TOKEN)
    {
      DEBUG_PRINT(3, "Read token received from nibe\n");
      int packetSize = udp.parsePacket();
      if (packetSize) {
        len = udp.read(data, packetSize);
        DEBUG_PRINTDATA(2, "Send read command to nibe, len=%d, ", len);
        DEBUG_PRINT(1, " data in: ");
        DEBUG_PRINTARRAY(1, data, len)
        DEBUG_PRINT(1, "\n");
      }
    }
    else if (token == WRITE_TOKEN)
    {
      DEBUG_PRINT(3, "Write token received from nibe\n");
      int packetSize = udp4writeCmnds.parsePacket();
      if (packetSize) {
        len = udp4writeCmnds.read(data, packetSize);
        DEBUG_PRINTDATA(2, "Send write command to nibe, len=%d, ", len);
        DEBUG_PRINT(1, " data in: ");
        DEBUG_PRINTARRAY(1, data, len)
        DEBUG_PRINT(1, "\n");
      }
    }
  }
  return len;
}

void nibeDebugCallback(byte verbose, char* data)
{
  DEBUG_PRINT(verbose, data);
}

void sendUdpPacket(const byte * const data, int len)
{
#ifdef ENABLE_DEBUG
  sprintf(debugBuf, "Sending UDP packet to %d.%d.%d.%d:", TARGET_IP);
  DEBUG_PRINT(2, debugBuf);
  DEBUG_PRINTDATA(2, "%d", TARGET_PORT);
  DEBUG_PRINTDATA(2, ", len=%d, ", len);
  DEBUG_PRINT(1, "data out: ");
  DEBUG_PRINTARRAY(1, data, len)
  DEBUG_PRINT(1, "\n");
#endif

  udp.beginPacket(targetIp, TARGET_PORT);
  udp.write(data, len);
  int retval = udp.endPacket();
  DEBUG_PRINTDATA(3, "UDP packet sent %s\n", retval == 0 ? "failed" : "succeed");
}
