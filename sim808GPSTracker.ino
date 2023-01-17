#include <SoftwareSerial.h>
#include <DHT.h>

#define rxPin 10
#define txPin 11

SoftwareSerial ss(rxPin,txPin);

#define GSM_PORT ss

#include "sim808.h"



#define JOURNEY "vr978rye7R"

void setup() {
  dht.begin();  
  // put your setup code here, to run once:
  ss.begin(9600);
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("Starting...");  
  sim808_setup();
}

void sendPositionReport(unsigned long now) {
  GSM_PORT.print("AT+HTTPPARA=\"URL\",\"http://www.iforce2d.net/gt/gt2.php?");
  GSM_PORT.print("&jn=");
  GSM_PORT.print(JOURNEY);
  GSM_PORT.print("&tm=");
  GSM_PORT.print( utc );
  GSM_PORT.print("&fx=");
  GSM_PORT.print(fixStatus);
  GSM_PORT.print("&lt=");
  GSM_PORT.print(lat);
  GSM_PORT.print("&ln=");
  GSM_PORT.print(lon);
  GSM_PORT.print("&sv=");
  GSM_PORT.print(sats);
  GSM_PORT.print("&ha=");
  GSM_PORT.print(hdop);
  GSM_PORT.print("&gs=");
  GSM_PORT.print(sog);
  GSM_PORT.print("&hd=");
  GSM_PORT.print(cog);
  GSM_PORT.println("\"");
  
  flushGSM(now);
  
}

void loop() {
  

  unsigned long now = millis();

  boolean gotGPS = false;

  if ( actionState == AS_IDLE ) {
    if ( fixStatus > 0 && now > lastActionTime + 10000 ) {
  sim808_loop();

      lastActionTime = now;
      httpResult = 0;
      actionState = AS_WAITING_FOR_RESPONSE;
    }
  }
  else {
    // waiting on response - abort if taking too long
    if ( now > lastActionTime + 15000 ) {
      actionState = AS_IDLE;
      parseState = PS_DETECT_MSG_TYPE;
      resetBuffer();
    }
  }
  
  sim808_loop();
}
