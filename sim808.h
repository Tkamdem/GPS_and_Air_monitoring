
#define DEBUG_SERIAL

#define TOKEN               "lKvJIUDx8RWRFtVwwcCn"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"
#define THINGSBOARD_PORT    80

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD   115200

// Select your modem:
#define TINY_GSM_MODEM_SIM808
#define DHTPIN 7     // DHT sensor feed point
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

#include <TinyGsmClient.h>
#include "ThingsBoard.h"
#include <stdlib.h>
// Your GPRS credentials
// Leave empty, if missing user or pass

const char apn[]  = "internet";
const char user[] = "orange";
const char pass[] = "orange";

//#define DUMP_AT_COMMANDS 1

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(ss, Serial);
  TinyGsm modem(debugger);
#else
  // Initialize GSM modem
  TinyGsm modem(ss);
#endif

// Initialize GSM client
TinyGsmClient client(modem);

// Initialize ThingsBoard instance
ThingsBoardHttp tb(client, TOKEN, THINGSBOARD_SERVER, THINGSBOARD_PORT);


void updateScreen();

enum _parseState {
  PS_DETECT_MSG_TYPE,

  PS_IGNORING_COMMAND_ECHO,

  PS_HTTPACTION_TYPE,
  PS_HTTPACTION_RESULT,
  PS_HTTPACTION_LENGTH,

  PS_HTTPREAD_LENGTH,
  PS_HTTPREAD_CONTENT,

  PS_CGNSINF_RUN_STATUS,
  PS_CGNSINF_FIX_STATUS,
  PS_CGNSINF_UTC,
  PS_CGNSINF_LAT,
  PS_CGNSINF_LON,
  PS_CGNSINF_MSL,
  PS_CGNSINF_SOG,
  PS_CGNSINF_COG,
  PS_CGNSINF_FIX_MODE,
  PS_CGNSINF_RESERVED1,
  PS_CGNSINF_HDOP,
  PS_CGNSINF_PDOP,
  PS_CGNSINF_VDOP,
  PS_CGNSINF_RESERVED2,
  PS_CGNSINF_GPS_SATS_IN_VIEW,
  PS_CGNSINF_GNSS_SATS_USED,
  PS_CGNSINF_GLONASS_SATS_IN_VIEW,
  PS_CGNSINF_RESERVED3,
  PS_CGNSINF_CN0,
  PS_CGNSINF_HPA,
  PS_CGNSINF_VPA
};

enum _actionState {
  AS_IDLE,
  AS_WAITING_FOR_RESPONSE
};

byte actionState = AS_IDLE;
unsigned long lastActionTime = 0;

byte parseState = PS_DETECT_MSG_TYPE;
char buffer[20];
byte pos = 0;

int httpResult = 0;
int contentLength = 0;

byte fixStatus = 0;
char utc[24];
char lat[16];
char lon[16];
char sog[8];
char cog[8];
char hdop[8];
byte sats = 0;



void resetBuffer() {
  memset(buffer, 0, sizeof(buffer));
  pos = 0;
}

void parseATText(byte b) {

#ifdef DEBUG_SERIAL
  Serial.write(b);
#endif

  buffer[pos++] = b;

  if ( pos >= sizeof(buffer) ) 
    resetBuffer(); // just to be safe
  
    #ifdef DEBUG_SERIAL
   // Detailed debugging
   /*Serial.println();
   Serial.print("state = ");
   Serial.println(parseStat);
   Serial.print("b = ");
   Serial.println(b);
   Serial.print("pos = ");
   Serial.println(pos);
   Serial.print("buffer = ");
   Serial.println(buffer);*/
   #endif
   

  switch (parseState) {
  case PS_DETECT_MSG_TYPE: 
    {
      if ( b == '\n' )
        resetBuffer();
      else {        
        if ( pos == 3 && strcmp(buffer, "AT+") == 0 ) {
          parseState = PS_IGNORING_COMMAND_ECHO;
        }
        else if ( b == ':' ) {
#ifdef DEBUG_SERIAL
          Serial.print("Checking message type: ");
          Serial.println(buffer);
#endif

          if ( strcmp(buffer, "+HTTPACTION:") == 0 ) {
#ifdef DEBUG_SERIAL
            Serial.println("Received HTTPACTION");
#endif
            parseState = PS_HTTPACTION_TYPE;
          }
          else if ( strcmp(buffer, "+HTTPREAD:") == 0 ) {
#ifdef DEBUG_SERIAL
            Serial.println("Received HTTPREAD"); 
#endif           
            parseState = PS_HTTPREAD_LENGTH;
          }
          else if ( strcmp(buffer, "+CGNSINF:") == 0 ) {
#ifdef DEBUG_SERIAL
            Serial.println("Received CGNSINF"); 
#endif           
            parseState = PS_CGNSINF_RUN_STATUS;
          }
          resetBuffer();
        }
      }
    }
    break;

  case PS_IGNORING_COMMAND_ECHO:
    {
      if ( b == '\n' ) {
#ifdef DEBUG_SERIAL
        Serial.print("Ignoring echo: ");
        Serial.println(buffer);
#endif
        parseState = PS_DETECT_MSG_TYPE;
        resetBuffer();
      }
    }
    break;

  case PS_HTTPACTION_TYPE:
    {
      if ( b == ',' ) {
#ifdef DEBUG_SERIAL
        Serial.print("HTTPACTION type is ");
        Serial.println(buffer);
#endif
        parseState = PS_HTTPACTION_RESULT;
        resetBuffer();
      }
    }
    break;

  case PS_HTTPACTION_RESULT:
    {
      if ( b == ',' ) {
#ifdef DEBUG_SERIAL
        Serial.print("HTTPACTION result is ");
        Serial.println(buffer);
#endif
        httpResult = atoi(buffer);
        parseState = PS_HTTPACTION_LENGTH;
        resetBuffer();
      }
    }
    break;

  case PS_HTTPACTION_LENGTH:
    {
      if ( b == '\n' ) {
#ifdef DEBUG_SERIAL
        Serial.print("HTTPACTION length is ");
        Serial.println(buffer);
#endif

        contentLength = atoi(buffer);
        // now request content
        if ( contentLength > 0 ) {
          GSM_PORT.print("AT+HTTPREAD=0,");
          GSM_PORT.println(buffer);
        }
        else
          actionState = AS_IDLE;

        parseState = PS_DETECT_MSG_TYPE;
        resetBuffer();
      }
    }
    break;

  case PS_HTTPREAD_LENGTH:
    {
      if ( b == '\n' ) {
        contentLength = atoi(buffer);
#ifdef DEBUG_SERIAL
        Serial.print("HTTPREAD length is ");
        Serial.println(contentLength);

        Serial.print("HTTPREAD content: ");
#endif

        parseState = PS_HTTPREAD_CONTENT;
        resetBuffer();
      }
    }
    break;

  case PS_HTTPREAD_CONTENT:
    {
      // for this demo I'm just showing the content bytes in the serial monitor
#ifdef DEBUG_SERIAL
      Serial.write(b);
#endif

      contentLength--;

      if ( contentLength <= 0 ) {

        // all content bytes have now been read

        parseState = PS_DETECT_MSG_TYPE;
        resetBuffer();

#ifdef DEBUG_SERIAL
        Serial.print("\n\n\n\n");
#endif

        actionState = AS_IDLE;
      }
    }
    break;

  case PS_CGNSINF_RUN_STATUS:
  case PS_CGNSINF_FIX_STATUS:
  case PS_CGNSINF_UTC:
  case PS_CGNSINF_LAT:
  case PS_CGNSINF_LON:
  case PS_CGNSINF_MSL:
  case PS_CGNSINF_SOG:
  case PS_CGNSINF_COG:
  case PS_CGNSINF_FIX_MODE:
  case PS_CGNSINF_RESERVED1:
  case PS_CGNSINF_HDOP:
  case PS_CGNSINF_PDOP:
  case PS_CGNSINF_VDOP:
  case PS_CGNSINF_RESERVED2:
  case PS_CGNSINF_GPS_SATS_IN_VIEW:
  case PS_CGNSINF_GNSS_SATS_USED:
  case PS_CGNSINF_GLONASS_SATS_IN_VIEW:
  case PS_CGNSINF_RESERVED3:
  case PS_CGNSINF_CN0:
  case PS_CGNSINF_HPA:
    {
      if ( b == ',' ) {
#ifdef DEBUG_SERIAL
        Serial.print("CGNSINF result for is ");
        Serial.print( parseState );
        Serial.print(" is ");
        Serial.println(buffer);
#endif
        if ( parseState == PS_CGNSINF_FIX_STATUS )
          fixStatus = atoi( buffer );
        else if ( parseState == PS_CGNSINF_GNSS_SATS_USED )
          sats = atoi( buffer );
        else if ( parseState == PS_CGNSINF_LAT )
          strncpy( lat, buffer, min(15,strlen(buffer)-1));
        else if ( parseState == PS_CGNSINF_LON )
          strncpy( lon, buffer, min(15,strlen(buffer)-1));
        else if ( parseState == PS_CGNSINF_SOG )
          strncpy( sog, buffer, min(7,strlen(buffer)-1));
        else if ( parseState == PS_CGNSINF_COG )
          strncpy( cog, buffer, min(7,strlen(buffer)-1));
        else if ( parseState == PS_CGNSINF_HDOP )
          strncpy( hdop, buffer, min(7,strlen(buffer)-1));
        else if ( parseState == PS_CGNSINF_UTC )
          strncpy( utc, buffer, min(14,strlen(buffer)-1));
          
        parseState += 1;
        resetBuffer();
      }
    }
    break;

  case PS_CGNSINF_VPA:
    {
      if ( b == '\n' ) {
#ifdef DEBUG_SERIAL
        Serial.print("PS_CGNSINF_VPA is ");
        Serial.println(buffer);
#endif
        actionState = AS_IDLE;
        parseState = PS_DETECT_MSG_TYPE;
        resetBuffer();
        Serial.println("Sending data...");
        tb.sendTelemetryFloat("temperature", dht.readTemperature());

  // Serial.println("Sending humidity data...");
        tb.sendTelemetryFloat("humidity", dht.readHumidity());
        tb.sendTelemetryFloat("latitude",atof(lat));
  // Serial.println(atof(lat));

  // Serial.println("Sending longitude data...");
        tb.sendTelemetryFloat("longitude", atof(lon));
  
  // Serial.println("Sending speed data...");
        tb.sendTelemetryFloat("speed", atof(sog));
  // Serial.println(sog);
        //updateScreen();
      }
    }
    break;
  }
}

void sendGSM(const char* msg, int waitMs = 500) {
  GSM_PORT.println(msg);
  while(GSM_PORT.available()) {
    parseATText(GSM_PORT.read());
  }
  delay(waitMs);
}

void sim808_setup() {

  delay(500);
  
  ss.println("AT+CGNSPWR=1\n");

  delay(500);

  // wait ten seconds for GSM module to connect to mobile network
  Serial.print( "Waiting for SIM startup..." );
  delay(10000);

  sendGSM("AT+SAPBR=3,1,\"APN\",\"vodafone\"");  // change this for your cell provider
  sendGSM("AT+SAPBR=1,1",3000);
  sendGSM("AT+HTTPINIT", 500);
  sendGSM("AT+HTTPPARA=\"CID\",1", 500);

  delay(500);
  
  ss.println("AT+CGNSPWR=1\n");

  delay(500);
}

void flushGSM(unsigned long now) {
  while(GSM_PORT.available()) {
    //lastActionTime = now;
    parseATText(GSM_PORT.read());
  }
}

void sim808_loop() {

  unsigned long now = millis();
  flushGSM(now);

  static unsigned long lastLocCheck = 0;
  if ( now - lastLocCheck > 5000 ) {
    ss.println("AT+CGNSINF\n");
    lastLocCheck = now;
  }
}

