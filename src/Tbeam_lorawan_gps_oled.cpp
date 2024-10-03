 /*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/
#include <Arduino.h>
#include <TinyGPS++.h>                       
#include <SPI.h>
#include <Wire.h>  
//#include "SSD1306.h" 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMono9pt7b.h>
//#include <TimeLib.h>    // can't use <Time.h> starting with v1.6.1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#include <lmic.h>
#include <hal/hal.h>

#define  LMIC_DEBUG_LEVEL = 1 
#define CFG_au915

#include <secrets.h>


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[4];
uint8_t txBuffer[9];
static osjob_t sendjob;

uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t nSat;
uint8_t SF;
volatile uint8_t oneChannel;
uint32_t packet=0;
volatile uint8_t flag= 0; 

// SD SPI pins
//#define MOSI 13
//#define MISO 34
//#define CLK 4 //was 23 in TTGO
//#define CS  2

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;
const unsigned TICKS_INTERVAL= 2;
unsigned int loop_count= 10;  // sends the first message


int lastpress=0;

// Pin mapping. T-beam
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23, //was 14
  .dio = {26, 33, 32}  // T-beam
};


TinyGPSPlus gps;   
uint32_t userUTCTime; // Seconds since the UTC epoch

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}


void buildPacket(uint8_t txBuffer[9])
{
  char t[32]; // used to sprintf for Serial output
  
  LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  
  sprintf(t, "Lat: %f", gps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", gps.location.lng());
  Serial.println(t);
  
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = gps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  nSat = gps.satellites.value();
  txBuffer[8] = nSat & 0xFF;
}


void displayLoc(){
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 5);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");

  display.clearDisplay();
  //display.setFont(&FreeMono9pt7b);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font


  display.setCursor(0,0); 
  display.print("Lat "+ String(gps.location.lat()) );
  display.setCursor(0,11);
  display.print("Long "+ String(gps.location.lng()) );
  display.setCursor(0, 21);
  display.print( "alt "+ String(gps.altitude.feet() / 3.2808) );
 
  display.print( "  sat "+ String(gps.satellites.value()) );
  display.setCursor(0, 31);
  display.print("Packet ");
  display.print(packet);
  
  display.setCursor(0, 41);
  display.print("SF:");
  display.print(SF);
  display.setCursor(64, 41);
  display.print((TX_INTERVAL -loop_count));
  if (oneChannel == 1){
    display.setCursor(0, 51);
    display.print(" Only (917Mhz): YES");
  } else {
    display.setCursor(0, 51);
    display.print(" Only (917Mhz): NO ");
  }
  
  display.display();
  
  
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void setSF() {
    switch (SF) {
        case 7:
            LMIC_setDrTxpow(DR_SF7, 14);
            break;
        case 8:
            LMIC_setDrTxpow(DR_SF8, 14);
            break;
        case 9: 
            LMIC_setDrTxpow(DR_SF9, 14);
            break;
        case 10: 
            LMIC_setDrTxpow(DR_SF10, 14);
            break;
        case 11: 
            LMIC_setDrTxpow(DR_SF11, 14);
            break;
        case 12: 
            LMIC_setDrTxpow(DR_SF12, 14);
            break;
    }

}



void do_send(osjob_t* j) {  
  // Check if there is not a current TX/RX job running
  smartDelay(500); 
  displayLoc();
    
  if (loop_count >= TX_INTERVAL) {
    loop_count=0;
    packet++; 
    
    if (LMIC.opmode & OP_TXRXPEND)
    {
      Serial.println(F("OP_TXRXPEND, not sending"));
    //  LoraStatus = "OP_TXRXPEND, not sending";
    }
    else
    { 
    // if (checkGpsFix())
    //  {
        // Prepare upstream data transmission at the next possible time.
        //LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
        setSF();
        buildPacket(txBuffer);
        if (oneChannel ==1){
          LMIC_disableChannel(8);
      
            //     LMIC_disableChannel(9);
      
          LMIC_disableChannel(10);
          LMIC_disableChannel(11);
          LMIC_disableChannel(12);
          LMIC_disableChannel(13);
          LMIC_disableChannel(14);
          LMIC_disableChannel(15);

        }
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
        Serial.println(F("Packet queued"));
      //axp.setChgLEDMode(ledMode);
      //LoraStatus = "Packet queued";
        //   }
        //   else
        //   {
              //try again in 3 seconds
        //     os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
        //  }
    }
  } else {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TICKS_INTERVAL), do_send);
    
  }    
  
  
  loop_count+=TICKS_INTERVAL;



  // Next TX is scheduled after TX_COMPLETE event.
}


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      else
        Serial.println("Not received ack");
        
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TICKS_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}




void IRAM_ATTR changeSF(){
  if ((millis() - lastpress) > 1000) {
    lastpress= millis();
    if (++SF % 13 == 0){
        SF= 7;
        if (oneChannel == 0) oneChannel=1;
        else oneChannel =0;
    }

  }  
}


                         
//HardwareSerial SerialGPS(1);                 


void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
  pinMode(16,OUTPUT);
  
  Wire.begin(21,22);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // button 38 

  pinMode(38, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(38), changeSF, RISING);
  
 // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band  
 // LMIC_setupChannel(8, 994000000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  2);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);



#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);
  SF=7;
  oneChannel= 0;

  // Start job
  do_send(&sendjob);
  

}




void loop()
{
  
  os_runloop_once();
                                
 /* if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));*/
}



/* Original
void gps::buildPacket(uint8_t txBuffer[9])
{
  LatitudeBinary = ((tGps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((tGps.location.lng() + 180) / 360.0) * 16777215;
  
  sprintf(t, "Lat: %f", tGps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", tGps.location.lng());
  Serial.println(t);
  
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = tGps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = tGps.hdop.value()/10;
  txBuffer[8] = hdopGps & 0xFF;
}
*/





void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        Serial.println(F("USER CALLBACK: Not a success"));
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        Serial.println(F("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed"));
        return;
    }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

    // Add the delay between the instant the time was transmitted and
    // the current time

    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;

    // Update the system time with the time read from the network
    //  setTime(*pUserUTCTime);

    Serial.print(F("Received by TTN: "));
    const char *str = ctime((const time_t*)pUserUTCTime);
    Serial.println(str);
   /* Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print('/');
    Serial.print(month());
    Serial.print('/');
    Serial.print(year());
    Serial.println();*/
}
