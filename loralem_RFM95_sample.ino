#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "i2c_SI7021.h"
SI7021 si7021;
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int GPSRXPin = 7, GPSTXPin = 8;
static const uint32_t GPSBaud = 9600;

SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

// LoRaWAN NwkSKey, network session key, FILL IN YOUR OWN KEY
static const PROGMEM u1_t NWKSKEY[16] = { 0xAx, 0xEx, 0x41, 0xEC, 0x77, 0xC1, 0xD6, 0xB6, 0xF5, 0x3B, 0xE7, 0xE9, 0x28, 0xBD, 0x48, 0xCD };

// LoRaWAN AppSKey, application session key, FILL IN YOUR OWN KEY
static const u1_t PROGMEM APPSKEY[16] = { 0xFC, 0x3F, 0x16, 0x2A, 0xE8, 0xD3, 0xC8, 0xDE, 0x98, 0x75, 0x9F, 0x4F, 0xEB, 0x49, 0xB3, 0x01 };

// LoRaWAN end-device address (DevAddr), FILL IN YOUR OWN ADDRESS
static const u4_t DEVADDR =   0x2601152C ; // console
                      
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void  getGpsCoordinate (long *locationx){
  GPSSerial.begin(GPSBaud);

  long  lat;
  long  lon;

  lat = 0;

  while (lat == 0){
    delay(100);
    
    while (GPSSerial.available() > 0)
        gps.encode(GPSSerial.read());
  
     if (gps.location.isUpdated())
    {
      lat = (long) (gps.location.lat() * 1000000);
      lon = (long) (gps.location.lng() * 1000000);

     locationx[0] = lat;
     locationx[1] = lon;
    }
  }

  Serial.print("x:");
  Serial.println(locationx[0]);
  Serial.print("y:");
  Serial.println(locationx[1]);
  
}

void onEvent (ev_t ev) {
  digitalWrite(10, HIGH);
    
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
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
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
    digitalWrite(10, LOW);
}

void do_send(osjob_t* j){

    digitalWrite(10, HIGH);

 
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
    // Prepare upstream data transmission at the next possible time.

    long location[2];
  
    static float  temp;
    static float  humidity;

    si7021.getTemperature(temp);
    si7021.getHumidity(humidity);
    si7021.triggerMeasurement();
    
    int16_t int_temp = (int16_t)(temp * 100);
    int16_t int_humidity = (int16_t)(humidity * 100);

    uint8_t coords[12];
    
    coords[8] = int_temp >> 8;
    coords[9] = int_temp & 0xFF;

    coords[10] = int_humidity >> 8;
    coords[11] = int_humidity & 0xFF;

    getGpsCoordinate (&location[0]);

    coords[0] = location[0] >> 24;
    coords[1] = location[0] >> 16;
    coords[2] = location[0] >> 8;
    coords[3] = location[0];

    coords[4] = location[1] >> 24;
    coords[5] = location[1] >> 16;
    coords[6] = location[1] >> 8;
    coords[7] = location[1];
    
    LMIC_setTxData2(1, coords, sizeof(coords), 0);

    Serial.println(F("Packet queued"));
    Serial.print("LAT:");
    Serial.println(location[0]);
    Serial.print("LNG:");
    Serial.println(location[1]);
    }
    // Next TX is scheduled after TX_COMPLETE event.
    digitalWrite(10, LOW);
}

void setup() {
    pinMode(10, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("Starting"));

    si7021.initialize();

  
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

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
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
