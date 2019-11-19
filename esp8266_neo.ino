/*
Module standard
- WiFi connectivity 
- MQTT support 
- Manage NEO Pixel led strip

Add former RGB support 

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include "OTA.h"
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#include <Adafruit_NeoPixel.h>
/*
 * http://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */


#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)|| defined(ESP8266_WEMOS_D1R1)
  // for Wemos
  #include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
  #define PIN_RESET 255  //
  #define ISWEMOS 1
  #define DC_JUMPER 0  // I2C Addres: 0 - 0x3C, 1 - 0x3D
  MicroOLED oled(PIN_RESET, DC_JUMPER); // Example I2C declaration
  #define ISWEMOS 1
  #define PINI2C_SDA 4 // pin for I2C SDA GPIO4 (D2)
  #define PINI2C_SCL 5 // pin for I2C SCL GPIO5 (D1)
  #define DHT_PIN 14     // what digital pin we're connected to TO BE CHECKED WITH WEMOS
  #define ONE_WIRE_BUS1 2  // DS18B20 pin (D4) Yellow on D4 / Black on GND / red on 3.3V pullup between Yellow and red 4,7k  TO BE TESTED
  #define ONE_WIRE_BUS2 0  // DS18B20 pin (D4) Yellow on D4 / Black on GND / red on 3.3V pullup between Yellow and red 4,7k  TO BE TESTED
#elif defined(ARDUINO_ESP8266_NODEMCU)
  // for Nodemcu 0.9 or 1.0
  #define ISNODEMCU 1
  #define PIN_R1 4 // pin for relay 1 GPIO4 (D2)
  #define PIN_R2 5 // pin for relay 2 GPIO5 (D1) 
  #define PINI2C_SDA 12 // pin for I2C SDA GPIO12 (D6)
  #define PINI2C_SCL 13 // pin for I2C SCL GPIO13 (D7)
  #define DHT_PIN 14     // what digital pin we're connected to
  #define ONE_WIRE_BUS1 2  // DS18B20 pin (D4) Yellow on D4 / Black on GND / red on 3.3V pullup between Yellow and red 4,7k  
  #define ONE_WIRE_BUS2 0  // DS18B20 pin (D4) Yellow on D4 / Black on GND / red on 3.3V pullup between Yellow and red 4,7k  TO BE TESTED
#elif !(defined(ARDUINO_ESP8266_WEMOS_D1MINI) || defined(ARDUINO_ESP8266_NODEMCU)|| defined(ESP8266_WEMOS_D1R1))
  #error "please check the board you selected, is wiring ok ? only NODEMCU and WeMos supported"
#endif

#define NEOPIN 5 
#define NUM_LEDS 150

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIN, NEO_GRB + NEO_KHZ800);
// Attention pas plus de 5 car sur emplacement
#define TopicCompu "/compu/maison/"   // this one is to be used by other programs
#define TopicHuman "/human/maison/"  // this one to be understand in MQTT by a human being
#define TopicJSON  "/Tjson/"  // in JSON format, only valid mesures
#define Topiccmdctrl "/cmdctrl/maison/down/"  // to receive cmd ctrl commands
#define TopicJeedom "/jeedom/"
// structure
// "cmd" : command to be executed (ie : R1ON R1OFF DEF)
// "dest" hostname targeted
// "attr1" attribute 1 if any 
// "attr2" attribute 2 if any


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define FADESPEED 1
#define colorheartbeat 1

// Uncomment whatever type you're using! pin has been defined above
//#define DHT_TYPE DHT11   // DHT 11
#define DHT_TYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHT_TYPE DHT21   // DHT 21 (AM2301)


#define MainDelay 8000 // main wait in loop()
#define BrokerPace 2000 // Minimal delay between 2 messages, to sure we do not polute the Broker 

// Parametres locaux et Wifi--------------------------------
// const char* ssid = "Livebox-79B6";
// const char* password = "1C1ED33F5A367EC7D564F53F9D";
const char* ssid = "NETGEAR98";
const char* password = "greenvase352";
const char* mqtt_server = "raspfja.home";
char mqtt_client_name[25];
String hostnam ;
char ProbeNameC[10];
String TimeStamp1; // to timestamp the publications
String neoseq ;// sequence to execute for neo led
String ledseq; // sequence to execute for RGB strip

boolean ledon=true;
int ledbright=0;
int ledcolor=0;
int curcolor=0;
int Blue=0;
int Green=2;
int Red=1;


WiFiClient espClient;
OTA ota;


// parametres pour NTP-------------------------------------
unsigned int localPort = 2390;      // local port to listen for UDP packets
/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
IPAddress timeServerIP; // time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
const int timeZone = 1;     // Central European Time

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;



// MQTT ----------------------------------------------------
PubSubClient client(espClient);

long lastMsg = 0;
char msg[150];
int value = 0;
int compteur = 0;
  
void setup() {                      // -------------------------------------------------------------
  String chipidstr;
  String topiccc;
  String probename;
  Serial.begin(115200);
  Serial.println();
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  Serial.println("Init Nodemcu version V1.0.0 20161201a");
  #endif
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  Serial.println("Init Wemos version V1.0.0 20161201a");
  #endif
  

  hostnam =String("ESP8266-OTA-");
  chipidstr =String(ESP.getChipId(), HEX);
  hostnam += chipidstr;
  Serial.print("Hostname: ");
  Serial.println(hostnam);
  
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
// specific Wemos OLED
  Serial.println("OLED is starting...");
  oled.begin();
  oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
  oled.clear(PAGE); // clear display buffer  
  oled.display();   // push to physical display   
  oled.setCursor(0,0);
  oled.setFontType(0);
  oled.println("WH V1.2.0");
  oled.print("starting..");
  oled.println(chipidstr);
  oled.display();
  #endif
  
// start wifi  
  setup_wifi();
// SetUp for OTA
  ota.setup();
  
  
// start MQTT stuff
  mqtt_client_name[0]=0;
  hostnam.toCharArray(mqtt_client_name,hostnam.length()+1);
  Serial.print("MQTT Client Name : ");
  Serial.println(mqtt_client_name);
  
// MQTT setup --------------------------------------
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); // to capture incoming 

  chipidstr.toCharArray(ProbeNameC, chipidstr.length()+1);


// UDP start --------------------------------------
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  Serial.println("Depart =====================");
  
// start NeoPIxel
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  ledseq="BRT";
  neoseq = "STR";

// strart PWM
#ifdef ESP8266
  Wire.begin(PINI2C_SDA,PINI2C_SCL); 
  Wire.pins(PINI2C_SDA,PINI2C_SCL);   
#endif
     

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
    Serial.println("16 channel PWM test!");
    for (int ii=0;ii<6;ii++) {
      pwm.setPWM(ii , 0, 0);
    }
    Serial.println("Rouge....");
    pwm.setPWM(Red,0, 4094);
    pwm.setPWM(Red+3,0, 4094); 
    delay(1000);
        for (int ii=0;ii<6;ii++) {
      pwm.setPWM(ii , 0, 0);
    }
    Serial.println("Blue....");
    pwm.setPWM(Blue,0, 4094);
    pwm.setPWM(Blue+3,0, 4094); 
    delay(1000);
        for (int ii=0;ii<6;ii++) {
      pwm.setPWM(ii , 0, 0);
    }
    Serial.println("Green....");
    pwm.setPWM(Green,0, 4094);
    pwm.setPWM(Green+3,0, 4094); 
    delay(1000); 
        for (int ii=0;ii<6;ii++) {
      pwm.setPWM(ii , 0, 0);
    }
  
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

void setup_wifi() {
  // We start by connecting to a WiFi network
  delay(1000);
   WiFi.hostname(hostnam);
    // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      delay(10);
    }
  }
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);   
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
      oled.print("WiFi:");
    #endif

  WiFi.begin(ssid, password);

 //   WL_NO_SHIELD= 255,   / WL_IDLE_STATUS= 0,WL_NO_SSID_AVAIL=1,WL_SCAN_COMPLETED   = 2,
 //   WL_CONNECTED = 3, WL_FAILED= 4,WL_CONNECTION_LOS = 5, WL_DISCONNECTED= 6
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("."); // will loop forever
    #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
      oled.print(".");
      oled.display();
    #endif
    Serial.print(WiFi.status()); // for debug purpose
    yield();
  }

  Serial.println("");
  Serial.print("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
  String AdrIP;
  AdrIP=IpAddress2String(WiFi.localIP());
      
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    oled.println();
    oled.println(AdrIP);oled.display();
  #endif
  // probleme DHCP qui ne donne pas toujours bonne adresse
  if (AdrIP.substring(0,6)=="10.0.0")
    Serial.println("bon range");
  else
    {
      Serial.println("pas bon le range");
      ESP.reset();
    }
  Serial.print("AP subnet mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("AP gateway: ");
  Serial.println(WiFi.gatewayIP()); 
}

void callback(char* topic, byte* payload, unsigned int length) { // we have made subscription so we could receive something ------
  String MessRecu;

  String stopic;
  String topicdest;
  String probname;
  String topiccc;
  String neoseqS;
  String ledseqS;
  Serial.print("Message arrived >>>>>>>>>>>>>>>>>>>>>>[");
  Serial.print(topic);
  Serial.print("] ");
  

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    MessRecu = MessRecu+(char)payload[i];
  }
  Serial.println(MessRecu);
  topicdest=Topiccmdctrl+String(ESP.getChipId(), HEX);
  Serial.print("Topic dest :");Serial.println(topicdest);
  stopic=String(topic);
  // HTU21/YL69/BMP180/HT22/ds18b20/
  if (stopic == topicdest)
  {
    Serial.println("Received command to be executed");
    Serial.print(">>>>>>> contenu >>>");Serial.print(MessRecu);Serial.println(">");
    Serial.print(">>>>>>> extract >>>");Serial.print(MessRecu.substring(0,3));Serial.print(">");Serial.print(MessRecu.substring(3,4));Serial.print(">");
    if (MessRecu.substring(0,3) == "RLY")
    {
      if (MessRecu.substring(3,6)== "R1ON")
      {
       //digitalWrite(PIN_R1, LOW);
       //digitalWrite(PIN_R2, LOW);
       Serial.println("OOOOOOOOOOOOOOn------------------------------");
      }
      if (MessRecu.substring(3,6) == "R1OF")
      {
      // digitalWrite(PIN_R1, HIGH);
      // digitalWrite(PIN_R2, HIGH);
       Serial.println("OOffffffffffffffffff-------------------------");
      }
    }
    if (MessRecu.substring(0,3) == "CFG")
    {
      probname=MessRecu.substring(3,12);
      Serial.print("Probe name changed to ");
      Serial.println(probname);
      probname.toCharArray(ProbeNameC, probname.length()+1);

    }
    if (MessRecu.substring(0,3) == "NEO")
    {
      neoseq=MessRecu.substring(3,6);
      Serial.print("code recu :");Serial.println(neoseq);Serial.println("substr :");Serial.println(MessRecu.substring(3,6));
      Serial.println("Strobe ..");
      Strobe(0xff, 0xff, 0xff, 10, 50, 1000);
    }
    if (MessRecu.substring(0,3) == "LED")
    {
      ledseq=MessRecu.substring(3,6);
      Serial.print("code recu :");Serial.println(ledseq);Serial.println("substr :");Serial.println(MessRecu.substring(3,6));
      Serial.println("Hip Hop ..");
  //    Strobe(0xff, 0xff, 0xff, 10, 50, 1000);
    }
  }
}

  

void reconnect() { //  let's reconnect to the broker -------------------------------------------------
  char hellomessagechar[150];
  String hellomessagestr;
  String stopic;
  char topic[30];
  String lstreboot;

  hellomessagechar[0]=0;
  lstreboot = ESP.getBootMode();
  hellomessagestr=String ("tu as le bonjour de l'ESP8266, ici ");
  hellomessagestr += hostnam+ " last reboot was "+lstreboot;

  hellomessagestr.toCharArray(hellomessagechar,150);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_name)) {
      Serial.println("connected");
      
      // Once connected, publish an announcement...say hello !!!
      client.publish("outTopic", hellomessagechar);
      // ... and resubscribe in case
      stopic=String(Topiccmdctrl);
     // stopic+="+";
      stopic+= String(ESP.getChipId(), HEX);
      stopic.toCharArray(topic,30);
      client.subscribe(topic);
      client.subscribe("/intopic");
      Serial.print(">>>> subscribe to ");Serial.println(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void NTPGetTime() { // what time is it ? -------------------------------------------------------------------
  // get time
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(4000); // this one may need to be optimised depending on the availability of the targeted server ...

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
    delay(0);
    yield();
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    epoch = epoch + 3600; // to convert to Paris time
    Serial.println(epoch);
    Serial.print(day(epoch));Serial.print("/");
    Serial.print(month(epoch));Serial.print("/");
    Serial.println(year(epoch));Serial.print(" ");
    Serial.print(hour(epoch));Serial.print(":");
    Serial.print(minute(epoch));Serial.print(":");
    Serial.println(second(epoch));
    setTime(epoch); // now our ESP is in our century ...

  }
}

// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void Normaliz( String &StrNorm) { // in case there is only 1 digit add a zero upfront
  if (StrNorm.length() == 1) {
    StrNorm = "0" + StrNorm ;
  }
}

void TimeStamp(String &tmStp) {
  String LocTmpStr;
  tmStp = String(year());
  LocTmpStr = String(month()); Normaliz(LocTmpStr);tmStp +=LocTmpStr;
  LocTmpStr = String(day());   Normaliz(LocTmpStr);tmStp +=LocTmpStr;
  LocTmpStr = String(hour());  Normaliz(LocTmpStr);tmStp +=LocTmpStr;
  LocTmpStr = String(minute());Normaliz(LocTmpStr);tmStp +=LocTmpStr;
  LocTmpStr = String(second());Normaliz(LocTmpStr);tmStp +=LocTmpStr;
}

void loop() {  // et voila on est parti pour un tour ---------------------------------========================
  Serial.print("=");
  // we call NTP only if has not worked yet. at start time is 0 as so 1970 per definition
  if (year() == 1970) { // time to go back to current century ....
    NTPGetTime();
    delay(10);
  }

  long now = millis();
//  delay(MainDelay);
  compteur++; // compteur de nombre boucle
  if (!client.connected()) {
    reconnect(); // to connect to the broker
  }
  client.loop();

  TimeStamp(TimeStamp1);
//  Serial.print("et voila le timestamp : ");
//  Serial.println(TimeStamp1);

  if (ledseq == "COM"){
   Serial.println("Color move ....");
   #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
     // display on OLED
     oled.clear(PAGE);
     oled.setFontType(0);
     oled.setCursor(1,1);
      oled.print("Color Move");
      oled.display();
    #endif
    colormove();
    extinctiondesfeux();
  }

   if (ledseq == "HRT"){
    Serial.println("Heart beat ....");
    #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
      // display on OLED
     oled.clear(PAGE);
     oled.setFontType(0);
     oled.setCursor(1,1);
     oled.print("Heart Beat");
     oled.display();
    #endif
    for (int ii=0; ii<5; ii++) {
      ledheartbeat(); 
      Serial.print("|"); 
      Serial.print(ii);
      client.loop();
      }
    extinctiondesfeux();
   }

   if (ledseq == "BRT"){
    Serial.println("Breathe  ......");
    #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
     // display on OLED
      oled.clear(PAGE);
      oled.setFontType(0);
      oled.setCursor(1,1);
      oled.print("Breathe");
      oled.display();
    #endif
    for (int ii=0; ii<10000; ii++) {ledbreathe();  client.loop();}
    extinctiondesfeux();
   }

      if (ledseq == "FBR"){
    Serial.println("Full breathe  ......");
    #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
     // display on OLED
      oled.clear(PAGE);
      oled.setFontType(0);
      oled.setCursor(1,1);
      oled.print("Full Breathe");
      oled.display();
    #endif
    ledfullbreathe();
    extinctiondesfeux();
   }
  /*
  if (neoseq == "STR"){
   Serial.println("Strip color ..");
   colorWipe(strip.Color(255, 0, 0), 50); // Red
     client.loop();
   colorWipe(strip.Color(0, 255, 0), 50); // Green
     client.loop();
   colorWipe(strip.Color(0, 0, 255), 50); // Blue
     client.loop();
  } 
  if (neoseq == "THR"){
   Serial.println("Theater pixel ..");
   theaterChase(strip.Color(  0,   0, 127), 50); // Blue
  }
  if (neoseq == "RNB"){
   Serial.println("Rainbow cycle ..");
   rainbowCycle(20);   
   client.loop();
  }
  if (neoseq == "TCR"){
   Serial.println("Theater chase raindow ..");
   theaterChaseRainbow(50);   
   client.loop();
  }
 if (neoseq == "BOB"){
   Serial.println("Bouncing ball ..");
   BouncingBalls(0xff,0,0, 3);   
   client.loop();
  }
 if (neoseq == "CYL"){
    Serial.println("Cylon ..");
    CylonBounce(0xff, 0, 0, 4, 10, 50);  
    client.loop();
  }
 if (neoseq == "BRT"){
    Serial.println("Breathe 1  ..");
    breathe1(); 
    client.loop();
  }*/
  client.loop();
  ota.loop();
  Serial.print(".");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void breathe1() {
//Written by: Jason Yandell

int TOTAL_LEDS = 60;
float MaximumBrightness = 255;
float SpeedFactor = 0.008; // I don't actually know what would look good
float StepDelay = 5; // ms for a step delay on the lights

// Make the lights breathe
for (int i = 0; i < 65535; i++) {
// Intensity will go from 10 - MaximumBrightness in a "breathing" manner
float intensity = MaximumBrightness /2.0 * (1.0 + sin(SpeedFactor * i));
strip.setBrightness(intensity);
// Now set every LED to that color
for (int ledNumber=0; ledNumber<TOTAL_LEDS; ledNumber++) {
strip.setPixelColor(ledNumber, 0, 0, 255);
}
strip.show();
}
}

void BouncingBalls(byte red, byte green, byte blue, int BallCount) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  long  BounceDuration=5000; // duration of the effect in ms
  long  BounceStop;

  BounceStop=millis()+BounceDuration;
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  while (BounceStop>millis()) {
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (NUM_LEDS - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],red,green,blue);
    }
    
    showStrip();
    setAll(0,0,0);
  }
}
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();

        delay(wait);

        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){
  for(int j = 0; j < StrobeCount; j++) {
    setAll(red,green,blue);
    showStrip();
    delay(FlashDelay);
    setAll(0,0,0);
    showStrip();
    delay(FlashDelay);
  }
 
 delay(EndPause);
}

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_LEDS-EyeSize-2; i > 0; i--) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}

void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j] );
  }

  showStrip();
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}

// -----------------------------------------------------------------------------
void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

void ledheartbeat(){
int dauer = 1700; //duration of the fade process over all keyframes
int frames [][2] = { //enter frames
                     //first column: time of duration in percent
                     //second column: intensity in percent
    {0, 0},{11, 100},{20, 0},{36, 0},{48, 65},{66, 0},{100, 0}
  };
//these frames are what I think represents a heartbeat


//internal variables:
int framenb; //number of frames
int c; //counter
float t; //time
float Tp1; //Time of first point being selected for fading
float Ip1; //Intensity of first point being selected for fading
float Tp2; //Time of second point being selected for fading
float Ip2; //Intensity of second point being selected for fading
float m; //slope of line
float T0; //time starting point for starting to fade on the cubic function
float Tmax; //stop time to stop fading on the cubic curve
float k; //constant in cubic function

  c = 0;
  framenb = sizeof(frames) / sizeof(int) / 2; //get number of frames
  framenb--; //Correction for array, since adressing in array starts at 0
  while(c < framenb)   //zählt gerenderte Keyframes durch
  {
    Tp1 = frames[c][0] / 100.0 * dauer;
    Tp2 = frames[c+1][0] / 100.0 * dauer; //get the 2 points
    Ip1 = frames[c][1];
    Ip2 = frames[c+1][1];
    Ip1 = Ip1 / 100 * 255; //convert intesities to 8-bit
    Ip2 = Ip2 / 100 * 255;
    if (Ip1 == Ip2)   //if fade is constant
    {
   //   analogWrite(led, pow(255, -2) * pow(Ip1, 3)); //write constant corrected brightness
   pwm.setPWM(colorheartbeat, 0, (pow(255, -2) * pow(Ip1, 3))/256*4096);
   pwm.setPWM(colorheartbeat+3, 0, (pow(255, -2) * pow(Ip1, 3))/256*4096);
//   pwm.setPWM(colorheartbeat, 0, (pow(255, -2) * pow(Ip1, 3))/256*4096);
      delay(Tp2 - Tp1); //wait till second point is reached
    }
    else
    {
      m = (Ip2 - Ip1) / (Tp2 - Tp1); //calculate slope of line
      if (Ip1 < Ip2) //if fade gets brighter
      {
        T0 = (m * Tp1 - Ip1) / m;
        Tmax = (255 + m * Tp1 - Ip1) / m;
        k = 255 / pow(Tmax - T0, 3); //calculating constants of the cubic function
        t = Tp1;
        while (t <= Tp2) //while second point is not reached rewrite the intensity every millicsecond
        {
          //     analogWrite(led, k * pow(t - T0, 3));
          pwm.setPWM(colorheartbeat,0,(k * pow(t - T0, 3)/256*4096));
          pwm.setPWM(colorheartbeat+3,0,(k * pow(t - T0, 3)/256*4096));
          t++;
          delayMicroseconds(100);
        }
      }
      if (Ip1 > Ip2) //if fade gets darker
      {
        Tmax = (m * Tp1 - Ip1) / m;
        T0 = (255 + m * Tp1 - Ip1) / m;
        k = 255 / pow(Tmax - T0, 3); //calc constants
        t = Tp1;
        while(t <= Tp2)
        {
        //  analogWrite(led, k * pow(Tmax - t, 3)); //write value every millisecond
        pwm.setPWM(colorheartbeat,0,(k * pow(Tmax - t, 3))/256*4096);
        pwm.setPWM(colorheartbeat+3,0,(k * pow(Tmax - t, 3))/256*4096);
          t++;
          delayMicroseconds(100);
        }
      }
    }
    c++; //got to next two frames
  }
}

void hiphop() {
  // Drive each PWM in a 'wave'
  int ledbright=200;
  for (uint16_t i=ledbright; i<4096; i += 8) {
    delay(1);
    for (uint8_t pwmnum=0; pwmnum < 3; pwmnum++) {
      pwm.setPWM(pwmnum, 0, i);
    }
  }
  for (uint16_t i=ledbright; i<4096; i += 8) {
    delay(1);
    for (uint8_t pwmnum=0; pwmnum < 3; pwmnum++) {
      pwm.setPWM(pwmnum, 0, 4086-i);
    }
  }
}

float breathe(){
  yield(); // to avoid soft reset on Wemos
  float val = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
  return val;

}

void ledfullbreathe(){
  int milstart;
  int offset_mil;
  int ledbright;
  int ledcolor=1;
  
  milstart=millis();
  offset_mil=(4000-(milstart-3000) % 4000);
  // =4000-MOD(F10-3000;4000)
  while (millis()<(milstart+4000)){
    ledbright= (int) (((exp(sin((millis()+offset_mil)/2000.0*PI)) - 0.36787944)*108.0)/256*4096);
    pwm.setPWM(ledcolor , 0, ledbright);
    pwm.setPWM(ledcolor+3 , 0, ledbright); 
    delay(1);
  }
}
void ledbreathe(){
  boolean ledon=true;
  int ledbright=0;  
  int ledcolor=0;
  int curcolor=0;
 //   delay(1);
    ledbright= (int) (breathe()/256*4096);

    if (ledbright<1){ // if first time change color
      if (ledon){
        pwm.setPWM(0 , 0, 0);pwm.setPWM(1 , 0, 0);
        pwm.setPWM(2 , 0, 0);pwm.setPWM(3 , 0, 0);
        pwm.setPWM(4 , 0, 0);pwm.setPWM(5 , 0, 0);         
        ledon=false;  
        ledcolor=(ledcolor+1) % 3;
        Serial.print("top "); Serial.println(ledcolor);
      }
    }
    else {
      ledon=true;
    }
    pwm.setPWM(ledcolor , 0, ledbright);
    pwm.setPWM(ledcolor+3 , 0, ledbright);    
}

void colormove(){
    int r, g, b;
 
  Serial.println("fade from blue to violet");
   #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
     // display on OLED
     oled.setFontType(0);
     oled.setCursor(0,10);
     oled.print("Blue->Vio");
     oled.display();
   #endif
  for (r = 0; r < 4096; r++) { 
    pwm.setPWM(Red,0, r);
   // Serial.print(" ");Serial.print(r);
    pwm.setPWM(Red+3,0, r);
    delay(FADESPEED);
  } 
  Serial.println("fade from violet to red");
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    // display on OLED
    oled.setFontType(0);
    oled.setCursor(0,10);
    oled.print("Vio->Red   ");
    oled.display();
  #endif
  for (b = 4095; b > 0; b--) { 
    pwm.setPWM(Blue,0, b);
    pwm.setPWM(Blue+3,0, b);
    delay(FADESPEED);
  } 
  Serial.println("fade from red to yellow");
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    // display on OLED
    oled.setFontType(0);
    oled.setCursor(0,10);
    oled.print("Red->Yel  ");
    oled.display();
  #endif   
  for (g = 0; g < 4096; g++) {    
    pwm.setPWM(Green, 0, g);
    pwm.setPWM(Green+3, 0, g);
    delay(FADESPEED);
  } 
  Serial.println("fade from yellow to green");
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    // display on OLED
    oled.setFontType(0);
    oled.setCursor(0,10);
    oled.print("Yel->Green  ");
    oled.display();
  #endif    
  for (r = 4095; r > 0; r--) {    
    pwm.setPWM(Red,0, r);
    pwm.setPWM(Red+3,0, r);    
    delay(FADESPEED);
  } 
  Serial.println("fade from green to teal");
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    // display on OLED
    oled.setFontType(0);
    oled.setCursor(0,10);
    oled.print("Gre->Teal  ");
    oled.display();
  #endif    
  for (b = 0; b < 4096; b++) {     
    pwm.setPWM(Blue,0, b);
    pwm.setPWM(Blue+3,0, b);    
    delay(FADESPEED);
  } 
  Serial.println("fade from teal to blue");
  #if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
    // display on OLED
    oled.setFontType(0);
    oled.setCursor(0,10);
    oled.print("Teal->Blu  ");
    oled.display();
  #endif 
  for (g = 4095; g > 0; g--) { 
    pwm.setPWM(Green,0, g);
    pwm.setPWM(Green+3,0, g);
    delay(FADESPEED);
  } 
   Serial.println("-----------------------------");
}

void extinctiondesfeux(){
   pwm.setPWM(Green,0, 0);
   pwm.setPWM(Red,0, 0);
   pwm.setPWM(Blue,0, 0);
   pwm.setPWM(Green+3,0, 0);
   pwm.setPWM(Red+3,0, 0);
   pwm.setPWM(Blue+3,0, 0);   
   delay(1000);
}
