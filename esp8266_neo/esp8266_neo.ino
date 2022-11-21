/*
Version b : new way to call NTP using a library
Module standard
- WiFi connectivity 
- MQTT support 
- Manage NEO Pixel led strip

MQTT is used to allow changes to the led sequence. Topic to be used to send the sequence to be used is /cmdctrl/maison/down/xxxx/ where XXXX is the ChipID of the ESP chip.

Message to be sent is NEOXXX where XXX is the sequence to be trigerred
NEOALL : all sequences one after another (default/start value) 
NEORGB : RGB sequence
NEOTHR : Theater
NEOCYL : Cylon
NEOHRT : Red Heart beat 
NEORNB : Rainbow
NEOBOB : Rebound
NEOBR2 : Breathe 2
NEOBRT : Breathe 3

Add former RGB support 

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include "OTA.h"
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <math.h>
#include "wificredential.h"
#include <Adafruit_NeoPixel.h>
#include <NTPClient.h>
/*
 * http://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 * is my reference !!!!!!
 */


#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
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
#elif !(defined(ARDUINO_ESP8266_WEMOS_D1MINI) || defined(ARDUINO_ESP8266_NODEMCU))
  #error "please check the board you selected, is wiring ok ? only NODEMCU and WeMos supported"
#endif

#define NEOPIN 5 // PIN D1 on wemos or nodemcu
#define WIFIMQTT 1 // 1 with  wifi and mqtt other value like 0 no Wifi and no MQTT
// #define NUM_LEDS 300 // number of LED on the strip 300 70
#define NUM_LEDS 208 // number of LED on the strip 300 70


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

// Verifier si sert ****
#define FADESPEED 1
#define colorheartbeat 1

// Uncomment whatever type you're using! pin has been defined above
//#define DHT_TYPE DHT11   // DHT 11
#define DHT_TYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHT_TYPE DHT21   // DHT 21 (AM2301)


#define MainDelay 8000 // main wait in loop()
#define BrokerPace 2000 // Minimal delay between 2 messages, to sure we do not polute the Broker 


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
int LoopNbr=0; // loop number


WiFiClient espClient;
OTA ota;


// parametres pour NTP-------------------------------------
// A UDP instance to let us send and receive packets over UDP
WiFiUDP ntpUDP;
NTPClient temps(ntpUDP, "fr.pool.ntp.org", 3600, 60000);


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
  Serial.println(__FILE__);Serial.println(__DATE__);  Serial.println(__TIME__);
  Serial.print("LED Strip animation "); Serial.print(NUM_LEDS);Serial.println(" LEDs");
  Serial.println("Data to be connected to PIN D1");
    

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
  oled.println("NEO2021 V1");
  oled.print("starting..");
  oled.println(chipidstr);
  oled.display();
  #endif

if (WIFIMQTT == 1)
{  
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


// NTPClient  start --------------------------------------
  temps.begin();
  temps.setTimeOffset(0); // I am in Paris
  
  Serial.println("Depart =====================");
} // end of wifi MQTT stuff 
  
// start NeoPIxel
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  neoseq = "ALL"; // we will start with 

} // end of setup

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
  String ledseqS;
  // Serial.print("neoseq avant :");Serial.println(neoseq);
  Serial.print("Message arrived >>>>>>>>>>>>>>>>>>>>>>[");
  Serial.print(topic);
  Serial.print("] ");
  

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    MessRecu = MessRecu+(char)payload[i];
  }
  Serial.println(MessRecu);
  topicdest=Topiccmdctrl+String(ESP.getChipId(), HEX)+"/";
  Serial.print("Topic dest :");Serial.println(topicdest);
  stopic=String(topic);
  // HTU21/YL69/BMP180/HT22/ds18b20/
  if (stopic == topicdest)
  {
    Serial.println("Received command to be executed");
    Serial.print(">> full content >>>");Serial.print(MessRecu);Serial.println("<");
    Serial.print(">> first code >>>>>");Serial.print(MessRecu.substring(0,3));Serial.print("<");
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
      strip.setBrightness(255);
    }
    if (MessRecu.substring(0,3) == "LED")
    {
      ledseq=MessRecu.substring(3,6);
      Serial.print("code recu :");Serial.println(ledseq);Serial.println("substr :");Serial.println(MessRecu.substring(3,6));
      Serial.println("Hip Hop ..");
  //    Strobe(0xff, 0xff, 0xff, 10, 50, 1000);
    }
  }
  // Serial.print("Apres :");Serial.println(neoseq);
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
      stopic+= "/";
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
  temps.update(); // Get NTP Time
  setTime(temps.getEpochTime()); // update OS date/time now our ESP is in our century ...
  Serial.println(temps.getFormattedTime());
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
  Serial.println();Serial.print(LoopNbr);Serial.print(" - loop type =");++LoopNbr;
  Serial.println(neoseq);
 if (WIFIMQTT == 1)
 {
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
  } 

// 2019 OK
if (neoseq == "ALL"){
  //STR
   strip.setBrightness(125);
   Serial.println("Snow sparkle...");
   for (int ii=0;ii<17;ii++) {
    SnowSparkle(strip.Color(10, 10, 10));
    client.loop();if (neoseq != "ALL"){return;}
   }
   
   Serial.println("Strip color ..");
   colorWipe(strip.Color(255, 0, 0), 50); // Red
     client.loop();if (neoseq != "ALL"){return;}
   colorWipe(strip.Color(0, 255, 0), 50); // Green
     client.loop();if (neoseq != "ALL"){return;}
   colorWipe(strip.Color(0, 0, 255), 50); // Blue
     client.loop();if (neoseq != "ALL"){return;}
   // RGB Wipe
   Serial.println("RGB Wipe ..");
   RGBLoop(); client.loop();RGBLoop(); client.loop();
   // Heart beat 
   Serial.println("Heart beat ..");
   heartbeat(10); 
   client.loop();if (neoseq != "ALL"){return;}
   // Theatre
   strip.setBrightness(125);
   Serial.println("Theater pixel ..");theaterChase(strip.Color(  0,   0, 127), 50);theaterChase(strip.Color(  0,   0, 127), 50); theaterChase(strip.Color(  0,   0, 127), 50); // Blue
   // Rainbow
   strip.setBrightness(125);
   Serial.println("Rainbow cycle ..");rainbowCycle(5); client.loop();if (neoseq != "ALL"){return;}
   // Theatre chase
   Serial.println("Theater chase raindow ..");theaterChaseRainbow(5); client.loop();if (neoseq != "ALL"){return;}
   // BOB
   Serial.println("Bouncing ball .."); 
   BouncingBalls(0xff,0,0, 3);client.loop();if (neoseq != "ALL"){return;}
   BouncingBalls(0xff,0,0, 3);client.loop();if (neoseq != "ALL"){return;}
   BouncingBalls(0xff,0,0, 3);client.loop();if (neoseq != "ALL"){return;}
   // CYlon 
   Serial.println("Cylon .."); 
   CylonBounce(0xff, 0, 0, 4, 10, 50); client.loop();if (neoseq != "ALL"){return;}
   CylonBounce(0xff, 0, 0, 4, 10, 50); client.loop();if (neoseq != "ALL"){return;}
   CylonBounce(0xff, 0, 0, 4, 10, 50); client.loop();if (neoseq != "ALL"){return;}
  // Breathe 2
    Serial.println("Breathe 2  .."); breathe2(); client.loop();if (neoseq != "ALL"){return;}
  // Fire 
    Serial.println("Fire  ..");
    Fire(55,120,15);
    client.loop();if (neoseq != "ALL"){return;}
  }

  if (neoseq == "STR"){
   strip.setBrightness(125);
   Serial.println("Strip color ..");
   colorWipe(strip.Color(255, 0, 0), 50); // Red
     client.loop();if (neoseq != "STR"){return;}
   colorWipe(strip.Color(0, 255, 0), 50); // Green
     client.loop();if (neoseq != "STR"){return;}
   colorWipe(strip.Color(0, 0, 255), 50); // Blue
     client.loop();if (neoseq != "STR"){return;}
  } 

// 2019 OK 
  if (neoseq == "RGB"){
   Serial.println("RGB Wipe ..");
   RGBLoop(); 
   client.loop();if (neoseq != "RGB"){return;}
  }
  if (neoseq == "HRT"){
   Serial.println("Heart beat ..");
   heartbeat(1); 
   client.loop();
  }
  if (neoseq == "THR"){
    
   Serial.println("Theater pixel ..");
   theaterChase(strip.Color(  0,   0, 127), 50); // Blue
  }
  if (neoseq == "RNB"){
   Serial.println("Rainbow cycle ..");
   rainbowCycle(1);   
   client.loop();
  }
  if (neoseq == "TCR"){
   Serial.println("Theater chase raindow ..");
   theaterChaseRainbow(1);   
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
    Serial.println("Breathe 3  ..");
    breathe3(); 
    client.loop();
  }
  if (neoseq == "BR2"){
    Serial.println("Breathe 2  ..");
    breathe2(); 
    client.loop();
  }
    if (neoseq == "FIR"){
    Serial.println("Fire  ..");
    Fire(55,120,15);
    client.loop();
  }
  client.loop();
  ota.loop();
  Serial.print(".");
}

// --------- ALL THE EFFECTS ARE BELOW -----------
void heartbeat(int num_beat) { //Written by Firionus, THANKS !!!!!

  int dauer = 600; //duration of the fade process over all keyframes
  //enter frames
  //first column: time of duration in percent
  //second column: intensity in percent
  int frames [][2] = {{0, 0},{11, 100},{20, 0},{36, 0},{48, 65},{66, 0},{100, 0}};
  //these frames are what I think represents a heartbeat


  //internal variables:
  int anzahl; //number of frames
  int c; //counter
  int d; // another counter
  float t; //time
  float Tp1; //Time of first point being selected for fading
  float Ip1; //Intensity of first point being selected for fading
  float Tp2; //Time of second point being selected for fading
  float Ip2; //Intensity of second point being selected for fading
  float m; //slope of line
  float T0; //time starting point for starting to fade on the cubic function
  float Tmax; //stop time to stop fading on the cubic curve
  float k; //constant in cubic function

  anzahl = sizeof(frames) / sizeof(int) / 2; //get number of frames
  anzahl--; //Correction for array, since adressing in array starts at 0
  // Serial.println(num_beat);
  for (int ii=0;ii<num_beat;ii++) {
    c = 0;
   // Serial.println("I was here");
   while(c < anzahl)   //zählt gerenderte Keyframes durch
   {
    Tp1 = frames[c][0] / 100.0 * dauer;
    Tp2 = frames[c+1][0] / 100.0 * dauer; //get the 2 points
    Ip1 = frames[c][1];
    Ip2 = frames[c+1][1];
    Ip1 = Ip1 / 100 * 255; //convert intesities to 8-bit
    Ip2 = Ip2 / 100 * 255;
    if (Ip1 == Ip2)   //if fade is constant
    {
     strip.setBrightness((pow(255, -2) * pow(Ip1, 3))/256*255);
     for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
      strip.setPixelColor(ledNumber, 255, 0, 0);
      }
      strip.show();
      yield();
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
     // pwm.setPWM(COLOOR,0,(k * pow(t - T0, 3)/256*4096));
     strip.setBrightness((k * pow(t - T0, 3)/256*255));
     for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
      strip.setPixelColor(ledNumber, 255, 0, 0);
      }
      strip.show();
      t++;
      delayMicroseconds(1);
      yield();
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
          // pwm.setPWM(COLOOR,0,(k * pow(Tmax - t, 3))/256*4096);
         strip.setBrightness((k * pow(Tmax - t, 3))/256*255);
         for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
          strip.setPixelColor(ledNumber, 255, 0, 0);
          }
          strip.show();
        
          t++;
          delayMicroseconds(1);
        }
      }
    }
    c++; //got to next two frames
  }
  }
  Serial.print("!");
}
void RGBLoop(){
  client.loop();
  for(int j = 0; j < 3; j++ ) {
    // Fade IN
    for(int k = 0; k < 256; k++) {
      switch(j) {
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
    client.loop();
    // Fade OUT
    for(int k = 255; k >= 0; k--) {
      switch(j) {
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
} /// end of main loop ==========================================

void breathe1() {
//Written by: Jason Yandell

float MaximumBrightness = 255;
float SpeedFactor = 0.003; // I don't actually know what would look good
float StepDelay = 5; // ms for a step delay on the lights
float Duration = 3000; // number of breath in this call

// Make the lights breathe
for (int i = 0; i < Duration; i++) {
// Intensity will go from 10 - MaximumBrightness in a "breathing" manner
float intensity = MaximumBrightness /2.0 * (1.0 + sin(SpeedFactor * i));
strip.setBrightness(intensity);
// Now set every LED to that color
for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
strip.setPixelColor(ledNumber, 0, 0, 255);
}
strip.show();
delay(0); // to avoif watchdog firing on long loop 
}
}

void breathe2() {
//Written by: Jason Yandell
// update FJA : adjust the cycle so that we start with dark and have always full cycles so that you have seamless repetition of the effect 
// execution of this function produces 1 cycle

float MaximumBrightness = 255;
float Duration = 3000; // number of steps so duration of 1 breath
float pi = 3.1416;

// Make the lights breathe
// adjust duration
Duration = Duration * 60 / NUM_LEDS;
for (int i = 0; i < Duration; i++) {
// Intensity will go from 0 to MaximumBrightness in a "breathing" manner
float intensity = MaximumBrightness /2.0 * (1.0 + sin(pi*1.5+2*pi/Duration * i));
strip.setBrightness(intensity);
// Serial.print(intensity);Serial.print("/");
// Now set every LED to that color
for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
strip.setPixelColor(ledNumber, 0, 0, 255); // this is blue
}
strip.show();
delay(0); // to avoif watchdog firing on long loop 
}
}
void breathe3() {
  // new algorythm : sinusoid is too simple 
  // based on EXP(SIN(t/fullduration*PI))-2
  // the -2 adjust the cycle so that we start with dark and have always full cycles so that you have seamless repetition of the effect 
  // execution of this function produces 1 cycle
  
  float MaximumBrightness = 200;
  float Duration = 4000; // number of steps so duration of 1 breath with 60 leds, so need to be adjusted below with the real number of LED (NUM_LEDS)
  float pi = 3.1416;
  int StopBreath = 3000; // delay between 2 breaths in milliseconds
  
  // Make the lights breathe
  Duration = Duration * 60 / NUM_LEDS;
  for (int i = 0; i < Duration; i++) {
  // Intensity will go from 0 to MaximumBrightness in a "breathing" manner
  // float intensity = MaximumBrightness /2.0 * (1.0 + sin(pi*1.5+2*pi/Duration * i));
  float intensity = MaximumBrightness /2.0 * (1.0 + exp(sin(i /Duration * pi) )-2);
  strip.setBrightness(intensity);
  // Serial.println(intensity);
  // Serial.print(intensity);Serial.print("/");
  // Now set every LED to that color
  for (int ledNumber=0; ledNumber<NUM_LEDS; ledNumber++) {
  strip.setPixelColor(ledNumber, 0, 0, 255); // this is blue
  }
  strip.show();
  delay(0); // to avoif watchdog firing on long loop 
  }
  delay(StopBreath);
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
    delay(0); // to avoid watchdog to fire
  }
}
void SnowSparkle(uint32_t snowcolor) {
  int j;
  strip.setBrightness(200);
  strip.fill(snowcolor);
  strip.show();
  for (int i = 0 ;i<10;i++){
  j=random(NUM_LEDS);
  strip.setPixelColor(j,strip.Color(255,255,255));
  strip.show();
  delay(random(500));
  strip.setPixelColor(j,snowcolor);
  strip.show();
  // for debug Serial.print(j);Serial.print(" ");
  }
  // Serial.println();
}
  
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    client.loop();
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
void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause)
{
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

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay)
{

  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    client.loop();
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
    delay(0);
  }
  showStrip();
}





float breathe(){
  yield(); // to avoid soft reset on Wemos
  float val = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
  return val;

}

void tstI2C(){ // test I2C and show what was found
  byte error, address;
  int nDevices;
  Serial.println("Hop");
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
 //   Serial.println(address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 //Serial.println(address);
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

  
