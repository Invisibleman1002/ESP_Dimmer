/*
 * Bathroom Dimmer by Trey Aughenbaugh
 * This works great with Dimable LED lights.
 * If you have a light switch in your bathroom and replace with a FAN/LIGHT combo, this give you the ability 
 * to dim the light using just a light switch.
 * When you turn on the light, it looks at the LDR, if its below the ldrlimit, its starts the light at 15% brightness.
 * If its above the limite, it starts the light at full brightness.
 * If its dark in your bathroom and you turn the light on and it starts dim.  turn the switch off then on again and it will become bright.
 * AS long as you do that within 6 seconds, you can go from Dim to bright.
 * 
 * Also, it does a fade when the number is higher than previous value.  SO it fades to brightness.
 * When it is going from brighter to dark, it does it immediately.
 * 
 * It uses a NON BLOCKING WIFI / MQTT setup.  Since Wifi is not as important as the light turning on when you want it, it does not wait for Wifi to connect before running.
 * In fact, upon powering, it doesnt connect to internet for at least 10 seconds.  GIve you a chance to flick the switch.
 * When you turn off the switch, the device is ran on battery, so it does into LIGHT_SLEEP mode.
 * Upon turning on the light switch, you wake the device.
 * I imagine after x days of not being used, the batter will eventual die in light sleep mode. 
 * The batteries only purpose really is to allow for flicking the switch.  You could use a nice size cap to keep the ESP on for several seconds.
 * You could also just save the state into memory. You flick the switch on, STATE = ON, flick off, then On.  Check state.  Oh, it was just on.  go BRIGHT, clear state.
 * If you have a newer house that has the correct common /ground wiring, you might not even need this setup.
 * 
 * There are some MQTT messages as well.
 * home/masterbath/light - post to that top and set the brightness.  Range is 0-255.  I would never go below 10, its gets odd and doesnt want to ramp back up.
 * - ESP8266 with Rotary encode can dim/brighten the light posting to this topic
 * .
 * home/masterbath/light/getldr - send any message to that topic and you will get a message(topic ldr) back telling you the LDR value, can be used to set your threshold.
 * 
 * home/masterbath/light/setldr - send a value to here and it gets written to the EEPROM and is read upon boot.
 * 
 * home/masterbath/light/setdim - send a value from 10-255 to set your lowest dim setting you want.  
 * 
 * home/masterbath/light/ldr - getldr publishes to this topic
 * 
 * home/masterbath/light/setreset - reset device, in case of oddities.
 */
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>

//#include "EEPROM.h"
//#include "Ticker.h"
#include <Ticker.h>
//#include "user_interface.h"

extern "C" {
#include "gpio.h"
}
extern "C" {
#include "user_interface.h"
}

#define DEBUG_ERROR true
#define DEBUG_ERROR_SERIAL \
  if (DEBUG_ERROR) Serial

  #define RTCMEMORYSTART 65
/*struct msgData {
  byte dimlow;
  byte ldrlow;
};*/
typedef struct {
  int dimlow;
  int ldrlow;
} msgData;//  __attribute__((aligned(4))); 
//#define msdData_size sizeof(msgData)
//#define eepromstart 10
//#define EEPROM_SIZE  sizeof(msgData)  
msgData mymsgData ={};  //{20, 50};



#define PUBLISH_DELAY   3000

#include "hw_timer.h"
const byte zcPin = 12;
const byte pwmPin = 14;
const byte PoweredPin = P5;  //4 //Detect when POwer is applied to Battery Charger, which means light switch was turned on.
byte fade = 0;
byte state = 1;
byte tarBrightness = 255;
byte curBrightness = 0;
byte zcState = 0; // 0 = ready; 1 = processing;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;

String ip = "";
bool startsend = HIGH;
bool PoweredPinstate = LOW;
unsigned long timelastchecked = 0;
unsigned long timelastslept = 0;
bool letsleep = false;
//const int zeroCrossPin  = 12; // P8;
volatile int i = 0;             // Variable to use as a counter volatile as it is in an interrupt
volatile boolean zero_cross = 0; // Boolean to store a "switch" to tell us if we have crossed zero
//int AC_pin1 = 14;//P9;// Output to Opto Triac
//int dim1 = 0;// Dimming level (0-100)  0 = on, 100 = 0ff

int inc = 1;        // counting up or down, 1=up, -1=down
int freqStep = 83; // make this 83 for 60Hz gridfrequency

#define SSID_MAX_LEN      32
#define PASS_MAX_LEN      64
#define ADA_URL_LEN       20
#define ADA_FEED_LEN      40
#define ADA_ID_LEN        30
#define ADA_KEY_LEN       33

struct configData {
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw[PASS_MAX_LEN];
  char mqtt_server[ADA_URL_LEN];
  char getfeed[ADA_FEED_LEN]; //SubScrbe
  char sendfeed[ADA_FEED_LEN]; //Publish // I'm using this as the BASE and will append to it. for getldr, setldr
  char mqtt_id[ADA_ID_LEN];
  char mqtt_key[ADA_KEY_LEN];
};
configData MyconfigData = { "A11_IOT", "SomebodyKIOT",  "192.168.0.44", "home/masterbath/light","home/masterbath/light","mqtt","mqtt" };

//Static IP address configuration
IPAddress staticIP(192, 168, 0, 102); //ESP static ip
IPAddress gateway(192, 168, 0, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(192, 168, 0, 1);//(8, 8, 8, 8);  //DNS

//WiFiClient espClient;
//PubSubClient client(espClient);
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
Ticker poweredStartWifi;
Ticker saveeeprom;
long previousMillis;


//Ticker timer1(dim_check, 83);
/*
void ICACHE_RAM_ATTR zero_cross_detect() {
  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  i = 0;
  digitalWrite(AC_pin1, LOW);       // turn off TRIAC (and AC)
}
*/
void ICACHE_RAM_ATTR zcDetectISR() {
  if (zcState == 0) {
    zcState = 1;

    if (curBrightness < 255 && curBrightness > 0) {
      digitalWrite(pwmPin, 0);

      int dimDelay = 30 * (255 - curBrightness) + 400;//400
      //DEBUG_ERROR_SERIAL.println(dimDelay);
      hw_timer_arm(dimDelay);
    }
  }
}
void readFromRTCMemory() {
  system_rtc_mem_read(RTCMEMORYSTART, &mymsgData, sizeof(mymsgData));
  DEBUG_ERROR_SERIAL.println(F("mymsgData dim,ldr"));
  DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
  DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
//   mymsgData.ldrlow = 10;
//    mymsgData.dimlow = 40;
//  writeToRTCMemory();
  
 // yield();
}

void writeToRTCMemory() {
    DEBUG_ERROR_SERIAL.println(F("inside save:"));
    DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
    DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
//    DEBUG_ERROR_SERIAL.println(eepromstart);

  system_rtc_mem_write(RTCMEMORYSTART, &mymsgData, sizeof(mymsgData));
 
 // yield();
}
void getconfig(){
//  EEPROM.get(eepromstart, mymsgData);
  delay(10);
  //EEPROM.end(); 
  DEBUG_ERROR_SERIAL.println(F("mymsgData dim,ldr"));
  DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
  DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
 //  mymsgData.ldrlow = 10;
//   mymsgData.dimlow = 40;
   // DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
   // saveconfigtoEE();
  
}
void saveconfigtoEE() {
//void saveconfigtoEE(msgData UpdatedData){
  DEBUG_ERROR_SERIAL.println(F("inside save:"));
    DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
    DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
//    DEBUG_ERROR_SERIAL.println(eepromstart);
//  EEPROM.put(eepromstart, mymsgData);
//  if (EEPROM.commit()) {
 //     DEBUG_ERROR_SERIAL.println("EEPROM successfully committed");
 //   } else {
//      DEBUG_ERROR_SERIAL.println("ERROR! EEPROM commit failed");
 //   }
   delay(200);
 //EEPROM.end();
}
 

void connectToWifi() {
  DEBUG_ERROR_SERIAL.println("Connecting to Wi-Fi...");
  if ( digitalRead(PoweredPin) == LOW)
  { DEBUG_ERROR_SERIAL.println(F("..wait, oh, oh, I woke up to early, nevermind no wifi needed."));
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, subnet, gateway, dns);
  WiFi.begin(MyconfigData.wifi_ssid, MyconfigData.wifi_pw);
  WiFi.setSleep(false);
}

void connectToMqtt() {
  DEBUG_ERROR_SERIAL.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  DEBUG_ERROR_SERIAL.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  DEBUG_ERROR_SERIAL.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  if (letsleep==false){
    DEBUG_ERROR_SERIAL.println("Disconnected from Wi-Fi. but we are retrying.");
  wifiReconnectTimer.once(2, connectToWifi);
  }
}

void onMqttConnect(bool sessionPresent) {
  DEBUG_ERROR_SERIAL.println("Connected to MQTT.");
  DEBUG_ERROR_SERIAL.print("Session present: ");
  DEBUG_ERROR_SERIAL.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MyconfigData.getfeed, 0);
  char fulltopc[45];
  sprintf(fulltopc, "%s/setldr", MyconfigData.sendfeed);
    uint16_t packetIdSub1 = mqttClient.subscribe(fulltopc, 0);
  sprintf(fulltopc, "%s/getldr", MyconfigData.sendfeed);
    uint16_t packetIdSub2 = mqttClient.subscribe(fulltopc, 0);
  sprintf(fulltopc, "%s/setdim", MyconfigData.sendfeed);
    uint16_t packetIdSub3 = mqttClient.subscribe(fulltopc, 0);
  sprintf(fulltopc, "%s/setreset", MyconfigData.sendfeed);
    uint16_t packetIdSub4 = mqttClient.subscribe(fulltopc, 0);

    ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]); 
  char ldr[5] = "/ip";
 // char fulltopc[45];
  strcpy (fulltopc, MyconfigData.sendfeed );
  strcat (fulltopc,ldr);
  DEBUG_ERROR_SERIAL.print(" /ip: ");DEBUG_ERROR_SERIAL.println(fulltopc);
  mqttClient.publish(fulltopc, 0, true, ip.c_str());

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  DEBUG_ERROR_SERIAL.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  DEBUG_ERROR_SERIAL.println("Subscribe acknowledged.");
  DEBUG_ERROR_SERIAL.print("  packetId: ");
  DEBUG_ERROR_SERIAL.println(packetId);
  DEBUG_ERROR_SERIAL.print("  qos: ");
  DEBUG_ERROR_SERIAL.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  DEBUG_ERROR_SERIAL.println("Unsubscribe acknowledged.");
  DEBUG_ERROR_SERIAL.print("  packetId: ");
  DEBUG_ERROR_SERIAL.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  DEBUG_ERROR_SERIAL.println("Publish received.");
  DEBUG_ERROR_SERIAL.print("  topic: ");
  DEBUG_ERROR_SERIAL.println(topic);
  DEBUG_ERROR_SERIAL.print("  qos: ");
  DEBUG_ERROR_SERIAL.println(properties.qos);
  DEBUG_ERROR_SERIAL.print("  dup: ");
  DEBUG_ERROR_SERIAL.println(properties.dup);
  DEBUG_ERROR_SERIAL.print("  retain: ");
  DEBUG_ERROR_SERIAL.println(properties.retain);
  DEBUG_ERROR_SERIAL.print("  len: ");
  DEBUG_ERROR_SERIAL.println(len);
  DEBUG_ERROR_SERIAL.print("  index: ");
  DEBUG_ERROR_SERIAL.println(index);
  DEBUG_ERROR_SERIAL.print("  total: ");
  DEBUG_ERROR_SERIAL.println(total);
  payload[len] = '\0'; 
  String strPayload = String((char*)payload);
 
  char fulltopc[45];
  sprintf(fulltopc, "%s/setldr", MyconfigData.sendfeed);
 // printf ("[%s] is a string %d chars long\n",buffer,n);
//  DEBUG_ERROR_SERIAL.println(fulltopc); 
  if (strcmp(topic, fulltopc)==0){//"setldr"
    DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
    mymsgData.ldrlow = strPayload.toInt();
    //msgData newldr = {mymsgData.dimlow, mymsgData.ldrlow};
    DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
    DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
   // saveconfigtoEE();
   saveeeprom.once(3,writeToRTCMemory);
  //delay(100);
   DEBUG_ERROR_SERIAL.println(F("ldr set!"));
  }
  sprintf(fulltopc, "%s/setdim", MyconfigData.sendfeed);
  if (strcmp(topic, fulltopc)==0){//"setdim"
    DEBUG_ERROR_SERIAL.println(strPayload.toInt());
      mymsgData.dimlow = strPayload.toInt();
   // byte dimlow =50;//strPayload.toInt();
    //byte ldrlow =  mymsgData.ldrlow ;
   // msgData newldr= {dimlow, ldrlow };
    //  mymsgData.ldrlow = 50;
//  mymsgData.dimlow = 15;
    DEBUG_ERROR_SERIAL.println(mymsgData.ldrlow);
    DEBUG_ERROR_SERIAL.println(mymsgData.dimlow);
    //saveconfigtoEE();
    writeToRTCMemory();
    //saveeeprom.once(3,writeToRTCMemory);
  //delay(100);
   DEBUG_ERROR_SERIAL.println(F("dim set!"));
  }  
 sprintf(fulltopc, "%s/getldr", MyconfigData.sendfeed);
 // DEBUG_ERROR_SERIAL.println(fulltopc); 
  if (strcmp(topic, fulltopc)==0){   //getldr
    char sendtopc[45];
      sprintf(sendtopc, "%s/ldr", MyconfigData.sendfeed);
     DEBUG_ERROR_SERIAL.print("  fulltopic: ");DEBUG_ERROR_SERIAL.println(sendtopc);
    char buf [4];
    int adc = analogRead (A0);
   sprintf(buf, "%d", adc);
    mqttClient.publish(sendtopc, 0, true, buf  );
    
  }
 sprintf(fulltopc, "%s/setreset", MyconfigData.sendfeed);
  
  if (strcmp(topic, fulltopc)==0){   //getldr
    ESP.restart();
  }

  
  sprintf(fulltopc, "%s", MyconfigData.sendfeed);
  //  DEBUG_ERROR_SERIAL.println(fulltopc); 
  if (strcmp(topic, fulltopc)==0){ //light
    int lights = strPayload.toInt();
    if(lights > tarBrightness){fade = 1;}else{fade = 0;}
    DEBUG_ERROR_SERIAL.print("  lights: ");DEBUG_ERROR_SERIAL.println(lights); 
    tarBrightness = lights;
  }
  
  
}

void onMqttPublish(uint16_t packetId) {
  DEBUG_ERROR_SERIAL.println("Publish acknowledged.");
  DEBUG_ERROR_SERIAL.print("  packetId: ");
  DEBUG_ERROR_SERIAL.println(packetId);
}


void setup() {
   //pinMode(P2, WAKEUP_PULLUP);// WAKEUP_PULLUP
 // digitalWrite(P2, HIGH); //THis is also the pin that has the button on for programming.  Down to the middle of board = prog mode.  Up towards board edge is use mode.  groudning It means program mode.
  
  DEBUG_ERROR_SERIAL.begin(115200);
//gpio_init();
  DEBUG_ERROR_SERIAL.println(F("dimmer"));
  DEBUG_ERROR_SERIAL.println();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PoweredPin, INPUT);
  pinMode(P6, INPUT); //Wakeup pin
  delay(10);
  PoweredPinstate = digitalRead(PoweredPin);
  DEBUG_ERROR_SERIAL.print("pp: ");
  DEBUG_ERROR_SERIAL.println( PoweredPinstate);
  pinMode(A0, INPUT);
  pinMode(zcPin, INPUT_PULLUP);
 // EEPROM.begin(512);//(EEPROM_SIZE);
  //IF you start with a NEW ESP, dont forget to initialize the data or it starts as 255,255
  delay(10);
  readFromRTCMemory();//getconfig();
  pinMode(pwmPin, OUTPUT);
  attachInterrupt(zcPin, zcDetectISR, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  hw_timer_init(NMI_SOURCE, 0);
  hw_timer_set_func(dimTimerISR);

  //attachInterrupt(digitalPinToInterrupt(PoweredPin), powerISR, CHANGE );
  //attachInterrupt(PoweredPin, powerFISR, FALLING );
  tarBrightness = 255;
  int ldrvalue = readldr();// analogRead(A0);
  if (ldrvalue <= mymsgData.ldrlow) {
    tarBrightness = mymsgData.dimlow;
  }
  DEBUG_ERROR_SERIAL.print("ldrvalue: ");
  DEBUG_ERROR_SERIAL.println(ldrvalue);
  
  //DEBUG_ERROR_SERIAL.println(" starting wifi");

 //gpio_init();
  //https://github.com/esp8266/Arduino/issues/4091#issuecomment-534634763

 /* while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    DEBUG_ERROR_SERIAL.print("status:");
    DEBUG_ERROR_SERIAL.println(WiFi.status());
  }
  DEBUG_ERROR_SERIAL.println(WiFi.localIP());

  DEBUG_ERROR_SERIAL.print(F("IP address: "));
  DEBUG_ERROR_SERIAL.println(WiFi.localIP());
  DEBUG_ERROR_SERIAL.println();

  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);
  //DEBUG_ERROR_SERIAL.println(ip);
  */
  DEBUG_ERROR_SERIAL.print(ESP.getSdkVersion());

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MyconfigData.mqtt_server, 1883);
  mqttClient.setCredentials(MyconfigData.mqtt_id,MyconfigData.mqtt_key);
  
  poweredStartWifi.once(10,connectToWifi); //start wifi is we've bene on for x seconds. 
  
  previousMillis = millis();
  timelastslept = millis();
}

void wakeupCallback() {  // unlike ISRs, you can do a print() from a callback function
//  testPoint_LOW;  // testPoint tracks latency from WAKE_UP_PIN LOW to testPoint LOW
  //printMillis();  // show time difference across sleep; millis is wrong as the CPU eventually stops
  letsleep=false;
 
  DEBUG_ERROR_SERIAL.println(F("Woke from Light Sleep - this is the callback"));

}

void runTest6() {
  DEBUG_ERROR_SERIAL.println(F("\n6th test - Forced Light Sleep, wake with GPIO interrupt"));
  DEBUG_ERROR_SERIAL.flush();
   delay(100);
    WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
//  WiFi.disconnect();
 //  wifi_station_disconnect();
//   wifi_set_opmode(NULL_MODE);
 
  DEBUG_ERROR_SERIAL.println(F("CPU going to sleep, pull WAKE_UP_PIN low to wake it (press the switch)"));
  
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

//  gpio_pin_wakeup_enable(GPIO_ID_PIN(PoweredPin), GPIO_PIN_INTR_ANYEDGE);
  gpio_pin_wakeup_enable(GPIO_ID_PIN(P6), GPIO_PIN_INTR_HILEVEL);
  // only LOLEVEL or HILEVEL interrupts work, no edge, that's an SDK or CPU limitation
 
 //````````` delay(200);

//gpio_pin_wakeup_disable();
  wifi_fpm_set_wakeup_cb(wakeupCallback); // Set wakeup callback (optional)
  wifi_fpm_open();
  wifi_fpm_do_sleep(0xFFFFFFF);  // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
  delay(10);  // it goes to sleep during this delay() and waits for an interrupt
  DEBUG_ERROR_SERIAL.println(F("Woke up!"));  // the interrupt callback hits before this is executed*/
 
  letsleep=false;
}


 
void loop() {
 
  readButton();
 
  if (Serial.available()) {

    int val = Serial.parseInt();
    if (val > 0) {
      Serial.println(tarBrightness);
     
      if(val > tarBrightness){fade = 1;}else{fade = 0;}
       tarBrightness = val;
      Serial.println(tarBrightness);
    } else {
      int ldr = analogRead(A0);
      Serial.print("ldvaluer: ");
      Serial.println(ldr);
    }

  }//else
  // {
  //  int potencia = analogRead(A0);
  //tarBrightness = map(potencia, 0, 1023, 0, 255);
  //     }
  //Put the device to sleep after 60 seconds of inactivity.  save the battery!
  if (PoweredPinstate == LOW && millis() - timelastslept > 60000) { // 1 hour 3600000000

    DEBUG_ERROR_SERIAL.println("TIME TO SLEEP!");
    //ESP.deepSleep(0);
    //WiFiOff();
    tarBrightness = mymsgData.dimlow;
    fade = 0;
    delay(100);
  //  detachInterrupt(zcPin);  //Kept here as I'm seing an oddity upon waking up from sleep.  Light turns on, then off.  

     digitalWrite(pwmPin, 0); 
    //gotosleep();
    timelastslept = millis();
    letsleep = true;
  }
  if (PoweredPinstate == HIGH)
  {
    timelastslept = millis();
  }
  //DEBUG_ERROR_SERIAL.println(digitalRead(PoweredPin ));
  
  if (letsleep==true){
    runTest6();//sleepNow();
    
    if (letsleep == false){
      DEBUG_ERROR_SERIAL.println("WOKE UP AND NEED WIFI!");
    //  attachInterrupt(zcPin, zcDetectISR, RISING);    // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
      //digitalWrite(pwmPin, 0); was on, but 
      tarBrightness = 255;
      int ldrvalue =readldr();// analogRead(A0);
      if (ldrvalue <= mymsgData.ldrlow) {
        tarBrightness = mymsgData.dimlow;
      }
       // 
       poweredStartWifi.once(10,connectToWifi);
       }
    }
}
/*
void WiFiOff()
{
  DEBUG_ERROR_SERIAL.println("---disconnect");
  wifi_station_disconnect();
  DEBUG_ERROR_SERIAL.println("---disconnected");
  bool stopped;
 // do
  {
    stopped = wifi_station_get_connect_status() == DHCP_STOPPED;
    if (!stopped)
    {
      DEBUG_ERROR_SERIAL.println("dhcp not stopped?");
      delay(100);
    }
  }// while (!stopped);
  DEBUG_ERROR_SERIAL.println("---off...");
  wifi_set_opmode(NULL_MODE);
   WiFi.mode(WIFI_STA) ;
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 5);
  //wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  gpio_pin_wakeup_enable(digitalPinToInterrupt(PoweredPin), GPIO_PIN_INTR_HILEVEL);
  gpio_intr_handler_register(wakeup, NULL);
  wifi_fpm_open();
  wifi_fpm_do_sleep(0xFFFFFFF);
}
*/

void dimTimerISR() {
  if (fade == 1) {
    if (curBrightness > tarBrightness || (state == 0 && curBrightness > 0)) {
      --curBrightness;
    }
    else if (curBrightness < tarBrightness && state == 1 && curBrightness < 255) {
      ++curBrightness;
    }
  }
  else {
    if (state == 1) {
      curBrightness = tarBrightness;
    }
    else {
      curBrightness = 0;
    }
  }

  if (curBrightness == 0) {
    state = 0;
    digitalWrite(pwmPin, 0);
  }
  else if (curBrightness == 255) {
    state = 1;
   pwmPin_On();// digitalWrite(pwmPin, 1);
  }
  else {
   pwmPin_On();// digitalWrite(pwmPin, 1);
  }

  zcState = 0;
}

void pwmPin_On(){
  //I'm not turning on the light until 159 ms has passed.  Hopefully long enough to detect the state of the LDR.
if (buttonState == HIGH){
//  DEBUG_ERROR_SERIAL.println(F("dimTimerISR"));
  if ((millis() - lastDebounceTime) > 150){      
   digitalWrite(pwmPin, 1);
   }
  
//  else
 // {
//    digitalWrite(pwmPin, 0);
  }
}

int readldr(){
 int ldrvalue = 0;
 //DEBUG_ERROR_SERIAL.print(F("start:"));
 for (int i = 0; i < 10; i++) {
   //DEBUG_ERROR_SERIAL.println(analogRead(A0));
    ldrvalue += analogRead(A0);
    delay(10);
  }
  //DEBUG_ERROR_SERIAL.println(ldrvalue);
 return ldrvalue / 10; 
}
void readButton() {
  int reading = digitalRead(PoweredPin);
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        DEBUG_ERROR_SERIAL.println(F("POWERED RISING"));
        int ldrvalue = readldr();// analogRead(A0);
        DEBUG_ERROR_SERIAL.print("ldrvalue: ");
        DEBUG_ERROR_SERIAL.println(ldrvalue);
        tarBrightness = 255;
        if (ldrvalue <= mymsgData.ldrlow)
        {
          tarBrightness = mymsgData.dimlow;
        } 
        
        if (PoweredPinstate == LOW && millis() - timelastchecked < 3000)
        {
          tarBrightness = 255;
        }
       // else
       // { tarBrightness = mymsgData.dimlow;
       // }
        DEBUG_ERROR_SERIAL.print("tarBrightness: ");
        DEBUG_ERROR_SERIAL.println(tarBrightness);
        DEBUG_ERROR_SERIAL.print("tarBrightness analog read: ");
        DEBUG_ERROR_SERIAL.println(tarBrightness);
        timelastchecked = millis();
        PoweredPinstate = HIGH;
          if (!WiFi.isConnected()) {
          DEBUG_ERROR_SERIAL.print("button on wifi status: "); DEBUG_ERROR_SERIAL.println(WiFi.status());
          poweredStartWifi.once(10,connectToWifi);
         }
      }
      else
      {
        DEBUG_ERROR_SERIAL.println(F("POWERED FALLING"));
        digitalWrite(pwmPin, 0);
        delay(100);
        tarBrightness = mymsgData.dimlow; 
        PoweredPinstate = LOW;
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
/*
void callback(char* topic, byte* payload, unsigned int length) {
  char msgBuffer[20];

  payload[length] = '\0';            // terminate string with '0'
  String strPayload = String((char*)payload);  // convert to string
  DEBUG_ERROR_SERIAL.print("strPayload =  ");
  DEBUG_ERROR_SERIAL.println(strPayload); //can use this if using longer southbound topics
  DEBUG_ERROR_SERIAL.print("Message arrived [");
  DEBUG_ERROR_SERIAL.print(topic);
  DEBUG_ERROR_SERIAL.print("] ");//MQTT_BROKER
  for (int i = 0; i < length; i++) {
    DEBUG_ERROR_SERIAL.print((char)payload[i]);
  }
  DEBUG_ERROR_SERIAL.println();
  DEBUG_ERROR_SERIAL.println(payload[0]);

int  dim1 = strPayload.toInt();
  tarBrightness = dim1;
  DEBUG_ERROR_SERIAL.println(dim1);
}
*/
