/*
 
portal-server 0.1:
                  hier jetzt mqtt - settings,
                  OTA - settings einbinden, um bisherige einbindung nicht zu verlieren.
                  einen reconfig-trigger mit Button zum Testen einbauen
                  
                  wie bringe ich variablen aus currentSettings auf webSeite?
                  von webSeite in newSettings?
                
                  wie rufe ich widgets horizontalScroller auf
                
                  moonSketch hier einfügen



portalino 0.3  : grob zusammengefügt aus moonLamp 05.4m, esp8266-vindriktning und portal-server
                 nur 3 der 6 felder in Config sind ausgeführt, die anderen (mqtt-port, OTA-password und OTA-port laufen aus default in Config.h mit
                 code ist ziemlich durcheinandergewürfelt
                 OTA könnte gehen
                 mqtt scheint zu gehen, braucht noch Abfrage connected? for Zugriffen
                 TBD: einstellungsseite
                 
portalino 0.7  : hat die Einstellungsseite grob funktional
                 OTA und mqtt laufen
                 roleSweet als Weiterentwicklung von roleSoft
                 f moon und moonie

                 wishList: 
portalino 0.10             
                           ja if (isMqttConnected()) vor MQTT Zugriffen, so dass simple access locker laeuft
                           ja Conf erweitern um die diversen parameter, so dass das Ding immer mit den zuletzt gewählten Einstellungen startet
                           ja bright: webServerChanges analog callbackChanges
                           ja mDNS auf identifier.local

                           Todo fehlende fields ergänzen oder page auf dem embAjax
 */
 
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>
#include <ESP8266WiFi.h>          
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"    

#include <ESP8266mDNS.h>
     
#include "FastLED.h"

#include "Config.h"
#include <Streaming.h>

#include <EmbAJAX.h>

/********************************************/
#define moon true
//#define moonie true

const char* version = "portaLuna 0.10.1";


/********************************************/
#ifdef moon

char identifier[24] = "moon";


  const char* state_topic = "led/moon/state";
  const char* set_topic   = "led/moon/set";
  const char* dbg_topic   = "led/moon/dbg";

 #define NUM_LEDS    28 
 #define DATA_PIN    4

#endif

#ifdef moonie

char identifier[24] = "moonie";

  const char* state_topic = "led/moonie/state";
  const char* set_topic   = "led/moonie/set";
  const char* dbg_topic   = "led/moonie/dbg";
  
 #define NUM_LEDS    28 
 #define DATA_PIN    4

#endif

/********************************************/

uint8_t mqttRetryCounter = 0;
bool shouldSaveConfig = false;

bool isInStartup = true;

const char* on_cmd = "ON";
const char* off_cmd = "OFF";

/*********************************env-info************************************************/
// not used here

const char* env_topic = "r48/env/#";
String phaseOfDay = "n/a";
bool   isDuster = true;
bool da_jmd = true;
String werDa = "";


/****************************     **************************************************/

// timed_loop
#define INTERVAL_1    100
#define INTERVAL_2  30000
#define INTERVAL_3  60000
#define INTERVAL_4 180000

unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;

// extraWurst für colorLoop
unsigned long INTERVAL_0 = 60;
unsigned long time_0     =  0;



// debug messages
const int numMsg=80;
int msgCount=0;
String msg=".";
String arrMessages[numMsg];



/****************************************FOR JSON***************************************/
const int BUFFER_SIZE = JSON_OBJECT_SIZE(32);



/*********************************** FastLED Defintions ********************************/

#define CHIPSET     WS2812B
#define COLOR_ORDER GRB

struct CRGB leds[NUM_LEDS];
struct CRGB fadeTargets[NUM_LEDS];
struct CRGB fadeNormals[NUM_LEDS];


unsigned long  INTERVAL_anim = 2000;
int animFactor = 1;
unsigned long time_anim = 0;


bool isStalled = false;
bool firstRun  = true;


// (712)
struct CHSV roleSpool[NUM_LEDS];

const int faderLength=30;
int arrFader[faderLength];
struct CHSV roleOrg[faderLength];
struct CHSV roleZiel[faderLength];
int newFade=0;

/********************************************/


  byte brightness = 32;
  bool stateOn = true;
  
  CHSV baseColor(10,232,209);
  CHSV currColor(10,232,209);
  int hueRange=81;
  int satRange=63;
  int quart=27;
  int direction=1;
  
  int stepTime=50;
  int stepTimeStore=stepTime;
  const char* effect = "roleSweet";
  String effectString = "roleSweet";

  uint8_t probab = 15;


float hueStep;
float hue;
float satStep;
float sat;
int delayMultiplier = 1;

/******************************** GLOBALS for animations *******************************/
int roleStepTime = 20;

int blockStart;
int blockLength;

bool lockForAnim=false;
int loopCount=0;
int animCount=0;
unsigned long lastLoop=millis();


/**********   instances ************************/

//std::unique_ptr<ESP8266WebServer> webServer;

WiFiClient wifiClient;
PubSubClient mqClient(wifiClient);

/************** embAjax **************************/

// Set up web server, and register it with EmbAJAX. Note: EmbAJAXOutputDriverWebServerClass is a
// convenience #define to allow using the same example code across platforms
EmbAJAXOutputDriverWebServerClass server(80);
EmbAJAXOutputDriver driver(&server);

// Define the main elements of interest as variables, so we can access to them later in our sketch.
#define BUFLEN 30
EmbAJAXSlider hue_slider("hue_slider", 0, 255, 0);
EmbAJAXSlider sat_slider("sat_slider", 0, 255, 0);
EmbAJAXSlider val_slider("val_slider", 0, 255, 0);
EmbAJAXSlider bri_slider("bri_slider", 0, 255, 0);
EmbAJAXSlider step_slider("step_slider", 0, 500, 0);
EmbAJAXSlider prob_slider("prob_slider", 0, 100, 0);

const char* state_opts[] = {"OFF","ON"};
EmbAJAXRadioGroup<2> state_radio("state_radio", state_opts);
//EmbAJAXMutableSpan state_radio_d("state_radio_d");

const char* effect_opts[] = {"roleSweet","roleSoft","roleSlow", "solid","shiftBand", "rangeWave","satRangeWave"};
EmbAJAXRadioGroup<7> effect_radio("effect_radio", effect_opts);

//
//EmbAJAXColorPicker color("color", 0, 255, 255);
//EmbAJAXMutableSpan color_d("color_d");
//char color_d_buf[BUFLEN];

// Define two very simple pages "page1" and "page2"
MAKE_EmbAJAXPage(page1, "porta luna WebConfig", "",
  
  new EmbAJAXStatic("<h1>porta luna WebConfig</h1><p>&nbsp;</p><p>Hue: "),
  &hue_slider,
  new EmbAJAXStatic("</p><p>Saturation: "),
  &sat_slider,
  new EmbAJAXStatic("</p><p>Value: "),
  &val_slider,
  new EmbAJAXStatic("</p><p>Brightness: "),
  &bri_slider,
  new EmbAJAXStatic("</p><p>&nbsp;</p><p>step time:"),
  &step_slider,
  new EmbAJAXStatic("</p><p>density:"),
  &prob_slider,
  new EmbAJAXStatic("</p><p>&nbsp;</p><p>effect:"),
  &effect_radio,  
  new EmbAJAXStatic("</p><p>power:"),
  &state_radio,
  new EmbAJAXStatic("</p><p>&nbsp;</p><p>&nbsp;</p>")
)

//MAKE_EmbAJAXPage(page2, "EmbAJAX example - Two Pages II", "",
//  new EmbAJAXStatic("<h1>Page 2</h1><p>This slider is shared across pages: "),
//  &shared_slider,
//  new EmbAJAXStatic("</p><p>This display shows the value of the bottom slider on page 1: "),
//  &other_slider_display,
//  new EmbAJAXStatic("</p><p>Link to <a href=\"/\">page 1</a> (you can open this directly, or in a tab of your browser).</p>")
//)



/*************************** setup **************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);



  Config::load();
  delay(300);

  
  //Serial.setDebugOutput(true);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  wifiManager.resetSettings();

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqtt_server("mqtt_server", "mqtt server", Config::mqtt_server, sizeof(Config::mqtt_server));
  WiFiManagerParameter custom_mqtt_user("mqtt_username", "MQTT username", Config::mqtt_username, sizeof(Config::mqtt_username));
  WiFiManagerParameter custom_mqtt_pass("mqtt_password", "MQTT password", Config::mqtt_password, sizeof(Config::mqtt_password));
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  WiFi.hostname(identifier);




  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //wifiManager.autoConnect("AutoConnectAP");
  //or use this for auto generated name ESP + ChipID
  wifiManager.autoConnect(identifier);
  mqClient.setClient(wifiClient);

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  
  strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
  strcpy(Config::mqtt_username, custom_mqtt_user.getValue());
  strcpy(Config::mqtt_password, custom_mqtt_pass.getValue());

  if (shouldSaveConfig) {
      Config::save();
  } else {
      // For some reason, the read values get overwritten in this function
      // To combat this, we just reload the config
      // This is most likely a logic error which could be fixed otherwise
      Config::load();
  }

  // read and set the stored params from Config
 
  baseColor.h = Config::hue;
  baseColor.s = Config::sat;
  baseColor.v = Config::val;
  brightness = Config::bright;
  probab = Config::probab;
  stepTime = Config::stepTime;
  byte effectOption = Config::effectOption;
  effect = effect_opts[effectOption];
  effectString = String( effect ); 
  stateOn = Config::stateOn;
    
  


  // Tell the server to serve the two pages at root, and at "/page2", respectively.
  // installPage() abstracts over the (trivial but not uniform) WebServer-specific instructions to do so
  driver.installPage(&page1, "/", updateUI);
 
  server.begin(); 
  initValues();
  updateUI(); // to initialize display
  
  Serial.println("HTTP server started");
  Serial.println(WiFi.localIP());


  // Bei Android-Geräten wird der mDNS oft nicht unterstützt, dann muss auf die IP-Adresse zurückgegriffen werden
  
  if (MDNS.begin(identifier)) {
    MDNS.addService("http", "tcp", 80);
    //Serial.println("DNS gestartet, erreichbar unter: ");
    //Serial.println("http://" + String(identifier) + ".local/");
    debug("mDNS gestartet",0);
  } 
  else {
    debug("mDNS fail",0);
  }
   
  //Serial << "Hallo Welt\n";

  debug(String(identifier)+" "+version,true);

  randomSeed(analogRead(0));  
  FastLED.addLeds<CHIPSET, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);


  setupMq();
  setupOta();

  colorInit();
     
}

/************************** loop **************************/
void loop() {
  // put your main code here, to run repeatedly:
  //webServer->handleClient();
  driver.loopHook();
  
  mqClient.loop();
  ArduinoOTA.handle();
  MDNS.update();

  timed_loop();

}


void timed_loop() {
   if(millis() > time_0 + INTERVAL_0){
    time_0 = millis();
    
    colorLoop();  
    if (!lockForAnim){
      //sendPulse();  
      checkDebug();
    }      
  } 
  
  if(millis() > time_1 + INTERVAL_1){
    time_1 = millis();
    

    if (!mqClient.connected()) {
      mqReconnect();
    }  
  }
   
  if(millis() > time_2 + INTERVAL_2){
    time_2 = millis();
    
    if (isInStartup){
      // as if stateOn was cycled
      
      stateOn = true;
      firstRun= true;   
      FastLED.setBrightness(brightness);
      FastLED.show();
      isInStartup=false;
    }
    
    if (shouldSaveConfig) {
        Config::save();
        shouldSaveConfig = false;
    } 
  }
 
  if(millis() > time_3 + INTERVAL_3){
     time_3 = millis();
 
     if (!lockForAnim){
      sendState();
     }     

 
  }

  if(millis() > time_4 + INTERVAL_4){
    time_4 = millis();
      
  }
  
 

}




void colorLoop(){
  
   if (stateOn ) {
    if (effectString == "roleSweet"){
      roleSweet();
    }
    if (effectString == "roleSoft"){
      roleSoft();
    }
    if (effectString == "roleSlow"){
      roleSlow();
    }
    else if (effectString == "rangeWave"){
      rangeWave();
    }
    else if (effectString == "satRangeWave"){
      satRangeWave();
    }
    else if (effectString == "shiftBand"){
      shiftBand();
    }

    else if (effectString == "solid"){
      solid();
    } 
    
    FastLED.show();
    
  }
  
  INTERVAL_0 = stepTime * delayMultiplier;
}

/*********************** webServer handlers ***************/

// const char* effect_opts[] = {"roleSlow", "solid","shiftBand"}; 
void initValues(){
  hue_slider.setValue(baseColor.h);
  sat_slider.setValue(baseColor.s);
  val_slider.setValue(baseColor.v);
  bri_slider.setValue(brightness);
  step_slider.setValue(stepTime);
  prob_slider.setValue(probab);
  int pos = stateOn ? 0:1;
  state_radio.selectOption(pos);
  
  if (effectString == "roleSweet"){
    pos = 0;
  }
  else  
if (effectString == "roleSoft"){
    pos = 1;
  }
  else  if (effectString == "roleSlow"){
    pos = 2;
  }
  else if (effectString == "solid"){
    pos = 3;
  }else if (effectString == "shiftBand"){
    pos = 4;
  }
  else if (effectString == "rangeWave"){
    pos = 5;
  }else if (effectString == "satRangeWave"){
    pos = 6;
  }
  effect_radio.selectOption(pos);
  
}

void updateUI() {
  // updateUI() is called whenever one of the controls is changed in the web client.
  //other_slider_display.setValue(itoa(other_slider.intValue(), other_slider_display_buf, 10));
  baseColor.h = hue_slider.intValue();
  baseColor.s = sat_slider.intValue();
  baseColor.v = val_slider.intValue();
  brightness  = bri_slider.intValue();
  probab      = prob_slider.intValue();
  stepTime    = step_slider.intValue();
  
  stateOn = state_opts[state_radio.selectedOption()]=="ON";
  if (stateOn){
    firstRun= true;
  }
    else {
      setColor(0, 0, 0);
    }
  byte effectOption = effect_radio.selectedOption();

  Config::hue = baseColor.h;
  Config::sat = baseColor.s;
  Config::val = baseColor.v;
  Config::bright = brightness;
  Config::probab = probab;
  Config::stepTime = stepTime;
  Config::effectOption = effectOption;
  
  
  effectString = String( effect_opts[effect_radio.selectedOption()] );
  FastLED.setBrightness(brightness);
  FastLED.show();

  shouldSaveConfig = true;
  sendState();
}


/********************************************************/

void saveConfigCallback() {
    shouldSaveConfig = true;
}

/************************ restart *************************/

void seppuku(String msg){
  debug("about to restart "+msg,true);
  checkDebug();
  delay(50);
  ESP.restart();
}

/************************ illumination helpers ***************************/


void colorInit(){
  effectString=effect;
  hueStep = hueRange*1.0 / quart;

  for (int i = 0; i < NUM_LEDS; i++) {
    currColor.h = ( baseColor.h+i);
    leds[i]=currColor;
  }
  
  FastLED.setBrightness(brightness);
  FastLED.setDither( 0 );
  FastLED.show();
}


void showleds(){

  delay(1);

  if (stateOn) {
    FastLED.setBrightness(brightness);  //EXECUTE EFFECT COLOR
    FastLED.show();
    if (stepTime > 0 && stepTime < 500) {  //Sets animation speed based on receieved value
      FastLED.delay(stepTime / 10 * delayMultiplier); //1000 / transitionTime);
      //delay(10*transitionTime);
    }
  }
}


/********************************** START Set Color*****************************************/
void setColor(int inR, int inG, int inB) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].red   = inR;
    leds[i].green = inG;
    leds[i].blue  = inB;
  }

  FastLED.setBrightness(brightness);
  FastLED.show();

}

void fillHsv(CHSV hsv){
  for (int i = 0; i < NUM_LEDS; i++) {
    hsv2rgb_rainbow( hsv, leds[i]);
  }
  FastLED.setBrightness(brightness);
  
}



// Helper function that blends one uint8_t toward another by a given amount
void nblendU8TowardU8( uint8_t& cur, const uint8_t target, uint8_t amount)
{
  if( cur == target) return;
  
  if( cur < target ) {
    uint8_t delta = target - cur;
    delta = scale8_video( delta, amount);
    cur += delta;
  } else {
    uint8_t delta = cur - target;
    delta = scale8_video( delta, amount);
    cur -= delta;
  }
}

// Blend one CRGB color toward another CRGB color by a given amount.
// Blending is linear, and done in the RGB color space.
// This function modifies 'cur' in place.
CRGB fadeTowardColor( CRGB& cur, const CRGB& target, uint8_t amount)
{
  nblendU8TowardU8( cur.red,   target.red,   amount);
  nblendU8TowardU8( cur.green, target.green, amount);
  nblendU8TowardU8( cur.blue,  target.blue,  amount);
  return cur;
}

// Fade an entire array of CRGBs toward a given background color by a given amount
// This function modifies the pixel array in place.
void fadeTowardColor( CRGB* L, uint16_t N, const CRGB& bgColor, uint8_t fadeAmount)
{
  for( uint16_t i = 0; i < N; i++) {
    fadeTowardColor( L[i], bgColor, fadeAmount);
  }
}

/***************************************** effect drivers ***********************************************/
// roleSlow()
// roleSweet()
// roleSoft()
// rangeWave()
// satRangeWave()
// shiftBand()
// solid()

void roleSlow(){
  int segLen=30;
  if (firstRun){
    //debug("roleIn firstrun",false);
    // steps timen auf 60 leds / sec = 1000 msec/60 =1,66 ~17
    //stepTime=roleStepTime;


    // loopCnt zuruecksetzen
    loopCount=0;
    
    
    
    int p1=30;
    int p2=8; 
    int p3=10;
    
    int d1=-1;
    int d2=-1;
    int d3=-1;
    
    int hn=baseColor.h;
    int sn=baseColor.s;
    int vn=baseColor.v;
    int hv=baseColor.h;
    int sv=baseColor.s;
    int vv=baseColor.v;   
    
    //debug("roleIn firstRun  baseColor: "+String(baseColor.h)+" sn: "+String(baseColor.s)+" vn: "+String(baseColor.v),false);

    //int hv,sv,vv;

    for (int i=0;i<segLen;i++){
      if (i%p1==0){d1*=-1;hn=hv;}
       hv = hn + d1*i%p1;
      if (i%p2==0){d2*=-1;sn=sv;}
       sv = sn + d2*i%p2;
      if (i%p3==0){d3*=-1;vn=vv;}
       vv = vn + d3*i%p3;

      roleSpool[i]= CHSV(hv,sv,vv);
    }

    firstRun=false;

  }

  // the wheel is roling
  //debug("roleIn loopCount "+String(loopCount),false);
  if ( loopCount < NUM_LEDS){
    lockForAnim=true;
    int srcPos = loopCount % segLen;
    //debug("roleIn loopCount "+String(loopCount)+" srcPos: "+String(srcPos)+" h: "+String(roleSpool[srcPos].h),false);
    leds[loopCount] = roleSpool[srcPos];
    FastLED.show();
      loopCount++;

    //if (loopCount % 60 == 1){
      //debug("roleIn loopCount "+String(loopCount),false);
    //}
    
  } 
  else {
    
    lockForAnim=false;
    //debug("roleIn loopCount else "+String(loopCount),false);
    //stepTime=240;
    randomSeed(analogRead(0));

    CRGB aColor = CHSV(random8(255), 180+random8(74), 180+random8(74));

      // hartcodierte haeufigkeit von 40 auf ...
  
      if ( random8() < 5) {
        int aPos=random16(NUM_LEDS-1)+1;
        // knallt neuen Farbspratzel rein
        leds[ aPos ] += CRGB(aColor.r, aColor.g, aColor.b);

        // nimmt die Pos in arrFader zum ausblenden auf
        arrFader[newFade] = aPos;
        newFade++;
        if (newFade>=faderLength){newFade=1;}
      }

      // alle Pos in arrFader: fade down
      for (int i=0;i<faderLength;i++){
        int aPos = arrFader[i];
        if (aPos > 0){
          leds[aPos].fadeToBlackBy(4);
        }
      }

      // alle, die dunkler als limit auf allen 3 kanälen sind springen auf urspruengliche farbe zurueck
      int limit=10;
      for (int i=0;i<NUM_LEDS;i++){
        if (leds[ i ].r<=limit &&leds[ i ].g<=limit && leds[ i ].b<=limit){
          int srcPos = i % segLen;
          leds[i] = roleSpool[srcPos];
        }
      }
  }

  
}



/*
 * Aufbau der Reihe wie gehabt in HSV
 * fadeColor zu Color habe ich aber nur für RGB.
 roleOrg , roleZiel und leds ist Ist
 zielfarben für farbsprazzel und fadeBack landen in roleZiel

 conf:
  amount  : wie granular sind die Fades?
  probab  : die wahrscheinlichkeit, dass eine Led einen Farbsprung macht
  limit   : pixel < limit auf allen drei kanaelen (rgb) swappt zurueck
  
 */
void roleSweet(){
  int segLen=30;
  uint8_t amount = 1;
  uint8_t limit=10;
  
  if (firstRun){

    stepTimeStore=stepTime;
    stepTime=160;
    

    // loopCnt zuruecksetzen
    loopCount=0;
    roleStepTime = 160;
    
    
    int p1=30;
    int p2=8; 
    int p3=10;
    
    int d1=-1;
    int d2=-1;
    int d3=-1;
    int offset = 20;
    
    int hn=baseColor.h;
    hn - offset;
    if (hn<0){
      hn += 256;
    }
    
    int sn=baseColor.s;
    int vn=baseColor.v;
    int hv=hn;
    int sv=sn;
    int vv=vn;   
    
    //debug("roleIn firstRun  baseColor: "+String(baseColor.h)+" sn: "+String(baseColor.s)+" vn: "+String(baseColor.v),false);

    //int hv,sv,vv;

    for (int i=0;i<segLen;i++){
      if (i%p1==0){d1*=-1;hn=hv;}
       hv = hn + d1*i%p1;
      if (i%p2==0){d2*=-1;sn=sv;}
       sv = sn + d2*i%p2;
      if (i%p3==0){d3*=-1;vn=vv;}
       vv = vn + d3*i%p3;

      roleSpool[i]  = CHSV(hv,sv,vv);
      
    }
    // noch 1 init
    for (int i=0;i<faderLength;i++){
        arrFader[i]= -1;
    }
    firstRun=false;

  }

  // the wheel is roling
  //debug("roleIn loopCount "+String(loopCount),false);
  // die vorberechnete Reihe wird hier 0 bis NUM_LEDS-1 eingeschrieben. In leds und gleich auch in fadeTargets und fadeNormals 
  if ( loopCount < NUM_LEDS){
    lockForAnim=true;
    int srcPos = loopCount % segLen;
    //debug("roleIn loopCount "+String(loopCount)+" srcPos: "+String(srcPos)+" h: "+String(roleSpool[srcPos].h),false);
    leds[loopCount] = roleSpool[srcPos];
    fadeTargets[loopCount] = roleSpool[srcPos];
    fadeNormals[loopCount] = roleSpool[srcPos];
    FastLED.show();
    loopCount++;

    
  } 
  else {
    stepTime = stepTimeStore;
    lockForAnim=false;
    
    randomSeed(analogRead(0));

    CRGB aColor = CHSV(random8(255), 180+random8(74), 180+random8(74));

      // hartcodierte haeufigkeit von 40 auf ...
  
      if ( random8() < probab) {
        int aPos=random16(NUM_LEDS-1)+1;
        // knallt neuen Farbspratzel rein
        //leds[ aPos ] += CRGB(aColor.r, aColor.g, aColor.b);
        fadeTargets[ aPos ] = CRGB(aColor.r, aColor.g, aColor.b);
        //fadeTargets[ aPos ] = CRGB(255, 255, 255);

      }

      for (int i=0;i<NUM_LEDS;i++){
        if (leds[i] != fadeTargets[ i ]){
            //CRGB aCol =fadeTowardColor(leds[i],fadeTargets[ i ],amount);
            leds[i] =fadeTowardColor(leds[i],fadeTargets[ i ],amount);
            } else {
              if (leds[i] != fadeNormals[ i ]){
                fadeTargets[ i ] = fadeNormals[ i ];
              }
            }
            
      }

      // alle, die dunkler als limit auf allen 3 kanälen sind springen auf urspruengliche farbe zurueck
      limit=20;
      for (int i=0;i<NUM_LEDS;i++){
        if (leds[ i ].r<=limit &&leds[ i ].g<=limit && leds[ i ].b<=limit){
          fadeTargets[ i ] = fadeNormals[ i ];
        }
      }
  }

  
}


void roleSoft(){
  int segLen=30;
  uint8_t amount = 1;
  uint8_t probab = 25;
  uint8_t limit=10;
  
  if (firstRun){
    //debug("roleIn firstrun",false);
    // steps timen auf 60 leds / sec = 1000 msec/60 =1,66 ~17
    stepTimeStore=stepTime;
    stepTime=160;
    

    // loopCnt zuruecksetzen
    loopCount=0;
    roleStepTime = 160;
    
    
    int p1=30;
    int p2=8; 
    int p3=10;
    
    int d1=-1;
    int d2=-1;
    int d3=-1;
    int offset = 20;
    
    int hn=baseColor.h;
    hn - offset;
    if (hn<0){
      hn += 256;
    }
    
    int sn=baseColor.s;
    int vn=baseColor.v;
    int hv=hn;
    int sv=sn;
    int vv=vn;   
    
    //debug("roleIn firstRun  baseColor: "+String(baseColor.h)+" sn: "+String(baseColor.s)+" vn: "+String(baseColor.v),false);

    //int hv,sv,vv;

    for (int i=0;i<segLen;i++){
      if (i%p1==0){d1*=-1;hn=hv;}
       hv = hn + d1*i%p1;
      if (i%p2==0){d2*=-1;sn=sv;}
       sv = sn + d2*i%p2;
      if (i%p3==0){d3*=-1;vn=vv;}
       vv = vn + d3*i%p3;

      roleSpool[i]  = CHSV(hv,sv,vv);
      
    }
    // noch 1 init
    for (int i=0;i<faderLength;i++){
        arrFader[i]= -1;
    }
    firstRun=false;

  }

  // the wheel is roling
  //debug("roleIn loopCount "+String(loopCount),false);
  // die vorberechnete Reihe wird hier 0 bis NUM_LEDS-1 eingeschrieben. In leds und gleich auch in fadeTargets und fadeNormals 
  if ( loopCount < NUM_LEDS){
    lockForAnim=true;
    int srcPos = loopCount % segLen;
    //debug("roleIn loopCount "+String(loopCount)+" srcPos: "+String(srcPos)+" h: "+String(roleSpool[srcPos].h),false);
    leds[loopCount] = roleSpool[srcPos];
    fadeTargets[loopCount] = roleSpool[srcPos];
    fadeNormals[loopCount] = roleSpool[srcPos];
    FastLED.show();
    loopCount++;

    
  } 
  else {
    stepTime = stepTimeStore;
    lockForAnim=false;
    
    randomSeed(analogRead(0));

    CRGB aColor = CHSV(random8(255), 180+random8(74), 180+random8(74));

      // hartcodierte haeufigkeit von 40 auf ...
  
      if ( random8() < probab) {
        int aPos=random16(NUM_LEDS-1)+1;
        // knallt neuen Farbspratzel rein
        //leds[ aPos ] += CRGB(aColor.r, aColor.g, aColor.b);
        fadeTargets[ aPos ] += CRGB(aColor.r, aColor.g, aColor.b);
        //fadeNormals[ aPos ] += CRGB(aColor.r, aColor.g, aColor.b);
        
        // nimmt die Pos in arrFader zum ausblenden auf
        arrFader[newFade] = aPos;
        newFade++;
        if (newFade>=faderLength){newFade=0;}
      }

      // alle Pos in arrFader: fade down
      for (int i=0;i<faderLength;i++){
        int aPos = arrFader[i];
        if (aPos > -1){
          // vgl leds[aPos] mit fadeTargets[aPos]
          // sind sie gleich, ist ein fade erfolgreich abgeschlossen
          // sonst: CRGB aCol =fadeTowardColor(leds[aPos],fadeTargets[ aPos ],amount
          if (leds[aPos] != fadeTargets[ aPos ]){
            //CRGB aCol =fadeTowardColor(leds[aPos],fadeTargets[ aPos ],amount);
            leds[aPos] =fadeTowardColor(leds[aPos],fadeTargets[ aPos ],amount);
            }
            else {
              if (leds[aPos] != fadeNormals[ aPos ]){
                fadeTargets[ aPos ] = fadeNormals[ aPos ];
              } else {
                arrFader[i] =-1;
              }
            }
          //leds[aPos].fadeToBlackBy(4);
        }
      }

      // alle, die dunkler als limit auf allen 3 kanälen sind springen auf urspruengliche farbe zurueck
      
      for (int i=0;i<NUM_LEDS;i++){
        if (leds[ i ].r<=limit &&leds[ i ].g<=limit && leds[ i ].b<=limit){
          fadeTargets[ i ] = fadeNormals[ i ];
          int srcPos = i % segLen;
          leds[i] = roleSpool[srcPos];
          // nimmt die Pos in arrFader zum ausblenden auf
          arrFader[newFade] = i;
          newFade++;
          if (newFade>=faderLength){newFade=0;}
        }
      }
  }

  
}



// hsv2rgb_rainbow( hsv, rgb);
// CHSV rgb2hsv_rainbow(CRGB input_color)

void rangeWave(){
    unsigned long now = millis();
//    Serial << (now - lastLoop) << " " << stepTime << "\n";
//    delay(1000);
    if (now - lastLoop > stepTime) {
        lastLoop = now;
        loopCount++;
        loopCount = loopCount % NUM_LEDS;
        
        hue = baseColor.h*1.0;
        hueStep = hueRange*1.0 / quart;
        currColor = baseColor;
        int pos=0;

        for (int i = 0; i < quart*4; i++) {
          if (i<quart){direction=1;}
          if (i>=quart && i<quart*2){direction=-1;}
          if (i>=quart*2 && i<quart*3){direction=1;}
          if (i>=quart*3 && i<quart*4){direction=-1;}
          
          hue += hueStep*direction;
          currColor.h = (int) hue;
          pos=(loopCount+i) % NUM_LEDS;
          //leds[pos]=currColor;
          
          hsv2rgb_rainbow( currColor, leds[pos]);
        
        }
    }
}


void satRangeWave(){
    unsigned long now = millis();
//    Serial << (now - lastLoop) << " " << stepTime << "\n";
//    delay(1000);
    if (now - lastLoop > stepTime) {
        lastLoop = now;
        loopCount++;
        loopCount = loopCount % NUM_LEDS;
        
        
        sat = baseColor.s*1.0;
        satStep =satRange*1.0 /quart;
       
        
        currColor = baseColor;
        int pos=0;

        for (int i = 0; i < quart*4; i++) {
          if (i<quart){direction=1;}
          if (i>=quart && i<quart*2){direction=-1;}
          if (i>=quart*2 && i<quart*3){direction=-1;}
          if (i>=quart*3 && i<quart*4){direction=1;}
          
          sat += satStep*direction;
          currColor.s = (int) sat;
          pos=(loopCount+i) % NUM_LEDS;
          //leds[pos]=currColor;
          hsv2rgb_rainbow( currColor, leds[pos]);
        }
    }
}


void shiftBand(){
  for (int i = 0; i < NUM_LEDS; i++) {
    currColor.h = ( baseColor.h+i+loopCount);
    leds[i]=currColor;
  }
  loopCount++;
}


void solid() {
  if (firstRun){
    FastLED.clear();
    fillHsv(baseColor);
    FastLED.show();
    firstRun=false;
  }
}




/********************************** MQTT MESSAGING *****************************************/
void sendState() {
  StaticJsonDocument<512> root;

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  root["bright"] = brightness;
  root["effect"] = effectString;
  root["stepTime"] = stepTime;
  root["density"] = probab;
  root["quart"] = quart;
  root["hueRange"] = hueRange;
  root["satRange"] = satRange;
  root["vers"] = version;
   
  JsonObject colorHsv = root.createNestedObject("colorHsv");
  colorHsv["h"] = baseColor.h;
  colorHsv["s"] = baseColor.s;
  colorHsv["v"] = baseColor.v;
 
  char buffer[512];
  serializeJson(root, buffer);

  if (isMqttConnected()){
    mqClient.publish(state_topic, buffer, true);
  }
  root.clear();
}

///////////////////////
// time_0 + INTERVAL_0
void sendPulse(){
  StaticJsonDocument<512> doc;
 
  doc["time_0"]=time_0;
  doc["INTERVAL_0"]=INTERVAL_0;

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  if (isMqttConnected()){  
    mqClient.publish(dbg_topic, buffer, n);
  }
  doc.clear();
}
////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// send a message to mq
void sendDbg(String msg){
  StaticJsonDocument<512> doc;
 
  doc["dbg"]=msg;
  

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  if (isMqttConnected()){
    mqClient.publish(dbg_topic, buffer, n);
  }
  doc.clear();
}

// called out of timed_loop async
void checkDebug(){
  if (msgCount>0){
    
    String message = arrMessages[0];

     for (int i = 0; i < numMsg-1; i++) {
      arrMessages[i]=arrMessages[i+1];
    }
    arrMessages[numMsg-1]="";
    msgCount--;
    sendDbg(message);
  }
  
  
}

// stuff the line into an array. Another function will send it to mq later
void debug(String dbgMsg, boolean withSerial){
  //Serial << "dbgMsg: " << dbgMsg <<  "\n";
  
  if (withSerial) {
    Serial.println( dbgMsg );
  }
  if (msgCount<numMsg){
    arrMessages[msgCount]=dbgMsg;
    msgCount++;
  }
  
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {

    
  StaticJsonDocument<512> root;
  deserializeJson(root, payload, length);

  // reset

  if (root.containsKey("seppuku")) {
      String memento = root["seppuku"];
      seppuku(memento);
  }
  
  /*--------------------------------------*/
  // env-info
  /*
   
String phaseOfDay = "n/a";
bool   isDuster = true;
bool da_jmd = true;
String werDa = "";
   */
  
  if (root.containsKey("phase")) {
   const char* phase=root["phase"];
   phaseOfDay = phase;
    //debug("phase: "+phaseOfDay,false);
  }
 
  if (root.containsKey("werDa")) {
    const char*  sDa=root["werDa"];
  }
  
  if (root.containsKey("isDuster")) {
    if (strcmp(root["isDuster"], "ON") == 0) {
      isDuster = true;
    }
    else if (strcmp(root["isDuster"], "OFF") == 0) {
      isDuster = false;
    }
  }
  
  if (root.containsKey("da_jmd")) {
    if (strcmp(root["da_jmd"], "ON") == 0) {
      da_jmd = true;
    }
    else if (strcmp(root["da_jmd"], "OFF") == 0) {
      da_jmd = false;
    }
  }
  
  /*--------------------------------------*/

  if (root.containsKey("effect")) {
      
      String test = root["effect"];
      if (test.indexOf("\\")>1) {
        debug("jaja,da",false);
        effect="kelvin";
        
        return;
      }
      if (test.length() > 12){
        debug("effect strange: |"+test,false);
        return;
      }
    }
    

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      
      if (!stateOn){
        stateOn = true;
        firstRun= true;
      }
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
      setColor(0, 0, 0);
    }
    initValues();
  }
  if (root.containsKey("pause")) {
    if (strcmp(root["pause"], on_cmd) == 0) {
      isStalled = true;
    }
    else if (strcmp(root["pause"], off_cmd) == 0) {
      isStalled = false;

    }
  }

    if (root.containsKey("density")) {
      probab = root["density"];
      initValues();
    }

    if (root.containsKey("hueRange")) {
      hueRange = root["hueRange"];
      //initValues();
    }

    if (root.containsKey("satRange")) {
      satRange = root["satRange"];
      //initValues();
    }
    
     if (root.containsKey("quart")) {
      quart = root["quart"];
      //initValues();
    }  

     if (root.containsKey("animFactor")) {
      animFactor = root["animFactor"];
    }  



    if (root.containsKey("transition")) {
      stepTime = root["transition"];
    }

    if (root.containsKey("stepTime")) {
      stepTime = root["stepTime"];
      initValues();
    }

    if (root.containsKey("bright")) {
      brightness = root["bright"];
      FastLED.setBrightness(brightness);
      initValues();
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
      FastLED.setBrightness(brightness);
      initValues();
    }

 
 
 
    if (root.containsKey("colorHsv")) {
      baseColor.h = root["colorHsv"]["h"];
      baseColor.s = root["colorHsv"]["s"];
      baseColor.v = root["colorHsv"]["v"];
      initValues();

      //debug("callBack colorHsv  baseColor: "+String(baseColor.h)+" sn: "+String(baseColor.s)+" vn: "+String(baseColor.v),false);
      
      //fillHsv(baseColor);
    }

    if (root.containsKey("effect")) {
      // roleIn lass weiter laufen
      if (!(effectString == "roleIn" && stateOn)){      
        firstRun=true;
      }

      // block mit defaults
      if (effectString == "block"){
        
        blockStart=0;
        blockLength = NUM_LEDS;
        
        if (root.containsKey("blockStart")) {
          blockStart=root["blockStart"];
        }
        if (root.containsKey("blockLength")) {
          blockLength=root["blockLength"];
        }
        
      }

      
      effect = root["effect"];
      effectString=effect;
      
      initValues();
    }




  root.clear();
  sendState();
}




/******************************* setup OTA **********************************/

void setupOta(){

  //OTA SETUP
  ArduinoOTA.setPort(Config::OTA_port);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(identifier);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)Config::OTA_password);

  ArduinoOTA.onStart([]() {
    debug("Starting OTA",false);
  });
  ArduinoOTA.onEnd([]() {
    debug("End OTA",false);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debug("OTA Auth Failed",false);
    else if (error == OTA_BEGIN_ERROR) debug("OTA Begin Failed",false);
    else if (error == OTA_CONNECT_ERROR) debug("OTA Connect Failed",false);
    else if (error == OTA_RECEIVE_ERROR) debug("OTA Receive Failed",false);
    else if (error == OTA_END_ERROR) debug("OTA End Failed",false);
  });
  ArduinoOTA.begin();
  
}





/********************************** START mosquitto *****************************************/



void setupMq(){
  // pubsub setup
  
  mqClient.setServer(Config::mqtt_server, Config::mqtt_port);
  mqClient.setKeepAlive(10);
  mqClient.setBufferSize(2048);
  mqClient.setCallback(mqttCallback);
  mqReconnect();  
}


void mqReconnect() {
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
      
        //if (mqClient.connect(identifier, Config::mqtt_username, Config::mqtt_password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
        
        if (mqClient.connect(identifier, Config::mqtt_username, Config::mqtt_password)) {  
            //mqClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
            //publishAutoConfig();

            // Make sure to subscribe after polling the status so that we never execute commands with the default data
            //mqClient.subscribe(MQTT_TOPIC_COMMAND);

           mqClient.subscribe(set_topic); 
           mqClient.subscribe(env_topic);                            
            break;
        }
        delay(5000);
    }
}

bool isMqttConnected() {
    return mqClient.connected();
}
