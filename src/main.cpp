#include <Arduino.h>

/**
 * This is an ESP8266 program to measure distance with an VL53L0X
 * infrared ranger, and report over MQTT whether or not some object
 * is within a specified distance window. Its purpose is to send a
 * notification when a package is placed in or removed from a
 * delivery box.
 * 
 * It utilizes the ESP8266's  
 * sleep mode to maximize battery life.  It will wake up at least
 * once per hour to let you know it's still alive.
 *
 * Configuration is done via serial connection.  Enter:
 *  broker=<broker name or address>
 *  port=<port number>   (defaults to 1883)
 *  topicroot=<topic root> (something like buteomont/gate/package/ - must end with / and 
 *  "present", "distance", "analog", or "voltage" will be added)
 *  user=<mqtt user>
 *  pass=<mqtt password>
 *  ssid=<wifi ssid>
 *  wifipass=<wifi password>
 *  mindistance=<minimum presence distance>
 *  maxdistance=<maximum presence distance>
 *  sleepTime=<seconds to sleep between measurements> (set to zero for continuous readings)
 */

#define VERSION "24.11.09.0"  //remember to update this after every change! YY.MM.DD.REV
 
#include <PubSubClient.h> 
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "delivery_reporter.h"
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

VL53L0X sensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  int mindistance=0;  // Item is present if distance is greater than this
  int maxdistance=400;// and distance is less than this
  int sleeptime=10; //seconds to sleep between distance checks
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=false;
  char address[ADDRESS_SIZE]=""; //static address for this device
  char netmask[ADDRESS_SIZE]=""; //size of network
  bool displayenabled=true;    //enable the display
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

unsigned long doneTimestamp=0; //used to allow publishes to complete before sleeping

//This is true if a package is detected. It will be written to RTC memory 
// as "wasPresent" just before sleeping
bool isPresent=false;

//This is the distance measured on this pass. It will be written to RTC memory just before sleeping
int distance=0;

//We should report at least once per hour, whether we have a package or not.  This
//will also let us retrieve any outstanding MQTT messages.  Since the internal millis()
//counter is reset every time it wakes up, we need to save it before sleeping and restore
//it when waking up. To keep from killing our flash memory, we'll store it in the RTC
//memory, which is kept alive by the battery or power supply.
typedef struct
  {
  unsigned long nextHealthReportTime=0;//the RTC for the next report, regardless of readings
  unsigned long rtc=0;        //the RTC maintained over sleep periods
  bool wasPresent=false;      //Package present on last check
  bool presentReported=false; //MQTT Package Present report was sent
  bool absentReported=false;  //MQTT Package Removed report was sent
  long rssi=-99;              //The signal strength
  } MY_RTC;
  
MY_RTC myRtc;

ADC_MODE(ADC_VCC); //so we can use the ADC to measure the battery voltage

IPAddress ip;
IPAddress mask;

/* Like delay() but checks for serial input */
void myDelay(ulong ms)
  {
  ulong doneTime=millis()+ms;
  while(millis()<doneTime)
    {
    checkForCommand();
    delay(10);
    }
  }

void show(uint16_t val, String suffix)
  {
  if (settings.displayenabled)
    {
    display.clearDisplay(); // clear the screen
    display.setCursor(0, 0);  // Top-left corner
    display.print(val); // print the distance on OLED
    display.println(suffix);
    display.display(); // move the buffer contents to the OLED
    }
  }

void show(String msg)
  {
  if (settings.displayenabled)
    {
    display.clearDisplay(); // clear the screen
    display.setCursor(0, 0);  // Top-left corner
    display.println(msg);
    display.display(); // move the buffer contents to the OLED
    }
  }
void initWiFi()
  {
  //Immediately turn off the WiFi radio (it comes on when we wake up)
  WiFi.mode( WIFI_OFF );
//  WiFi.forceSleepBegin();  
  }

void initSensor()
  {
  if (settings.debug)
    {
    Serial.println("Initializing sensor...");
    }

  pinMode(PORT_XSHUT,OUTPUT_OPEN_DRAIN);
  digitalWrite(PORT_XSHUT,HIGH); //Enable the sensor

  uint8 retry=0;

  // If the initialization fails, print the error message and then "fix it"
  // every 10 seconds until it quits failing.
  while (++retry)
    {
    if (sensor.init()) 
      {
      if (settings.debug)
        {
        Serial.println("VL53L0X init OK!");
        show("Sens OK");
        }
      break;
      } 
    else 
      {
      if (retry==1)
        {
        Serial.println("Error initializing VL53L0X sensor!");
        show("Error 1");
        retry=1;
        }
      Serial.print(MAX_HARDWARE_FAILURES-retry);
      Serial.print(". ");
      Serial.println("fix it!");
      myDelay(5000); // Gimme time to read it
      }
    if (retry>=MAX_HARDWARE_FAILURES)
      ESP.reset();
    }
  }

void initSerial()
  {
  Serial.begin(115200);
  Serial.setTimeout(10000);
  
  while (!Serial); // wait here for serial port to connect.
  if (settings.debug)
    {
    Serial.println();
    Serial.println("Serial communications established.");
    }
  }

void initSettings()
  {
  system_rtc_mem_read(64, &myRtc, sizeof(myRtc)); //load the last saved timestamps from before our nap
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom 

  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }

//Take a measurement
int getDistance()
  {  
  distance=sensor.readRangeSingleMillimeters();
  if (distance) 
    {
    if (settings.debug)
      {
      Serial.print("Inst. Dist. (mm): ");
      Serial.println(distance);
      }
    } 
  else 
    {
    Serial.println("Ranging test failed!");
    }
  return distance?distance:-1;
  }

void initDisplay()
  {
  pinMode(PORT_DISPLAY,OUTPUT); //port for display power
  if (settings.displayenabled)
    {
    if (settings.debug)
      {
      Serial.println("Initializing display");
      }
    digitalWrite(PORT_DISPLAY,HIGH); //turn it on
    myDelay(1000); //let the voltage stabilize

    // actual_delay_ms=max(0, REPORT_DELAY_MS-(MEASURE_DELAY_MS*SAMPLE_SIZE));
    // Serial.print("Report delay: ");
    // Serial.println(actual_delay_ms);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
      {
      Serial.println(F("SSD1306 allocation failed"));
      myDelay(5000);
      ESP.reset();  //try again
      }
    display.clearDisplay();       //no initial logo
    display.setTextSize(3);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    if (settings.debug)
      show("Init");
    }
  else
    {
    Serial.println("Display is disabled.");
    digitalWrite(PORT_DISPLAY,LOW); //turn it off
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); // this seems to be the only way to get i2c to work
    //Wire.begin(SDA_PIN, SCL_PIN); //normally handled by the display driver
    }
  }

void setup() 
  {
  delay(1000); //let it stabilize after a deep sleep

  //initialize everything
  initSerial();
  initSettings();
  initWiFi(); 
  initDisplay();
  initSensor(); //sensor should be initialized after display because display sets up i2c

  if (settingsAreValid)
    {      
    if (settings.debug)
      {
      if (!ip.fromString(settings.address))
        {
        Serial.println("IP Address "+String(settings.address)+" is not valid. Using dynamic addressing.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      else if (!mask.fromString(settings.netmask))
        {
        Serial.println("Network mask "+String(settings.netmask)+" is not valid.");
        // settingsAreValid=false;
        // settings.validConfig=false;
        }
      }

    //Get a measurement and compare the presence with the last one stored in EEPROM.
    //If they are the same, no need to phone home. Unless an hour has passed since
    //the last time home was phoned. 
    distance=measure(); 
    show(distance," mm");

    isPresent=distance>settings.mindistance 
                && distance<settings.maxdistance;
    int analog=readBattery();
    
    Serial.print("**************\nThis measured distance: ");
    Serial.print(distance);
    Serial.println(" mm ");

    Serial.print("Package is ");
    Serial.println(isPresent?"present":"absent");
    
    if (settings.debug)
      {
      Serial.print("Last RSSI was ");
      Serial.println(myRtc.rssi);

      Serial.print("Analog input is ");
      Serial.println(analog);

      Serial.print("Battery voltage: ");
      Serial.println(convertToVoltage(analog));
      }

    sendOrNot(); //decide whether or not to send a report
    mqttClient.loop(); //This lets any incoming mqtt messages arrive

    if (settings.displayenabled)
      {
      myDelay(3000); //give someone a chance to read the value
      }
    }
  else
    {
    showSettings();
    }
  }
 
void loop()
  {
  mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
  checkForCommand(); // Check for input in case something needs to be changed to work
  if (settingsAreValid && settings.sleeptime==0) //if sleepTime is zero then don't sleep
    {
    connectToWiFi(); //may need to connect to the wifi
    reconnect();  // may need to reconnect to the MQTT broker
    distance=measure();
    isPresent=distance>settings.mindistance 
              && distance<settings.maxdistance;
    show(distance," mm");
    report();
    myDelay(9000); 
    } 
  else if (settingsAreValid                        //setup has been done and
          && millis()-doneTimestamp>PUBLISH_DELAY) //waited long enough for report to finish
    {
    unsigned long nextReportSecs=(myRtc.nextHealthReportTime-myMillis())/1000;

    Serial.print("Next report in ");
    Serial.print(nextReportSecs/60);
    Serial.print(" minutes and ");
    Serial.print(nextReportSecs%60);
    Serial.println(" seconds.");

    //RTC memory is weird, I'm not sure I understand how it works on the 8266.
    //Reset the health report if it's way wrong
    if (myRtc.nextHealthReportTime-myMillis()>ONE_HOUR)
      {
      Serial.println("------------Fixing bogus health report time-------------");
      myRtc.nextHealthReportTime=myMillis();
      }

    //save the wakeup time so we can keep track of time across sleeps
    myRtc.rtc=myMillis()+settings.sleeptime*1000;
    myRtc.wasPresent=isPresent; //this presence flag becomes the last presence flag
    saveRTC(); //save the timing before we sleep 
    
    WiFi.disconnect(true);
    digitalWrite(PORT_XSHUT,LOW);   //turn off the TOF sensor
    if (settings.displayenabled)
      {
      digitalWrite(PORT_DISPLAY,LOW); //turn off the display only if it is enabled
      }

    unsigned long goodnight=min((unsigned long)settings.sleeptime,nextReportSecs);// whichever comes first
    Serial.print("Sleeping for ");
    Serial.print(goodnight);
    Serial.println(" seconds");
    ESP.deepSleep(goodnight*1000000, WAKE_RF_DEFAULT); 
    } 
  }

/**
 * This routine will decide if a report needs to be sent, and send it if so.
 * The decision is based on whether or not a package was detected for two
 * successive checks. If two successive checks show that the package is 
 * present, or two succesive checks show that the package is not present,
 * then send the report once.  Don't send another report until two successive
 * checks show the opposite or until an hour has passed, whichever comes first.
 * The truth table is:
 * Last |This |Present |Absent  |Send It and
 * Check|Check|Msg Sent|Msg Sent|Do This 
 * -----+-----+--------+--------+-------------------------------
 * No   | No  | N/A    | False  | Yes, set "Absent Sent"=true, "Present Sent"=false
 * No   | No  | N/A    | True   | No
 * No   | Yes | N/A    | N/A    | No
 * No   | Yes | N/A    | N/A    | No
 * Yes  | No  | N/A    | N/A    | No
 * Yes  | No  | N/A    | N/A    | No
 * Yes  | Yes | False  | N/A    | Yes, set "Present Sent"=true, "Absent Sent"=false
 * Yes  | Yes | True   | N/A    | No
 */
void sendOrNot()
  {
  if (myMillis()>myRtc.nextHealthReportTime
      ||((!myRtc.wasPresent && !isPresent) && !myRtc.absentReported)
      ||((myRtc.wasPresent && isPresent) && !myRtc.presentReported))
    {      
    // ********************* attempt to connect to Wifi network
    connectToWiFi();
      
    // ********************* Initialize the MQTT connection
    reconnect();  // connect to the MQTT broker
     
    report();

    if (isPresent)
      {
      myRtc.presentReported=true;
      myRtc.absentReported=false;
      }
    else
      {
      myRtc.absentReported=true;
      myRtc.presentReported=false;
      }
    
    doneTimestamp=millis(); //this is to allow the publish to complete before sleeping
    myRtc.nextHealthReportTime=myMillis()+ONE_HOUR;
    myDelay(5000); //wait for any incoming messages
    }
  
  }

  
/*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

//    WiFi.forceSleepWake(); //turn on the radio
//    delay(1);              //return control to let it come on
    
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (ip.isSet()) //Go with a dynamic address if no valid IP has been entered
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }
    WiFi.begin(settings.ssid, settings.wifiPassword);
    while (WiFi.status() != WL_CONNECTED) 
      {
      // not yet connected
      Serial.print(".");
      checkForCommand(); // Check for input in case something needs to be changed to work
      delay(500);
      }
  
    Serial.print("Connected to network with address ");
    Serial.println(WiFi.localIP());
    Serial.println();

    myRtc.rssi=WiFi.RSSI(); //save the RSSI for later report
    }
  else if (settings.debug)
    {
    Serial.print("Actual network address is ");
    Serial.println(WiFi.localIP());
    }
  }

/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  payload[length]='\0'; //this should have been done in the calling code, shouldn't have to do it here
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  const char* response;
  
  
  //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
  if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
    {
    char tempbuf[35]; //for converting numbers to strings
    char jsonStatus[JSON_STATUS_SIZE];
    
    strcpy(jsonStatus,"{");
    strcat(jsonStatus,"\"broker\":\"");
    strcat(jsonStatus,settings.mqttBrokerAddress);
    strcat(jsonStatus,"\", \"port\":");
    sprintf(tempbuf,"%d",settings.mqttBrokerPort);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,", \"topicroot\":\"");
    strcat(jsonStatus,settings.mqttTopicRoot);
    strcat(jsonStatus,"\", \"user\":\"");
    strcat(jsonStatus,settings.mqttUsername);
    strcat(jsonStatus,"\", \"pass\":\"");
    strcat(jsonStatus,settings.mqttPassword);
    strcat(jsonStatus,"\", \"ssid\":\"");
    strcat(jsonStatus,settings.ssid);
    strcat(jsonStatus,"\", \"wifipass\":\"");
    strcat(jsonStatus,settings.wifiPassword);
    strcat(jsonStatus,"\", \"mindistance\":\"");
    sprintf(tempbuf,"%d",settings.mindistance);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"maxdistance\":\"");
    sprintf(tempbuf,"%d",settings.maxdistance);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"sleeptime\":\"");
    sprintf(tempbuf,"%d",settings.sleeptime);
    strcat(jsonStatus,tempbuf);
    strcat(jsonStatus,"\", \"mqttClientId\":\"");
    strcat(jsonStatus,settings.mqttClientId);
    strcat(jsonStatus,"\", \"address\":\"");
    strcat(jsonStatus,settings.address);
    strcat(jsonStatus,"\", \"netmask\":\"");
    strcat(jsonStatus,settings.netmask);
    strcat(jsonStatus,"\", \"debug\":\"");
    strcat(jsonStatus,settings.debug?"true":"false");
    strcat(jsonStatus,"\", \"displayenabled\":\"");
    strcat(jsonStatus,settings.displayenabled?"true":"false");
    
    strcat(jsonStatus,"\"}");
    response=jsonStatus;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
    {
    char tmp[15];
    strcpy(tmp,VERSION);
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest value
    {
    report();
    
    char tmp[25];
    strcpy(tmp,"Status report complete");
    response=tmp;
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
    {
    char tmp[10];
    strcpy(tmp,"REBOOTING");
    response=tmp;
    rebootScheduled=true;
    }
  else if (processCommand(charbuf))
    {
    response="OK";
    }
  else
    {
    char badCmd[18];
    strcpy(badCmd,"(empty)");
    response=badCmd;
    }
    
  char topic[MQTT_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response,false)) //do not retain
    Serial.println("************ Failure when publishing status response!");
    
  delay(2000); //give publish time to complete
  
  if (rebootScheduled)
    {
    ESP.restart();
    }
  }

/* Draw a dot at a point on the screen, and increment to the next position */
void makeDot(uint8_t *position)
  {
  display.fillCircle(*position,SCREEN_HEIGHT-DOT_RADIUS*2,DOT_RADIUS,WHITE);
  display.display();
  *position+=DOT_RADIUS*2+DOT_SPACING;
  }

/*
 * This returns the elapsed milliseconds, even if we've been sleeping
 */
unsigned long myMillis()
  {
  return millis()+myRtc.rtc;
  }

// Read the distance SAMPLE_COUNT times and return the dominant value
int measure()
  {
  int vals[SAMPLE_COUNT];
  int answer=0,answerCount=0;
  uint8_t dotPosition=DOT_RADIUS; //where to start drawing the sampling dots

  //get samples
  for (int i=0;i<SAMPLE_COUNT;i++)
    {
    makeDot(&dotPosition);
    vals[i]=getDistance();

    // Turn off the LED
    digitalWrite(LED_BUILTIN,LED_OFF);
    
    delay(50); //give it some space
    }

  //find the most common value within the sample set
  //This code is not very efficient but hey, it's only 10 values
  for (int i=0;i<SAMPLE_COUNT-1;i++) //using SAMPLE_COUNT-1 here because the last one can only have a count of 1
    {
    int candidate=vals[i];
    int candidateCount=1;  
    for (int j=i+1;j<SAMPLE_COUNT;j++)
      {
      if (candidate==vals[j])
        {
        candidateCount++;
        }
      }
    if (candidateCount>answerCount)
      {
      answer=candidate;
      answerCount=candidateCount;
      }
    }
  return answer;
  }



void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicroot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("mindistance=<minimum presence distance in cm> (");
  Serial.print(settings.mindistance);
  Serial.println(")");
  Serial.print("maxdistance=<maximum presence distance in cm> (");
  Serial.print(settings.maxdistance);
  Serial.println(")");
  Serial.print("sleeptime=<seconds to sleep between measurements> (");
  Serial.print(settings.sleeptime);
  Serial.println(")");
  Serial.print("address=<Static IP address if so desired> (");
  Serial.print(settings.address);
  Serial.println(")");
  Serial.print("netmask=<Network mask to be used with static IP> (");
  Serial.print(settings.netmask);
  Serial.println(")");
  Serial.print("debug=1|0 (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("displayEnabled=1|0 (");
  Serial.print(settings.displayenabled);
  Serial.println(")");
  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.print("Address is ");
  Serial.println(wifiClient.localIP());
  Serial.println("\n*** Use NULL to reset a setting to its default value ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"complete.":"incomplete.");
  }

/*
 * Reconnect to the MQTT broker
 */
void reconnect() 
  {
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
    {      
    Serial.print("Attempting MQTT connection...");

    mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
    mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
    mqttClient.setCallback(incomingMqttHandler);
    
    // Attempt to connect
    if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
      {
      Serial.println("connected to MQTT broker.");

      //resubscribe to the incoming message topic
      char topic[MQTT_TOPIC_SIZE];
      strcpy(topic,settings.mqttTopicRoot);
      strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
      bool subgood=mqttClient.subscribe(topic);
      showSub(topic,subgood);
      }
    else 
      {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Will try again in a second");
      
      // Wait a second before retrying
      // In the meantime check for input in case something needs to be changed to make it work
    //  checkForCommand(); 
      
      myDelay(1000);
      }
    }
  mqttClient.loop(); //This has to happen every so often or we get disconnected for some reason
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

  
/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  //Get rid of the carriage return
  if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
    val[strlen(val)-1]=0; 

  if (nme==NULL || val==NULL || strlen(nme)==0 || strlen(val)==0)
    {
    showSettings();
    return false;   //not a valid command, or it's missing
    }
  else if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
    {
    strcpy(val,"");
    }
  
  if (strcmp(nme,"broker")==0)
    {
    strcpy(settings.mqttBrokerAddress,val);
    saveSettings();
    }
  else if (strcmp(nme,"port")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.mqttBrokerPort=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"topicroot")==0)
    {
    strcpy(settings.mqttTopicRoot,val);
    saveSettings();
    }
  else if (strcmp(nme,"user")==0)
    {
    strcpy(settings.mqttUsername,val);
    saveSettings();
    }
  else if (strcmp(nme,"pass")==0)
    {
    strcpy(settings.mqttPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"ssid")==0)
    {
    strcpy(settings.ssid,val);
    saveSettings();
    }
  else if (strcmp(nme,"wifipass")==0)
    {
    strcpy(settings.wifiPassword,val);
    saveSettings();
    }
  else if (strcmp(nme,"address")==0)
    {
    strcpy(settings.address,val);
    saveSettings();
    }
  else if (strcmp(nme,"netmask")==0)
    {
    strcpy(settings.netmask,val);
    saveSettings();
    }
  else if (strcmp(nme,"mindistance")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.mindistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"maxdistance")==0)
    {
    if (!val)
      strcpy(val,"400");
    settings.maxdistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"sleeptime")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.sleeptime=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"debug")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.debug=atoi(val)==1?true:false;
    saveSettings();
    }
  else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
    {
    generateMqttClientId(settings.mqttClientId);
    saveSettings();
    }
  else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
    {
    Serial.println("\n*********************** Resetting EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  else if (strcmp(nme,"displayenabled")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.displayenabled=atoi(val)==1?true:false;
    saveSettings();
    }
  else
    {
    showSettings();
    return false; //command not found
    }
  return true;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  strcpy(settings.address,"");
  strcpy(settings.netmask,"255.255.255.0");
  settings.mindistance=0;
  settings.maxdistance=400;
  settings.sleeptime=10;
  settings.displayenabled=true;
  generateMqttClientId(settings.mqttClientId);
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }

int readBattery()
  {
  int raw=ESP.getVcc(); //This commandeers the ADC port
  if (settings.debug)
    {
    Serial.print("Raw voltage count:");
    Serial.println(raw);
    }
  return raw;
  }

float convertToVoltage(int raw)
  {
  int vcc=map(raw,0,FULL_BATTERY,0,340);
  float f=((float)vcc)/100.0;
  return f;
  }


/************************
 * Do the MQTT thing
 ************************/
void report()
  {  
  char topic[MQTT_TOPIC_SIZE];
  char reading[18];
  boolean success=false;
  int analog=readBattery();
  
  //publish the radio strength reading
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RSSI);
  sprintf(reading,"%d",(int)myRtc.rssi); 
  success=publish(topic,reading,true); //retain
  if (!success)
    Serial.println("************ Failed publishing rssi!");
  
  //publish the raw battery reading
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_ANALOG);
  sprintf(reading,"%d",analog); 
  success=publish(topic,reading,true); //retain
  if (!success)
    Serial.println("************ Failed publishing raw battery reading!");

  //publish the battery voltage
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_BATTERY);
  sprintf(reading,"%.2f",convertToVoltage(analog)); 
  success=publish(topic,reading,true); //retain
  if (!success)
    Serial.println("************ Failed publishing battery voltage!");

  //publish the distance measurement
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_DISTANCE);
  sprintf(reading,"%d",distance); 
  success=publish(topic,reading,true); //retain
  if (!success)
    Serial.println("************ Failed publishing distance measurement!");

  //publish the object detection state
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_STATE);
  sprintf(reading,"%s",isPresent?"YES":"NO"); //item within range window
  success=publish(topic,reading,true); //retain
  if (!success)
    Serial.println("************ Failed publishing sensor state!");
  }

boolean publish(char* topic, const char* reading, boolean retain)
  {
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(reading);
  return mqttClient.publish(topic,reading,retain); 
  }

  
/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("Loaded configuration values from EEPROM");
//      showSettings();
      }
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
    strlen(settings.wifiPassword)>0 &&
    strlen(settings.mqttBrokerAddress)>0 &&
    settings.mqttBrokerPort!=0 &&
    strlen(settings.mqttTopicRoot)>0 &&
    strlen(settings.mqttClientId)>0)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }
    
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

/*
 * Save the pan-sleep information to the RTC battery-backed RAM
 */
void saveRTC()
  {
  system_rtc_mem_write(64, &myRtc, sizeof(myRtc)); 
  }


//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,MQTT_CLIENT_ID_ROOT);
  strcat(mqttId, String(random(0xffff), HEX).c_str());
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }
  
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingData() 
  {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }
