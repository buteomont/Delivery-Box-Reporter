// Pins
#define MAX_DIST 8100 // Anything over 8100 mm is "out of range"

#define LED_ON LOW
#define LED_OFF HIGH
#define PORT_XSHUT D6 //needs a pull-down resistor on the esp8266
#define PORT_DISPLAY D7
#define SDA_PIN D2
#define SCL_PIN D1

#define MAX_HARDWARE_FAILURES 20
#define VALID_SETTINGS_FLAG 0xDAB0
#define SSID_SIZE 100
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50
#define MQTT_CLIENTID_SIZE 25
#define MQTT_TOPIC_SIZE 150
#define MQTT_TOPIC_DISTANCE "distance"
#define MQTT_TOPIC_STATE "present"
#define MQTT_TOPIC_BATTERY "battery"
#define MQTT_TOPIC_ANALOG "analog"
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_CLIENT_ID_ROOT "DeliveryReporter"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define MQTT_PAYLOAD_SETTINGS_COMMAND "settings" //show all user accessable settings
#define MQTT_PAYLOAD_RESET_PULSE_COMMAND "resetPulseCounter" //reset the pulse counter to zero
#define MQTT_PAYLOAD_REBOOT_COMMAND "reboot" //reboot the controller
#define MQTT_PAYLOAD_VERSION_COMMAND "version" //show the version number
#define MQTT_PAYLOAD_STATUS_COMMAND "status" //show the most recent flow values
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+50 //+50 for associated field names, etc
#define PUBLISH_DELAY 400 //milliseconds to wait after publishing to MQTT to allow transaction to finish
//#define MAX_CHANGE_PCT 2 //percent distance change must be greater than this before reporting
#define FULL_BATTERY_COUNT 3686 //raw A0 count with a freshly charged 18650 lithium battery 
#define FULL_BATTERY_VOLTS 412 //4.12 volts for a fully charged 18650 lithium battery 
#define ONE_HOUR 3600000 //milliseconds
#define SAMPLE_COUNT 5 //number of samples to take per measurement 

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 32      // OLED display height, in pixels
#define OLED_RESET    -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define DOT_RADIUS    2       // radius of activity dot
#define DOT_SPACING   4       // spacing between dots

// Error codes copied from the MQTT library
// #define MQTT_CONNECTION_REFUSED            -2
// #define MQTT_CONNECTION_TIMEOUT            -1
// #define MQTT_SUCCESS                        0
// #define MQTT_UNACCEPTABLE_PROTOCOL_VERSION  1
// #define MQTT_IDENTIFIER_REJECTED            2
// #define MQTT_SERVER_UNAVAILABLE             3
// #define MQTT_BAD_USER_NAME_OR_PASSWORD      4
// #define MQTT_NOT_AUTHORIZED                 5

//WiFi status codes
//0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
//1 : WL_NO_SSID_AVAILin case configured SSID cannot be reached
//3 : WL_CONNECTED after successful connection is established
//4 : WL_CONNECT_FAILED if password is incorrect
//6 : WL_DISCONNECTED if module is not configured in station mode

//prototypes
//PubSubClient callback function prototype.  This must appear before the PubSubClient constructor.
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);

void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);;
unsigned long myMillis();
bool processCommand(String cmd);
void checkForCommand();
void connectToWiFi();
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length); 
int measure();
int getDistance();
void showSettings();
void reconnect(); 
void showSub(char* topic, bool subgood);
void initializeSettings();
int readBattery();
void report();
boolean publish(char* topic, const char* reading, bool retain);
void loadSettings();
boolean saveSettings();
void saveRTC();
void serialEvent(); 
void sendOrNot();
char* generateMqttClientId(char* mqttId);
float convertToVoltage(int raw);
void setup(); 
void loop();
void incomingData();