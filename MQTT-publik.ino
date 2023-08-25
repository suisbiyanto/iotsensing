/**** Koneksi ke hivemq.cloud *******/
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include "DHTesp.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/**** DHT11 sensor Settings *******/
#define DHTpin 13   //Set DHT pin as GPIO2
DHTesp dht;

/**** LED Settings *******/
const int led = 5; //Set LED pin as GPIO5

/****** WiFi Connection Details *******/
const char* ssid = "xxxxxxxxxxxxx";
const char* password = "xxxxxxxxxxxxxxxx";

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = "xxxxxxxxxxxxxxxxx.s2.eu.hivemq.cloud";
const char* mqtt_username = "xxxxxxxxxxxxxserver";
const char* mqtt_password = "xxxxxxxxxxxxxxxx";
const int mqtt_port =8883;

/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/****** root certificate *********/

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
bmQuY29uc29sZS5wb3J0YWwuZXVjMS5hd3MuaGl2ZW1xLmNsb3VkMBMGA1UdIAQM
MAowCAYGZ4EMAQIBMIIBBAYKKwYBBAHWeQIEAgSB9QSB8gDwAHYAejKMVNi3LbYg
6jjgUh7phBZwMhOFTTvSK8E6V6NS61IAAAGJlgTsLwAABAMARzBFAiEAsZoBAtKw
fY/yvfjD4OUsmpOMKc4X28DQXHs4Au2bgSECICcILgxOMvrf4VxCqYwB4y0lh98E
JjE0SwOIGwRWZUPxAHYAtz77JN+cTbp18jnFulj0bF38Qs96nzXEnh0JgSXttJkA
AAGJlgTsLgAABAMARzBFAiEAoCCmEW56BUwbArQuGXt/MNm8g7G0r/JoSigmpggA
gTkCIDHHYj6MQJPPugn8SIDrhDgxWaBtAX2Q400z33r9SCKeMA0GCSqGSIb3DQEB
CwUAA4IBAQAtVpF+9Dj6cO/4qc4nJuHIFQX4Rt22Sv8TUVb18AIq8hkfnGjjVN00
KBxVNNgDML/ZiM9bv4AZCEk4kegyXUnZ0i5ifu9+sP69B3qOh5KqdYoWVaV+MOe0
fxrE5GzZJG6QWHc6AggXpTTt4kv2DSMwlT8kP3E+UhP8id0BbNq9XlBjjFiFNFJ/
yTNbbXjHJ3BexAplYnYoWqviW1HLjQ6DZqyMr6LV3K8LhBo0ysNQWdtad+ikUr/s
JDFA44fdNS9WWiF7PoWWnGOrsU0EqXCduAFyBRRNHPNXf7/CEyzMAKqZvbMRVIZC
fmO+S2YZTESF7oXc/eNi6DLIPwsRwmDA
-----END CERTIFICATE-----
)EOF";


/************* Connect to WiFi ***********/
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}

/************* Connect to MQTT Broker ***********/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected :");
      Serial.println(clientId);
      

      client.subscribe("led_state");   // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/***** Call back Method for Receiving MQTT messages and Switching LED ****/

void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];

  Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);

  //--- check the incomming message
    if( strcmp(topic,"led_state") == 0){
     if (incommingMessage.equals("1")) digitalWrite(led, HIGH);   // Turn the LED on
     else digitalWrite(led, LOW);  // Turn the LED off
  }
}

/**** Method for Publishing MQTT Messages **********/
void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}

/**** Application Initialisation Function******/
/**** Application Initialisation Function******/
void setup() {

  dht.setup(DHTpin, DHTesp::DHT11); //Set up DHT11 sensor
  pinMode(led, OUTPUT); //set up LED
  Serial.begin(9600);
  while (!Serial) delay(1);
  setup_wifi();

  #ifdef ESP8266
    espClient.setInsecure();
  #else
    espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  #endif

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

/******** Main Function *************/
void loop() {

  if (!client.connected()) reconnect(); // check if client is connected
  client.loop();

//read DHT11 temperature and humidity reading
  delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  
  // ArduinoJson 5
  DynamicJsonBuffer jb(1024);
  // ArduinoJson 6
  //DynamicJsonDocument doc(1024);


  //JsonObject& obj = jb.createObject();
  JsonObject& obj = jb.createObject();
  obj["deviceId"] = "NodeMCU";
  obj["siteId"] = "MyDemoLab";
  obj["humidity"] = humidity;
  obj["temperature"] = temperature;

  char mqtt_message[256];
  
  // ArduinoJson 5
  obj.printTo(mqtt_message);
  // ArduinoJson 6
  //serializeJson(doc, mqtt_message);

  publishMessage("esp8266_data", mqtt_message, true);

  delay(5000);

}
