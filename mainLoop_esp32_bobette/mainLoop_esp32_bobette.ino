#include <WiFi.h>
#include <PubSubClient.h>

//https://pubsubclient.knolleary.net/api
//https://randomnerdtutorials.com/solved-reconnect-esp32-to-wifi/


//esp32 ip address
//192.168.178.52


const char* ssid = "FRITZ!Box 7590 XQ_EXT";
const char* password = "12143927197014187848";


//const char* mqttServer = "broker.hivemq.com";
//const char* broker = "raspberrypi";
IPAddress broker(192,168,178,82);
//IPAddress broker(169,254,60,122);
const int mqttPort = 1883;

#define MQTT_USER "testuser1"
#define MQTT_PASSWORD "test1"
// set channels for bot to listen to
#define MQTT_SERIAL_RECEIVER_CH_ONE "botInstruction/bobette"
#define MQTT_SERIAL_RECEIVER_CH_TWO "botRotation/bobette"
#define MQTT_SERIAL_RECEIVER_CH_THREE "botDistance/bobette"
#define MQTT_SERIAL_RECEIVER_DEBUG "debug/bobette"
// set channels for bot to publish to
#define MQTT_SERIAL_PUBLISH_CH_ONE "status/bobette"
#define MQTT_SERIAL_PUBLISH_CH_TWO "bobbinsPerBot"
#define MQTT_SERIAL_PUBLISH_CH_THREE "distanceToBobbin"
#define MQTT_SERIAL_PUBLISH_CH_FOUR "connectionStatus"

#define MQTT_SERIAL_PUBLISH_CH_FIVE "gripRightDistance"
#define MQTT_SERIAL_PUBLISH_CH_SIX "gripLeftDistance"

const char* will_message = "ESP_Bobette/disconnected";
unsigned long previousMillis = 0;
unsigned long interval = 40000;

unsigned long restartInterval = 10000;
unsigned long previousRestartMillis = 0;

//// Set your Static IP address
//IPAddress local_IP(192, 168, 1, 184);
//// Set your Gateway IP address
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 0, 0);
//IPAddress primaryDNS(8, 8, 8, 8); //optional
//IPAddress secondaryDNS(8, 8, 4, 4); //optional

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {

//  if(!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//    Serial.println("STA Failed to configure");
//}
  Serial.begin(115200);


  Serial.setTimeout(500);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  setup_wifi();
  

  // change "broker" to "mqttServer" if using HiveMQ
  client.setServer(broker, mqttPort);
  client.setCallback(callback);
  client.setKeepAlive(60); //default is 15secs. time to keep connection open when no data is being sent
  reconnect();
  

}

void loop() {

  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    //client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, "esp32 wifi down... =(");
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    //WiFi.reconnect();
    //WiFi.begin(ssid, password);
    setup_wifi();
    previousMillis = currentMillis;
  }
  
  client.loop();
  

  if(Serial.available()>0){
    char mun[501];          //char buffer to store the bytes
    memset(mun,0,501);
    Serial.readBytesUntil('\n',mun,500);
    
    // here we check the contents of the message from the arduino (sent to the esp via serial)
    // we check for keywords, and depending on those, publish the data to the respective channel
    String content;
    content = String(mun);
    if (content.indexOf("status") >= 0){
      // publish to status channel
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_ONE, mun);
    }
    if (content.indexOf("bobbins") >= 0){
      // publish to bobbinsPerBot channel
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_TWO, mun);
    }
    if (content.indexOf("distance") >= 0){
      // publish to distanceToBobbin channel
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_THREE, mun);
    }
    if (content.indexOf("right") >= 0){
      // publish to gripRightDistance channel
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_FIVE, mun);
    }
    if (content.indexOf("left") >= 0){
      // publish to gripLeftDistance channel
      publishSerialData(MQTT_SERIAL_PUBLISH_CH_SIX, mun);
    }
  }
}

void setup_wifi(){

  
  
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    unsigned long currentRestartMillis = millis();
    delay(500);
    Serial.print(".");
    // if it takes more than 10seconds to connect on boot, then restart
    if (currentRestartMillis - previousRestartMillis >= restartInterval){
      ESP.restart();
    }
  }
  randomSeed(micros());
  //Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, "esp32 reconnected");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Create a random client ID
    String clientId = "ESP_Bobette_";
    clientId += String(random(0xffff), HEX);
    String connectionId = clientId + "/connected";
    //String disconnectionId = clientId + "/disconnected";

    client.connect(clientId.c_str(), NULL, NULL, MQTT_SERIAL_PUBLISH_CH_FOUR, 0, true, will_message);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish(MQTT_SERIAL_PUBLISH_CH_ONE, "hello status");
      //client.publish(MQTT_SERIAL_PUBLISH_CH_TWO, "hello bobbinsPerBot");
      //client.publish(MQTT_SERIAL_PUBLISH_CH_THREE, "hello distanceToBobbin");
      client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, connectionId.c_str());
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH_ONE);
      client.subscribe(MQTT_SERIAL_RECEIVER_CH_TWO);
      client.subscribe(MQTT_SERIAL_RECEIVER_CH_THREE);
      client.subscribe(MQTT_SERIAL_RECEIVER_DEBUG);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      //client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, disconnectionId.c_str());
      Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
    }
    /*if (client.state() == -2){
      client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, "esp32/network connection failed");
    }*/
  }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
  client.publish(MQTT_SERIAL_PUBLISH_CH_FOUR, String(info.disconnected.reason).c_str());
}



// the callback is basically what to do when a message is received
void callback(char* topic, byte *payload, unsigned int length) {

    if (strcmp(topic,"botInstruction/bobette")==0){
      // == 0 is result of strcmp() if it is a match
      //code for botInstruction
      String response = "";
      //char response = 0;
      for (int i = 0; i < length; i++){
      response += (char)payload[i];
      }
      //Serial.println("The char response (in DEC) is: ");
      //Serial.print(response, DEC);
      //Serial.println("The char response is: ");
      Serial.print(response); // sends the recieved command to the arduino via serial as a byte
                              // the byte is the sum of the char's received. 
      Serial.println();
    }

   if (strcmp(topic,"botRotation/bobette")==0){
      String content = "rot/";
      //char response = 0;
      for (int i = 0; i < length; i++){
      //response += (char)payload[i];
      content += (char)payload[i];
      }
      Serial.print(content);
      Serial.println();
    
   }

    if (strcmp(topic,"botDistance/bobette")==0){
      String content = "dist/";
      //char response = 0;
      for (int i = 0; i < length; i++){
      //response += (char)payload[i];
      content += (char)payload[i];
      }
      Serial.print(content);
      Serial.println();
    
   }

    //debug channel for xi
    if (strcmp(topic,"debug/bobette")==0){
      String content = "bug/";
      for (int i = 0; i < length; i++){
      content += (char)payload[i];
      }
      Serial.print(content);
      Serial.println();
    
   }
    
}


void publishSerialData(char *channel, char *serialData){
  if (!client.connected()) {
    reconnect();
  }
  client.publish(channel, serialData);
}
