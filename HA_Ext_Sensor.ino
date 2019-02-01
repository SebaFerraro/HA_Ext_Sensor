
//#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <BH1750FVI.h>
//#include "Ticker.h"
#include "DHTesp.h"

#define DHT_DATA 23
#define I2C_SCL 22
#define I2C_SDA 21
#define DHTTYPE DHT11
#define RLY_P1 26
#define RLY_P2 27
#define ANALOG_PIN_0 35
#define MQTT_SERVER "10.77.17.89"  //you MQTT IP Address
#define MQTT_PORT 1883  //you MQTT IP Address
#define MQTT_USER "ha"  //you MQTT IP Address
#define MQTT_PASS "4ut0m4t1c0"  //you MQTT IP Address
#define INTERVALO 60000

const char* ssid = "wifi";
const char* password = "pass";
static int taskCore = 0;
int analog_value = 0;
static int Wconectado = 0;
float Sensibilidad=0.063; //sensibilidad en V/A para nuestro sensor
float offset=0.19; // Equivale a la amplitud del ruido
float Ioffset=0.07; // Equivale a la amplitud del ruido

//float Sensibilidad=0.066; //sensibilidad en V/A para nuestro sensor
//float offset=0.100; // Equivale a la amplitud del ruido
String TopicBase = "casa";
String TopicDev = "dev-o-01";
String Tswitch1 = "luz/o/sw1";
String Tswitch2 = "luz/o/sw2";
String TTemp = "temperatura";
String THum = "humedad";
String TSTerm = "termica";
String TLuz = "luz";
String TAmp = "corriente";

//DHT dht(DHT_DATA, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
DHTesp dht;
TaskHandle_t tempTaskHandle = NULL;
BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes2);

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}

void on_message(char* topic, byte* payload, unsigned int length) {
  Serial.println("Mensaje Recibido");
  String topicStr = String(topic);
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println((char)payload[0]);
  Serial.print("length: ");
  Serial.println(length);
  char cpayload[length + 1];
  strncpy (cpayload, (char*)payload, length);
  cpayload[length] = '\0';
  String msg=cpayload;
  String Topic1=TopicBase + "/" + Tswitch1;
  String Topic1C=TopicBase + "/" + Tswitch1 + "c";
  
  if (topicStr == Topic1){
    if((cpayload[0] == '1') || (cpayload[0] == '0')){
       Serial.print("Topic1 Payload : ");
       Serial.println(msg.toInt());
       digitalWrite(RLY_P1, msg.toInt());
       pubicatopic_mqtt(Topic1C, msg);
    }
  }else{
    Serial.println("Topic1 No coincide : ");
    Serial.println(topicStr);
    Serial.println(Topic1);
  }

  String Topic2=TopicBase + "/" + Tswitch2;
  String Topic2C=TopicBase + "/" + Tswitch2 + "c";
  if (topicStr == Topic2){
    if((payload[0] == '1') || (payload[0] == '0')){
       Serial.print("Topic2 Payload : ");
       Serial.println(msg.toInt());
       digitalWrite(RLY_P2, msg.toInt());
       pubicatopic_mqtt(Topic2C, msg);
    }
  }else{
    Serial.println("Topic2 No coincide : ");
    Serial.println(topicStr);
    Serial.println(Topic2);
  }
  
}

int suscribetopics_mqtt(){
  int rsus,rsus1;
  String Topic1=TopicBase + "/" + Tswitch1;
  rsus=client.subscribe(string2char(Topic1));
  Serial.print( "[SUBSCRIBE] Topic1" );
  Serial.println(Topic1);
  Serial.println(rsus);
  Topic1=TopicBase + "/" + Tswitch2;
  rsus1=client.subscribe(string2char(Topic1));
  Serial.print( "[SUBSCRIBE] Topic2" );
  Serial.println(Topic1);
  Serial.println(rsus1);
  return (rsus && rsus1);
}

char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}

int pubicatopic_mqtt(String topic, String msg){
  int rsus;
  rsus=client.publish(string2char(topic), msg.c_str());
  Serial.print( "Publish : ");
  Serial.println(rsus);
  Serial.print( "Topic : ");
  Serial.println(topic);
  Serial.print( "Valor :");
  Serial.println(msg);
  return rsus;
}

void send_mqtt(float t, float h, float hic, uint16_t l, float v){
  int rsus;
  
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando ThingsBoard node ...");
      if ( client.connect((char*) TopicDev.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println( "[DONE]" );
        suscribetopics_mqtt();
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
      }
    }
    if(client.connected()){
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String luz = String(l);
      String volt = String(v);
      
      String Topic=TopicBase + "/" + TopicDev + "/" + TTemp ;
      rsus=pubicatopic_mqtt(Topic, temperatura);
      Serial.print( "Publish Temp: ");
      Serial.println(rsus);
      Serial.print( "Temp: ");
      Serial.println(temperatura);
  
      Topic=TopicBase + "/" + TopicDev + "/" + THum ;
      rsus=pubicatopic_mqtt(Topic,humedad);
      Serial.print( "Publish Hum:");
      Serial.println(rsus);
      Serial.print( "Hum: ");
      Serial.println(humedad);
      

      Topic=TopicBase + "/" + TopicDev + "/" + TSTerm ;
      rsus=pubicatopic_mqtt(Topic,stermica);
      Serial.print( "Publish Sterm: ");
      Serial.println(rsus);
      Serial.print( "STerm: ");
      Serial.println(stermica);
      

      Topic=TopicBase + "/" + TopicDev + "/" + TLuz ;
      rsus=pubicatopic_mqtt(Topic,luz);
      Serial.print( "Publish Luz: ");
      Serial.println(rsus);
      Serial.print( "Luz: ");
      Serial.println(luz);

      Topic=TopicBase + "/" + TopicDev + "/" + TAmp ;
      rsus=pubicatopic_mqtt(Topic,volt);
      Serial.print( "Publish volt: ");
      Serial.println(rsus);
      Serial.print( "Volt: ");
      Serial.println(volt);
    }
  }
}

float get_corriente()
{
  float voltajeSensor;
  float corriente=0;
  float pcorriente=0;
  float tcorriente=0;
  float vsensor=0;
  long tiempo=millis();
  float Imax=0;
  float Imin=0;
  int n=0;
  while(millis()-tiempo<500)//realizamos mediciones durante 0.5 segundos
  { 
    vsensor=analogRead(ANALOG_PIN_0);
    voltajeSensor = vsensor * (3.3 / 4095.0) + offset;//lectura del sensor
    //corriente=0.9*corriente+0.1*((voltajeSensor-2.53)/Sensibilidad); //Ecuación  para obtener la corriente
    corriente=((voltajeSensor - 2.53) / Sensibilidad); //Ecuación  para obtener la corriente
    pcorriente=pcorriente + sq(corriente);
    //Serial.print("Vsensor :");
    //Serial.print(vsensor);
    //Serial.print("  voltajeSensor :");
    //Serial.print(voltajeSensor);
    //Serial.print("  pcorriente :");
    //Serial.print(pcorriente);
    //Serial.print("  corriente :");
    //Serial.println(corriente);
    
    if(corriente>Imax)Imax=corriente;
    if(corriente<Imin)Imin=corriente;
    n++;
  }
  tcorriente=(sqrt((pcorriente/n)) - Ioffset);
  Serial.print("Nro de Muestras :");
  Serial.print(n);
  Serial.print("   Imax:");
  Serial.print(Imax);
  Serial.print("   Imin :");
  Serial.print(Imin);
  Serial.print("  P Corriente :");
  Serial.print(sqrt(pcorriente/n));
  Serial.print("  Corriente - offset:");
  Serial.println(tcorriente);
  
  //return(((Imax-Imin)/2)-Ioffset);
  return(tcorriente);
}


void coreTask( void * pvParameters ){
 
    String taskMessage = "Corriendo en core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    int rsus;
    
    while(true){
      if(Wconectado == 1){
          if(!client.connected()) {
            Serial.print("Conectando ThingsBoard node ...");
            if ( client.connect((char*) TopicDev.c_str(), MQTT_USER, MQTT_PASS)) {
              Serial.println( "[DONE]" );
              suscribetopics_mqtt();
            } else {
              Serial.print( "[FAILED] [ rc = " );
              Serial.print( client.state() );
            }
          }
          //Serial.print("LOOP ");
          if(! client.loop()){
            Serial.println("Error en Loop.");  
          }
          //Serial.println(taskMessage);
      }else{
      Serial.print("No conectado wifi:");
      Serial.println(Wconectado);
    }
   delay(300);
   ArduinoOTA.handle();
   }
}

void setup() { 
  Serial.begin(115200);
  pinMode(RLY_P1, OUTPUT);
  pinMode(RLY_P2, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  pinMode(I2C_SDA,INPUT_PULLUP);
  pinMode(I2C_SCL,INPUT_PULLUP);
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  ArduinoOTA.setHostname((const char*) TopicDev.c_str()); // A name given to your ESP8266 module when discovering it as a port in ARDUINO IDE
  ArduinoOTA.begin(); // OTA initialization
  delay(400);
  dht.setup(DHT_DATA, DHTesp::DHT11);
  LightSensor.begin();

  //dht.begin();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(on_message);
  xTaskCreatePinnedToCore(coreTask, "coreTask", 10000, NULL, 0, NULL, taskCore);
  Serial.println("Hilo Creado...");  
}


void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }
    
  //float h = dht.readHumidity();
  //float t = dht.readTemperature();

  TempAndHumidity lastValues = dht.getTempAndHumidity(); 
  //float h = dht.readHumidity();
  //float t = dht.readTemperature();
  float t=lastValues.temperature;
  float h=lastValues.humidity;
  Serial.println("Temperature: " + String(lastValues.temperature,0));
  Serial.println("Humidity: " + String(lastValues.humidity,0));
  
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(800);
    return;
  }
  uint16_t lux = LightSensor.GetLightIntensity();
  delay(100);
  float hic = dht.computeHeatIndex(t, h, false);
  //int analog_value=analogRead(ANALOG_PIN_0);
  float Ip=get_corriente();//obtenemos la corriente pico
  float Irms=Ip*0.707; //Intensidad RMS = Ipico/(2^1/2)
  float P=Ip*220.0; // P=IV watts
  Serial.print("Ip: ");
  Serial.print(Ip,3);
  Serial.print("A , Irms: ");
  Serial.print(Irms,3);
  Serial.print("A, Potencia: ");
  Serial.print(P,3);  
  Serial.println("W");

  delay(INTERVALO);
  Serial.print("Analogico:");
  Serial.println(analog_value);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  Serial.print(" Lux :");
  Serial.println(lux);
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  send_mqtt(t,h,hic,lux,Irms);
 }
 
