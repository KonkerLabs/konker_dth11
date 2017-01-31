// Codigo do LED com ACK em comunicacao MQTT
// Responsavel: Luis Fernando Gomez Gonzalez (luis.gonzalez@inmetrics.com.br)
// Projeto Konker

// Agora usando o gerenciador de conexão WiFiManagerK.
#include <FS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ArduinoJson.h>
#include "konker.h"
//for LED id
#include "BlinkerID.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define period 1 //tempo do loop

// Uncomment the type of sensor in use:
#define DHTTYPE           DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);



long getTemperature(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    long temp = event.temperature;
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" *C");
    return temp;
  }
  return 0;
}

long getHumidity(){
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    long humi = event.relative_humidity;
    Serial.print("humidity: ");
    Serial.print(humi);
    Serial.println("%");
    return humi;
  }
  return 0;
}




//Variaveis da Configuracao em Json
char api_key[17];
char device_id[17];
char mqtt_server[64];
char mqtt_port[5];
char mqtt_login[32];
char mqtt_pass[32];
char *mensagemjson;
char mensagemC[100];

//Variaveis Fisicas
int name_ok=0;

//String de command
String pubString;
char message_buffer[20];
int reconnectcount=0;
char *type;
char typeC[32];
String Stype;
int disconnected=0;

int marked=0;


//Ticker
Ticker timer;

int timeout=10000;
int tic=0;
int newComand=0;

WiFiManager wifiManager;



//Criando as variaveis dentro do WiFiManager
WiFiManagerParameter custom_api_key("api", "api key", api_key, 17);
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 64);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
WiFiManagerParameter custom_mqtt_login("login", "mqtt login", mqtt_login, 32);
WiFiManagerParameter custom_mqtt_pass("password", "mqtt pass", mqtt_pass, 32);



// Funcao de Callback para o MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  int i;
  int state=0;

  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  for (i = 0; i < length; i++) {
    msgBufferIN[i] = payload[i];
    Serial.print((char)payload[i]);
  }
  msgBufferIN[i] = '\0';
  strcpy(msgTopic, topic);
  received_msg = 1;
  Serial.println("");
}

//Definindo os objetos de Wifi e MQTT
WiFiClient espClient;
PubSubClient client("mqtt.demos.konkerlabs.net", 1883, callback,espClient);


// Setup do Microcontrolador: Vamos configurar o acesso serial, conectar no Wifi, configurar o MQTT e o GPIO do botao
void setup(){
  Serial.println("Setup");
  Serial.begin(115200);

  //------------------- Montando Sistema de arquivos e copiando as configuracoes  ----------------------
  spiffsMount(mqtt_server, mqtt_port, mqtt_login, mqtt_pass, device_id, device_type, api_key, in_topic, cmd_topic, data_topic, ack_topic, config_topic, connect_topic, config_period);

  //Criando as variaveis dentro do WiFiManager
  WiFiManagerParameter custom_api_key("api", "api key", api_key, 17);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 64);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_login("login", "mqtt login", mqtt_login, 32);
  WiFiManagerParameter custom_mqtt_pass("password", "mqtt pass", mqtt_pass, 32);


  //wifiManager.resetSettings();
  //------------------- Configuracao do WifiManager K ----------------------
  wifiManager.setTimeout(500);
  wifiManager.setBreakAfterConfig(1);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_api_key);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_login);
  wifiManager.addParameter(&custom_mqtt_pass);





  //------------------- Caso conecte, copie os dados para o FS ----------------------
  if(!wifiManager.autoConnect(blinkMyID("Konker"))) {

    //Copiando parametros
    copyHTMLPar(api_key, mqtt_server, mqtt_port, mqtt_login, mqtt_pass, custom_api_key, custom_mqtt_server, custom_mqtt_port, custom_mqtt_login, custom_mqtt_pass);

    //Salvando Configuracao
    if (shouldSaveConfig) {
      saveConfigtoFile(api_key, device_id, mqtt_server, mqtt_port, mqtt_login, mqtt_pass);
    }
    delay(2500);
    ESP.reset();
  }

  //------------------- Caso tudo mais falhe copie os dados para o FS ----------------------
  //Copiando parametros
  copyHTMLPar(api_key, mqtt_server, mqtt_port, mqtt_login, mqtt_pass, custom_api_key, custom_mqtt_server, custom_mqtt_port, custom_mqtt_login, custom_mqtt_pass);

  //Salvando Configuracao
  if (shouldSaveConfig) {
    saveConfigtoFile(api_key,device_id, mqtt_server,mqtt_port,mqtt_login,mqtt_pass);
  }

  //------------------- Configurando MQTT e LED ----------------------
  /*if (espClient2.connect("http://svc.eg.konkerlabs.net", 5595))
  {
    delay(50);
    Serial.println("Conectado com o endereco: http://svc.eg.konkerlabs.net!!");
    espClient2.stop();
  }*/
  client.setServer(mqtt_server, atol(mqtt_port));
  client.setCallback(callback);
  delay(200);
  if (!client.connected()) {
    disconnected=1;
    reconnectcount = reconnect(client, api_key, mqtt_login, mqtt_pass);
    if (reconnectcount>10) resetsettings=1;
  }
  client.subscribe(in_topic);
  client.subscribe(config_topic);

  config_period_I=period;

  //-----------------------------------------------------------------------------------------

  timer.attach_ms(1000*config_period_I,sendMessage);



  Serial.println();
  Serial.println("Setup finalizado");
  Serial.println(config_period_I);

  startBlinkID(0);// pass zero to stop
  //keep LED on
  digitalWrite(_STATUS_LED, LOW);

}




// Loop com o programa principal
void loop(){
  delay(100);

  // Se desconectado, reconectar.
  if (!client.connected()) {
    disconnected = 1;
    reconnectcount = reconnect(client, api_key, mqtt_login, mqtt_pass);
    if (reconnectcount>10) resetsettings=1;
    else {
      client.subscribe(in_topic);
      client.subscribe(config_topic);
    }
  }

  client.loop();
  delay(10);


  if (resetsettings==1)
  {
    resetsettings=0;
    wifiManager.resetSettings();
    delay(500);
    ESP.restart();
  }

  if (disconnected==1)
  {
    long temperature = getTemperature();
    long humidity = getHumidity();
    mensagemjson = jsonMQTTmsgCONNECT("1454853486000", device_id, "Connected!", "Temperature",temperature,"Celsius", "Humidity",humidity,"%");
    //if (messageACK(client, connect_topic, mensagemjson, ack_topic, name_ok)) name_ok=0;
    if (messageACK(client, data_topic, mensagemjson, ack_topic, name_ok)) name_ok=0;
    disconnected = 0;
  }

  if (senddata==1)
  {
    long temperature = getTemperature();
    long humidity = getHumidity();
    mensagemjson = jsonMQTTmsgDATA("1454853486000", device_id,"Temperature",temperature,"Celsius", "Humidity",humidity,"%");
    if (messageACK(client, data_topic, mensagemjson, ack_topic, name_ok)) name_ok=0;
    senddata=0;
  }


  if (configured==1)
  {
    timer.detach();
    delay(200);
    timer.attach_ms(1000*config_period_I,sendMessage);
    configured=0;
  }


  if (received_msg==1) {
    if (String(msgTopic)==String(config_topic))
    {
      jsonMQTT_config_msg(msgBufferIN);
    }

    if (String(msgTopic)==String(in_topic))
    {
      type = jsonMQTT_in_msg(msgBufferIN);
      strcpy(typeC, type);
      //if (String(typeC)==String(IR_type)) sendIR(irsend, msgBufferIN);
      delay(200);
    }
    received_msg=0;
  }

}
