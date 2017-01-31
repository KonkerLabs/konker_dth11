// Essa eh uma primeira versao do arquivo contendo as funcoes utilizadas para a comunicacao de dispositivos via MQTT na plataforma
// Konker. O proximo passo serah a tranformacao desse arquivo de funcoes para classes em C++.
// A adocao original dessa estrutura simples em C se deu pela velocidade de prototipacao e facil transformacao para uma estrutura
// de classes em um momento de maior especificidade do projeto.
//
// Responsavel: Luis Fernando Gomez Gonzalez (luis@konkerlabs.com)



// Vamos precisar de uma variavel Global para todas as funcoes, um enable para rodar todas as maquinas de estado.
int enable=1;

// Vamos precisar de uma variavel Global para todas voltar os settings para o valor inicial.
int resetsettings=0;

// Vamos precisar de uma variavel Global para avisar que uma configuracao foi feita.
int configured=0;

int senddata=0;

// Trigger para mensagens recebidas
int received_msg=0;

//Trigger da configuracao via Json -- Tambem precisa ser uma variavel global para rodar em duas maquinas de estado.
bool shouldSaveConfig = false;

//Buffer das mensagens MQTT
char bufferJ[256];

//Buffer de entrada MQTT;
char msgBufferIN[2048];
char msgTopic[32];

char device_type[5];
char in_topic[32];
char cmd_topic[32];
char data_topic[32];
char ack_topic[32];
char config_topic[32];
char connect_topic[32];
char config_period[5];
int config_period_I;

const char sub_dev_modifier[4] = "sub";
const char pub_dev_modifier[4] = "pub";
const char cmd_channel[8] = "command";
const char data_channel[5] = "data";
const char ack_channel[4] = "ack";
const char in_channel[3] = "in";
const char config_channel[7] = "config";
const char connect_channel[8] = "connect";

//Opcoes de commandos
const char *IR_type = "IR";
const char *Button_type1 = "ButtonPressed";
const char *Button_type2 = "ButtonPressedLong";
const char *Hue_type = "Hue";
const char *LED_ON = "LED ON";
const char *LED_OFF = "LED OFF";
const char *LED_SWITCH = "LED SWITCH";

const char *firmware_version = "FW v2.0";

//IR Variables

int long irdata[512];
int commaIndex[512];

//LED
int LEDState=0;

Ticker t;

//----------------- Funcao de Trigger para salvar configuracao no FS ---------------------------
void saveConfigCallback() {
  Serial.println("Salvar a Configuracao");
  shouldSaveConfig = true;
}

//----------------- Copiando parametros configurados via HTML ---------------------------
void copyHTMLPar(char api_key[], char mqtt_server[], char mqtt_port[], char mqtt_login[], char mqtt_pass[], WiFiManagerParameter custom_api_key, WiFiManagerParameter custom_mqtt_server, WiFiManagerParameter custom_mqtt_port, WiFiManagerParameter custom_mqtt_login, WiFiManagerParameter custom_mqtt_pass){

  strcpy(api_key, custom_api_key.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_login, custom_mqtt_login.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());

}

//----------------- Montando o sistema de arquivo e lendo o arquivo config.json ---------------------------
void spiffsMount(char mqtt_server[], char mqtt_port[], char mqtt_login[], char mqtt_pass[], char device_id[], char device_type[], char api_key[], char in_topic[], char cmd_topic[], char data_topic[], char ack_topic[], char config_topic[], char connect_topic[], char config_period[]) {
  String config_periodS;

  if (SPIFFS.begin()) {
    Serial.println("Sistema de Arquivos Montado");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Arquivo jรก existente, lendo..");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Arquivo aberto com sucesso");
        size_t size = configFile.size();
        // Criando um Buffer para alocar os dados do arquivo.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          if (json.containsKey("mqtt_server")) strcpy(mqtt_server, json["mqtt_server"]);
          if (json.containsKey("mqtt_port")) strcpy(mqtt_port, json["mqtt_port"]);
          if (json.containsKey("mqtt_login")) strcpy(mqtt_login, json["mqtt_login"]);
          if (json.containsKey("mqtt_pass")) strcpy(mqtt_pass, json["mqtt_pass"]);
          if (json.containsKey("device_id")) strcpy(device_id, json["device_id"]);
          if (json.containsKey("device_type")) strcpy(device_type, json["device_type"]);
          if (json.containsKey("api_key")) strcpy(api_key, json["api_key"]);
          if (json.containsKey("in_topic")) strcpy(in_topic, json["in_topic"]);
          if (json.containsKey("cmd_topic")) strcpy(cmd_topic, json["cmd_topic"]);
          if (json.containsKey("data_topic")) strcpy(data_topic, json["data_topic"]);
          if (json.containsKey("ack_topic")) strcpy(ack_topic, json["ack_topic"]);
          if (json.containsKey("config_topic")) strcpy(config_topic, json["config_topic"]);
          if (json.containsKey("connect_topic")) strcpy(connect_topic, json["connect_topic"]);
          if (json.containsKey("config_period")) strcpy(config_period, json["config_period"]);

        }
        else {
          Serial.println("Falha em ler o Arquivo");
        }
      }
    }
  } else {
    Serial.println("Falha ao montar o sistema de arquivos");
  }
  config_periodS = config_period;
  config_period_I = config_periodS.toInt();

}

//----------------- Salvando arquivo de configuracao ---------------------------
void saveConfigtoFile(char api_key[], char device_id[], char mqtt_server[],char mqtt_port[],char mqtt_login[],char mqtt_pass[])
{
  String SString;
  SString = String(sub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(cmd_channel); //sub
  SString.toCharArray(cmd_topic, SString.length()+1);
  SString = String(pub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(data_channel); //pub
  SString.toCharArray(data_topic, SString.length()+1);
  SString = String(sub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(ack_channel); //sub
  SString.toCharArray(ack_topic, SString.length()+1);
  SString = String(sub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(in_channel); //sub
  SString.toCharArray(in_topic, SString.length()+1);
  SString = String(sub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(config_channel); //sub
  SString.toCharArray(config_topic, SString.length()+1);
  SString = String(pub_dev_modifier) + String("/") + String(mqtt_login) + String("/") + String(connect_channel); //pub
  SString.toCharArray(connect_topic, SString.length()+1);

  Serial.println("Salvando Configuracao");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_login"] = mqtt_login;
  json["mqtt_pass"] = mqtt_pass;
  json["device_id"] = device_id;
  json["device_type"] = device_type;
  json["api_key"] = api_key;
  json["in_topic"] = in_topic;
  json["cmd_topic"] = cmd_topic;
  json["data_topic"] = data_topic;
  json["ack_topic"] = ack_topic;
  json["config_topic"] = config_topic;
  json["connect_topic"] = connect_topic;
  json["config_period"] = config_period;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Falha em abrir o arquivo com permissao de gravacao");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

//----------------- Configuracao --------------------------------
void configuration(char* configs, int value)
{
  String config_periodS;
  const char *rst = "reset";
  const char *prd = "period";
  char api_key[17];
  char device_id[17];
  char mqtt_server[64];
  char mqtt_port[5];
  char mqtt_login[32];
  char mqtt_pass[32];

  if (String(configs) == String(rst))
  {
    resetsettings=1;
  }

  else if (String(configs) == String(prd))
  {
    spiffsMount(mqtt_server, mqtt_port, mqtt_login, mqtt_pass, device_id, device_type, api_key, in_topic, cmd_topic, data_topic, ack_topic, config_topic, connect_topic, config_period);
    config_period_I = value;
    config_periodS = value;
    config_periodS.toCharArray(config_period,5);
    delay(500);
    saveConfigtoFile(api_key, device_id, mqtt_server, mqtt_port, mqtt_login, mqtt_pass);
  }
}

//----------------------------------------------------------------

//----------------- Criacao da Mensagem Json --------------------------------

char *jsonMQTTmsgDATA(const char *ts, const char *device_id, const char *metric1, float value1, const char *unit1, const char *metric2, float value2, const char *unit2)
{
  StaticJsonBuffer<220> jsonMQTT;
  JsonObject& jsonMSG = jsonMQTT.createObject();
  jsonMSG["deviceId"] = device_id;
  jsonMSG["metric1"] = metric1;
  jsonMSG["value1"] = value1;
  jsonMSG["unit1"] = unit1;
  jsonMSG["metric2"] = metric2;
  jsonMSG["value2"] = value2;
  jsonMSG["unit2"] = unit2;
  jsonMSG["fw"] = firmware_version;
  jsonMSG["ts"] = ts;

  jsonMSG.printTo(bufferJ, sizeof(bufferJ));
  return bufferJ;
}



char *jsonMQTTmsgCMD(const char *ts, const char *device_id, const char *cmd)
{
  StaticJsonBuffer<220> jsonMQTT;
  JsonObject& jsonMSG = jsonMQTT.createObject();
  jsonMSG["deviceId"] = device_id;
  jsonMSG["command"] = cmd;
  jsonMSG["ts"] = ts;
  jsonMSG.printTo(bufferJ, sizeof(bufferJ));
  return bufferJ;
}


char *jsonMQTTmsgCONNECTED(const char *ts, const char *device_id, const char *connect_msg)
{
  StaticJsonBuffer<220> jsonMQTT;
  JsonObject& jsonMSG = jsonMQTT.createObject();
  jsonMSG["deviceId"] = device_id;
  jsonMSG["connectmsg"] = connect_msg;
  jsonMSG["fw"] = firmware_version;
  jsonMSG["ts"] = ts;
  jsonMSG.printTo(bufferJ, sizeof(bufferJ));
  return bufferJ;
}


char *jsonMQTTmsgCONNECT(const char *ts, const char *device_id, const char *connect_msg, const char *metric1, int value1, const char *unit1, const char *metric2, int value2, const char *unit2)
{
  StaticJsonBuffer<220> jsonMQTT;
  JsonObject& jsonMSG = jsonMQTT.createObject();
  jsonMSG["deviceId"] = device_id;
  jsonMSG["connectmsg"] = connect_msg;
  jsonMSG["metric1"] = metric1;
  jsonMSG["value1"] = value1;
  jsonMSG["unit1"] = unit1;
  jsonMSG["metric2"] = metric2;
  jsonMSG["value2"] = value2;
  jsonMSG["unit2"] = unit2;

  jsonMSG["fw"] = firmware_version;
  jsonMSG["ts"] = ts;
  jsonMSG.printTo(bufferJ, sizeof(bufferJ));
  return bufferJ;
}
//----------------- Decodificacao da mensagem Json In -----------------------------
char *jsonMQTT_in_msg(const char msg[])
{
  const char *jdevice_id;
  const char *cmd = "null";
  const char *ts;

  StaticJsonBuffer<2048> jsonBuffer;
  JsonObject& jsonMSG = jsonBuffer.parseObject(msg);
  if (jsonMSG.containsKey("deviceId")) jdevice_id = jsonMSG["deviceId"];
  if (jsonMSG.containsKey("command")) cmd = jsonMSG["command"];
  if (jsonMSG.containsKey("ts")) ts = jsonMSG["ts"];
  char *command = (char*)cmd;
  return command;
}

//----------------- Decodificacao dos dados da mensagem Json In --------------------
char *jsonMQTT_in_data_msg(const char msg[])
{
  const char *jdevice_id;
  const char *cmd;
  const char *ts;
  const char *dt;

  StaticJsonBuffer<2048> jsonBuffer;
  JsonObject& jsonMSG = jsonBuffer.parseObject(msg);
  if (jsonMSG.containsKey("deviceId")) jdevice_id = jsonMSG["deviceId"];
  if (jsonMSG.containsKey("command")) cmd = jsonMSG["command"];
  if (jsonMSG.containsKey("value")) dt = jsonMSG["value"];
  if (jsonMSG.containsKey("ts")) ts = jsonMSG["ts"];
  char *data = (char*)dt;
  return data;
}


//----------------- Decodificacao da mensagem Json CFG -----------------------------
void *jsonMQTT_config_msg(const char msg[])
{
  int value = 0;
  const char *configs;
  char *conf;
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject& jsonMSG = jsonBuffer.parseObject(msg);
  if (jsonMSG.containsKey("config"))
  {
    configs = jsonMSG["config"];
    conf=strdup(configs);
    if (jsonMSG.containsKey("value")) value = jsonMSG["value"];
    configuration(conf, value);
    configured=1;
  }
}

//---------------------------------------------------------------------------


//----------------- Feedback para o ACK (no momento eh um placeholder: soh escreve via serial)--------------------------------
void simplef(int charcount) {
  enable=1;
  //int charcount=0;
  if (charcount>18) {
    //sucess = 1;
    Serial.println("Mensagem recebida pelo device B com sucesso");
  }
  else            {
    //sucess = 0;
    Serial.println("Mensagem nao entregue");
  }
}

//----------------- Publica a mensagem e aguarda o ACK--------------------------------
int messageACK(PubSubClient client, char topic[], char message[],char ack[], int charcount){
  client.publish(topic,message);
  //client.publish(topic, message);
  Serial.println("topic="+(String)topic);
  Serial.println("message="+(String)message);

  //enable=0;
  // t.once_ms(3000,simplef,charcount);
  //name_ok=0;
  return 1;
}

//----------------- Funcao para conectar ao broker MQTT e reconectar quando perder a conexao --------------------------------
int reconnect(PubSubClient client, char id[], const char *mqtt_login, const char *mqtt_pass) {
  int i=0;
  // Loop ate estar conectado
  while (!client.connected()) {
    Serial.print("Tentando conectar no Broker MQTT...");
    // Tentando conectar no broker MQTT (PS.: Nao usar dois clientes com o mesmo nome. Causa desconexao no Mosquitto)
    if (client.connect(id, mqtt_login, mqtt_pass)) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Falhou, rc=");
      Serial.print(client.state());
      Serial.println("Tentando conectar novamente em 3 segundos");
      // Esperando 3 segundos antes de re-conectar
      delay(3000);
      if (i==19) break;
      i++;
    }
  }
  return i;
}


//----------------- Funcao para enviar a temperatura --------------------------------

void sendMessage()
{
  senddata=1;
}

//----------------- Funcao para testar as mensagens de entrada ----------------------

char *testCMD(const char msgBufferIN[])
{
  char *cmd;
  String msgString ="null";
  String type;
  char typeC[32];
  int commaIndex;

  cmd=jsonMQTT_in_msg(msgBufferIN);
  msgString = String(cmd);
  commaIndex = msgString.indexOf('-');
  type = msgString.substring(0, commaIndex);
  if (commaIndex == -1)
  {
    type = msgString.substring(0, msgString.length());
  }
  type.toCharArray(typeC,type.length()+1);
  typeC[type.length()+1] = '\0';

  return typeC;

}

//-----------------------------------------------------------------------------------
void LEDSwitch(int LED1, int LED2)
{
  //LEDState = digitalRead(LED1);
  Serial.print("Digital State: ");
  Serial.println(LEDState);
  if (LEDState)
  {
    analogWrite(LED1, 0);
    analogWrite(LED2, 0);
  }
  else
  {
    analogWrite(LED1, 1024);
    analogWrite(LED2, 1024);
  }
  LEDState=!LEDState;
}

void LEDOn(int LED1, int LED2, int pwm)
{

  analogWrite(LED1, pwm);
  analogWrite(LED2, pwm);
  LEDState=1;
}

void LEDOff(int LED1, int LED2)
{
  analogWrite(LED1, 0);
  analogWrite(LED2, 0);
  LEDState=0;
}
