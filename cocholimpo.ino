#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SSD1306.h>

//Deixe esta linha descomentada para compilar o Master
//Comente ou remova para compilar o Slave
#define MASTER
 
// Definição dos pinos Esp32 Lora (comente caso seja Esp32 MCU)
#define SCK 5 // GPIO5 -- SX127x's SCK
#define MISO 19 // GPIO19 -- SX127x's MISO
#define MOSI 27 // GPIO27 -- SX127x's MOSI
#define SS 18 // GPIO18 -- SX127x's CS
#define RST 14 // GPIO14 -- SX127x's RESET
#define DI00 26 // GPIO26 -- SX127x's IRQ (Interrupt Request)

// Definição dos pinos Esp32 MCU (comente caso seja Esp32 Lora)
//#define SCK 18 // GPIO18 -- SX127x's SCK
//#define MISO 19 // GPIO19 -- SX127x's MISO
//#define MOSI 23 // GPIO23 -- SX127x's MOSI
//#define SS 2 // GPIO2 -- SX127x's CS
//#define RST 34 // GPIO14 -- SX127x's RESET
//#define DI00 35 // GPIO26 -- SX127x's IRQ (Interrupt Request)

#define SDA 4
#define SCL 15
 
#define BAND 915E6 //Frequência do radio - exemplo : 433E6, 868E6, 915E6

//Constante que o Slave retorna junto com os dados para o Master
const String SETDATA = "setdata=";
 
//Estrutura com os dados do sensor
typedef struct {
  int count;
  int light;
}Data;
 
//Variável para controlar o display
SSD1306 display(0x3c, SDA, SCL);

void setupDisplay() {
  //O estado do GPIO16 é utilizado para controlar o display OLED
  pinMode(16, OUTPUT);
  //Reseta as configurações do display OLED
  digitalWrite(16, LOW);
  //Para o OLED permanecer ligado, o GPIO16 deve permanecer HIGH
  //Deve estar em HIGH antes de chamar o display.init() e fazer as demais configurações,
  //não inverta a ordem
  digitalWrite(16, HIGH);
 
  //Configurações do display
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

//Configurações iniciais do LoRa
void setupLoRa() { 
  //Inicializa a comunicação
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI00);
 
  //Inicializa o LoRa
  if(!LoRa.begin(BAND)) {
    //Se não conseguiu inicializar, mostra uma mensagem no display
    display.clear();
    display.drawString(0, 0, "Erro ao inicializar o LoRa!");
    display.display();
    Serial.println("Erro ao inicializar o LoRa!");
    while (1);
  }

  Serial.println("LoRa iniciada com sucesso.");
 
  //Ativa o crc
  LoRa.enableCrc();
  //Ativa o recebimento de pacotes
  LoRa.receive();
}


//===========================================================================MASTER=======================================================================================
//Compila apenas se MASTER estiver definido no arquivo principal
#ifdef MASTER

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> //5.7.0

const char* ssid     = "Sao_Joaquim_2.4";
const char* password = "saojoaquim";
//const char* ssid     = "CampoLab";
//const char* password = "campolab@2019";

//const char* MQTT_SERVER = "iot.plug.farm";
const char* MQTT_SERVER = "mqtt.eclipse.org";

WiFiClient CLIENT;
PubSubClient MQTT(CLIENT);
 
void setup() {
  Serial.begin(115200);
  //Chama a configuração inicial do display
  setupDisplay();
  //Chama a configuração inicial do LoRa
  setupLoRa();
  
  display.clear();
  display.drawString(0, 0, "Master");
  display.drawString(0, 32, "Connecting to ");
  display.drawString(0, 48, ssid);
  display.display();

  Serial.println("Master");
  Serial.println("Connecting to " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  display.clear();
  display.drawString(0, 0, "WiFi connected");
//  display.drawString(0, 32, "IP address:");
//  display.drawString(0, 48, WiFi.localIP());
  display.display();

  Serial.println();
  
  MQTT.setServer(MQTT_SERVER, 1883);
  
  if(!MQTT.connected()) {
    display.clear();
    display.drawString(0, 0, "Servidor MQTT");
    display.drawString(0, 16, "Connected");
    display.display();

    Serial.println("Servidor MQTT");
    Serial.println("Connected");
  }
}

void loop() {
  if(!MQTT.connected()) {
    reconnect();
  }

  MQTT.loop();
 Serial.println("tentou receber");
  //Verificamos se há pacotes para recebermos
  receive();
}

void receive() {
  //Tentamos ler o pacote
  int packetSize = LoRa.parsePacket();
   
  //Verificamos se o pacote tem o tamanho mínimo de caracteres que esperamos
  if(packetSize > SETDATA.length()) {
    String received = "";
    
    //Armazena os dados do pacote em uma string
    for(int i=0; i<SETDATA.length(); i++) {
      received += (char) LoRa.read();
    }
 
    //Se o cabeçalho é o que esperamos
    if(received.equals(SETDATA)) {
      //Fazemos a leitura dos dados
      Data data;
      LoRa.readBytes((uint8_t*)&data, sizeof(data));

      //envia dados
      enviaData(data);
    }
  Serial.println("recebeu");
  }
}

void showData(Data data) {
  //Mostra no display os dados e o tempo que a operação demorou
  display.clear();
  display.drawString(0, 0, "Pacote: " + String(data.count));
  display.drawString(0, 32, "Luz: " + String(data.light));
  display.drawString(0, 48, String(LoRa.packetRssi(), DEC)+ "dB");
  display.display();

  Serial.println("Pacote: " + String(data.count));
}

void enviaData(Data data) {
  DynamicJsonBuffer jsonBuffer;
  JsonObject & root = jsonBuffer.createObject();

  root["luz"] = data.light;
  root["pacote"] = data.count;
  root["id"] = "Iot Campo";
  String msg;
  root.printTo(msg);
  
  //Mostramos os dados no display
  showData(data);
  
  MQTT.publish("hello/temp", msg.c_str());
}

void reconnect() {
  while(!MQTT.connected()) {
    if(MQTT.connect("ESP32-TestePorta")) {
      //MQTT.subscribe("hello/led");
    } else {
      display.clear();
      display.drawString(0, 0, "Servidor MQTT");
      display.drawString(0, 16, "Tentanto conectar...");
      display.display();
      delay(3000);

      Serial.println("Tentanto conectar...");
    }
  }
}
#endif


//=============================================================//================SLAVE===========================//========================================================
//Compila apenas se MASTER não estiver definido no arquivo principal
#ifndef MASTER

//Intervalo entre os envios
#define INTERVAL 2500

//Tempo do último envio
long lastSendTime = 0;

int count = 0;
int pinLight = 33;

void setup() {
  Serial.begin(115200);
  
  //Chama a configuração inicial do display
  setupDisplay();
  //Chama a configuração inicial do LoRa
  setupLoRa();
  
  display.clear();
  display.drawString(0, 0, "Slave");
  display.drawString(0, 25, "FAZENDA RENTAVEL");
  display.display();
}

void loop() {
  //Se passou o tempo definido em INTERVAL desde o último envio
  if(millis() - lastSendTime > INTERVAL) {
    //Marcamos o tempo que ocorreu o último envio
    lastSendTime = millis();
    //Faz a leitura dos dados
    Data data = readData();
    Serial.println("Criando pacote para envio");
    showSentData(data);
  }
}

//Função onde se faz a leitura dos dados
Data readData() {
  count++; 
  Data data;
  data.count = count;
  data.light = analogRead(pinLight);
  return data;
}

void showSentData(Data data) {
  //Montando e Enviando pacote
  LoRa.beginPacket();
  LoRa.print(SETDATA);
  LoRa.write((uint8_t*)&data, sizeof(data));
  //Finaliza e envia o pacote
  LoRa.endPacket();
  
  //Mostra no display
  display.clear();
  display.drawString(0, 0, "Pacote: " + String(data.count));
  display.drawString(0, 48, "Luz: " + String(data.light));
  display.display();
}
#endif
