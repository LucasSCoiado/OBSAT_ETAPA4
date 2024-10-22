//----- Declaração das Bibliotecas Principais do Satélite -----//
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
#include <TinyGPS.h>
#include <LoRa.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "math.h"
#include "DHT.h"
#include "SparkFun_AS3935.h"
//----- Definição das constantes de cálculo -----//

#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/
#define SEALEVELPRESSURE_HPA (1013.25)
#define PATM 101325 // Pressão no nível do mar
#define GRAV 6.67408e-11 // Contante gravitacional
#define RAIO 6375109.0 // Raio da terra
#define MASS 5.972e24 // Massa da terra
#define AS3935_ADDR 0x03 
#define INDOOR 0x12 
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
#define DHTTYPE DHT11

SparkFun_AS3935 lightning(AS3935_ADDR);
const int lightningInt = 33; 

#define MPU6050_DEVICE_ID 0x68 // Define o valor correto do valor MPU6050_WHO_AM_I
const int DHTPIN = 4;




DHT dht(DHTPIN, DHTTYPE);

int vSensor;
//----- Pinos de Controle do LoRa -----//
/*
#define ss 5 // CHIP SELECT do LoRa - RA02
#define rst 14 // RESET do LoRa - RA02
#define dio0 2 // DIGITAL I/O do LoRa - RA02
*/

/**
 * Enumeração dos estados do satélite
 */
enum EstadoMissao{
  INICIALIZANDO, // Estado de inicialização da missão
  TESTE,         // Estado de teste do satelite
  SUBIDA,        // Estado de ascenção do satelite no balão atmosférico
  EMERGENCIA,    // Estado de emergência que protege o satelite contra destruição
  EXECUCAO,      // Estado em que o satélite está posicionado e começa a receber as coordenadas do caminhão
  DESCIDA,       // Estado de queda controlada do satélite
  RESGATE        // Estado que o satélite sinaliza para ser resgatado
};

EstadoMissao estado;     // Declara o estado dos periféricos
File myFile;             // Delaração do arquivo que será salvo no cartão SD no sistema de arquivos FAT
Adafruit_MPU6050 mpu;    // Declara o sensor Acelerômetro e Giroscópio
Adafruit_BMP280 bmp_ext; // Declara o sensor de Pressão Barométrica
WiFiMulti wifiMulti;     // Declara a biblioteca de multiplas conexões WiFi

const int CS = 2;        // Declara o pino do Chip Select do cartão SD
const char* ssid = "CLARO_827E8C"; // Declara o nome da rede WiFi que irá se conectar
const char* password_wifi = "coy07761"; // Declara a senha da rede WiFi que irá se conectar

String serverName = "https://obsat.org.br/teste_post/envio.php"; // URL do servidor que irá enviar a requisição POST com os dados
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

double angulo_atual; // Declaração do angulo relativo atual em variável global
double posicao_atual[3]; // Declaração da posição relativa atual em variavel global
double velocidade_atual[3]; // Declaração da velocidade relativa atual em variável global
double velocidade[3]; // Declaração da velocidade anterior relativa em variável global
unsigned long tempo; // Declaração do tempo anterior

bool status_sd;  // Status do cartão SD
bool status_mpu; // Status do sensor acelerômetro
bool status_bmp;  // Status do sensor de pressão

bool status_msg;  // Status do envio da mensagem
bool status_wifi;  // Status da conexão wifi
  
int intVal = 0;
int noise = 2; // Value between 1-7 
int disturber = 2; // Value between 1-10
int threshVal;

void WriteFile(const char * path, const char * message){
  myFile = SD.open(path, FILE_WRITE);
  if (myFile) {
    Serial.printf("Writing to %s ", path);
    myFile.println(message);
    myFile.close(); // close the file:
    Serial.println("completed.");
  } 
  else {
    Serial.println("error opening file ");
    Serial.println(path);
  }
}

void ReadFile(const char * path){
  // open the file for reading:
  myFile = SD.open(path);
  if (myFile) {
     Serial.printf("Reading file from %s\n", path);
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close(); // close the file:
  } 
  else {
    Serial.println("error opening test.txt");
  }
}




double getAceleracao(int eixo){
  sensors_event_t a,g,temp;
  mpu.getEvent(&a, &g, &temp);

  return Serial.print("Aceleração X: " +(String)a.acceleration.x+"--Aceleração Y: "+(String)a.acceleration.y+"--Aceleração Z: "+(String)a.acceleration.z);
}

double getGiroscopio(int eixo){
  // TODO: retorna a angulação
  sensors_event_t a,b,c;
  mpu.getEvent(&a,&b,&c);

  return Serial.print("Giro X: " +(String)b.gyro.x+"--Giro Y: " +(String)b.gyro.y+"--Giro Z: " +(String)b.gyro.z);
}

double getTemperatura(){
  float temp = bmp_ext.readTemperature(); 
  return temp;
}

double getPressao(){
  
  return Serial.print(bmp_ext.readPressure());
}

double getHumidade(){
  float h = dht.readHumidity();
  if (isnan(h)) {
    
    return Serial.println(F("Failed to read from DHT sensor!"));
  }
  return Serial.print("Humidade " +(String)h+"%");
}

double getLumi(int lum){
  float valorLDR;
  int LDR;
  valorLDR = analogRead(A0);
  LDR = map(valorLDR, 0, 4095, 0, 100);
  return LDR;
}





void setup() {
//----- Inicialização das Interfaces de Comunicação Seriais -----//
  
  Serial.begin(115200);   // Inicia a comunicação serial 1 que comunica com o conversor USB/Serial
  Serial2.begin(115200);   // Inicia a comunicação serial 2 que comunica com o módulo GSM/GPS/GPRS
  
  pinMode(A0, INPUT);
  //----- Verificação Inicial do WiFi -----//
  
  wifiMulti.addAP(ssid, password_wifi); // Conectando à rede WiFi
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.print("Falha ao conectar à rede: ");
    Serial.print(ssid);
    Serial.print(" com a senha: ");
    Serial.print(password_wifi);
  }
  Serial.print("Conectado à rede WiFi com IP Address: ");
  Serial.println(WiFi.localIP());
  status_msg = false;

  //----- Verificação inicial do Cartão SD -----//
  
  status_sd = SD.begin(CS);
  if (!status_sd) {
    Serial.println("Falha no módulo do cartão SD!");
    delay(100);
  }

  //----- DHT11------//
  
  dht.begin();
  //----- Verificação inicial do Giroscópio e Acelerômetro -----//
  
  status_mpu = mpu.begin(0x68);
  if (!status_mpu) {
    Serial.println("Falha no módulo MPU6050!");
    delay(100);
  } else {
    //----- Ranges possíveis para acelerômetro -----//
    //MPU6050_RANGE_2_G
    //MPU6050_RANGE_4_G
    //MPU6050_RANGE_8_G
    //MPU6050_RANGE_16_G
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    //----- Ranges possíveis para giroscópio -----//
    //MPU6050_RANGE_250_DEG
    //MPU6050_RANGE_500_DEG
    //MPU6050_RANGE_1000_DEG
    //MPU6050_RANGE_2000_DEG
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    //----- Possíveis bandas de filtragem -----//
    //MPU6050_BAND_260_HZ
    //MPU6050_BAND_184_HZ
    //MPU6050_BAND_94_HZ
    //MPU6050_BAND_44_HZ
    //MPU6050_BAND_21_HZ
    //MPU6050_BAND_10_HZ
    //MPU6050_BAND_5_HZ
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
    Serial.println("Adafruit MPU6050 test!");
  
    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
      delay(10);
      }
    }
    Serial.println("MPU6050 Found!");

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("");
    delay(100);
  }

  //----- Verificação inicial do Sensor de Pressão -----//
  Serial.println(F("BMP280 test"));
  bool status;
  status_bmp = bmp_ext.begin(0x76);
  if (!status_bmp) {
    Serial.println("Falha no módulo BMP280!");
    delay(100);
  }
  Serial.println("-- Default Test --");
  delay(1000);

  Serial.println();
  //----- Inicialização do Módulo LoRa Ai Thinker -----//
  
  /*
  LoRa.setPins(ss, rst, dio0);          // definindo os pinos de controle do LoRa
  cubeSat.setLed(L1, HIGH);             // indicador LED da inicialização do LoRa
  Serial.print("Inicialização LoRa ");  // Aviso de inicialização do LoRa
  while (!LoRa.begin(62.5E3)) {          // Enquanto o LoRa não inicia o programa permanece no ciclo
    Serial.print(".");                  // Mensagem de espera
    delay(500);                         // Espera de meio segundo
  }
  Serial.println("");                   // Encerramento da mensagem de espera
  LoRa.setSyncWord(0xF3);               // Mensagem de sincronização
  cubeSat.setLed(L1, LOW);              // desligar LED indicador de inicializador do LoRa
  */

  //----- Inicialização das Variáveis Globais -----//

  estado = INICIALIZANDO;  // Seta o estado inicial como iniciando a missão
  angulo_atual = 0;        // zerando o angulo atual
  posicao_atual[0] = 0;    // zerando a posição atual em x
  posicao_atual[1] = 0;    // zerando a posição atual em y
  posicao_atual[2] = 0;    // zerando a posição atual em z
  velocidade_atual[0] = 0; // zerando a velocidade atual em x
  velocidade_atual[1] = 0; // zerando a velocidade atual em y
  velocidade_atual[2] = 0; // zerando a velocidade atual em z
  velocidade[0] = 0;       // zerando a velocidade anterior em x
  velocidade[1] = 0;       // zerando a velocidade anterior em y
  velocidade[2] = 0;       // zerando a velocidade anterior em z
  tempo = micros();        // definindo o offset do tempo inicial

  //pinMode(27, OUTPUT);   // Pino definido como output
  //digitalWrite(18, LOW); // Definir para nível lógico baixo
  //digitalWrite(18, HIGH);// Definir para nível lógico alto

  //RAIOS
  pinMode(lightningInt, INPUT); 

  Serial.begin(115200); 
  Serial.println("AS3935 Franklin Lightning Detector"); 

  Wire.begin(); // Begin Wire before lightning sensor. 

  if( !lightning.begin() ) { // Initialize the sensor. 
    Serial.println ("Lightning Detector did not start up, freezing!"); 
    while(1); 
  }
  else{
    Serial.println("Schmow-ZoW, Lightning Detector Ready!");
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(getAceleracao(A4));
  Serial.println("");
  delay(1000);

  Serial.println(getGiroscopio(A4));
  Serial.println("");
  delay(1000);
  
  Serial.print(getTemperatura());
  Serial.print("°C");
  Serial.println("");
  delay(1000);

  Serial.print(getPressao());
  Serial.print("hPa");
  Serial.println("");
  delay(1000);

  Serial.print( getHumidade());
  Serial.print("%");
  Serial.println("");
  delay(1000);

  int lumin = getLumi(A0);
  Serial.print(lumin);
  Serial.print("%");
  Serial.println("");
  delay(1000);
  //--------------------RAIOS-------------------------------------
  if(digitalRead(lightningInt) == HIGH){
    // Hardware has alerted us to an event, now we read the interrupt register
    // to see exactly what it is. 
    intVal = lightning.readInterruptReg();
    if(intVal == NOISE_INT){
      Serial.println("Nada."); 
      // Too much noise? Uncomment the code below, a higher number means better
      // noise rejection.
      //lightning.setNoiseLevel(setNoiseLevel); 
    }
    else if(intVal == DISTURBER_INT){
      Serial.println("Disturber."); 
      // Too many disturbers? Uncomment the code below, a higher number means better
      // disturber rejection.
      //lightning.watchdogThreshold(threshVal);  
    }
    else if(intVal == LIGHTNING_INT){
      Serial.println("Lightning Strike Detected!"); 
      // Lightning! Now how far away is it? Distance estimation takes into
      // account any previously seen events in the last 15 seconds. 
      byte distance = lightning.distanceToStorm(); 
      Serial.print("Aproximadamente: "); 
      Serial.print(distance); 
      Serial.println("km de distancia!"); 
    }
  }
  delay(100); // Slow it down.
  //--------------------------------------------------------------
  switch(estado){
    case INICIALIZANDO: 
      break;
    case TESTE: 
      break;
    case SUBIDA: 
      break;
    case EMERGENCIA: 
      break;
    case EXECUCAO: 
      break;
    case DESCIDA: 
      break;
    case RESGATE: 
      break;
    default: 
      break;
  }
  //---------------------------------------------------------------------------------
  //         Rotinas de reorganização das variáveis incrementais
  //---------------------------------------------------------------------------------
  
  //----- Rotina de Envio de Mensagens Utilizando o Long Range (LoRa) Radio -----//
  /*
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  */

  //----- Rotina de Envio de Mensagens Utilizando o WiFi -----//
  if(status_msg){
    HTTPClient http;   
     
    http.begin(serverName);  
    http.addHeader("Content-Type", "application/json");         
       
    StaticJsonDocument<384> doc;
  
    doc["equipe"] = 1;
    doc["bateria"] = 100;
    doc["temperatura"] = getTemperatura();
    doc["pressao"] = getPressao();
    
    JsonArray giroscopio = doc.createNestedArray("giroscopio");
    //giroscopio.add(rotacao[0]);
    //giroscopio.add(rotacao[1]);
    //giroscopio.add(rotacao[2]);
    
    JsonArray acelerometro = doc.createNestedArray("acelerometro");
    //acelerometro.add(aceleracao[0]);
    //acelerometro.add(aceleracao[1]);
    //acelerometro.add(aceleracao[2]);
    
    JsonObject payload = doc.createNestedObject("payload");
    payload["nome"] = "spacegran";
    payload["saude"] = "100";
    payload["radiacao"] = "100";
    payload["luv"] = "100";
    payload["temp"] = "100";
    
    String requestBody;
    
    serializeJson(doc, requestBody);
    
    //-------------------------------------------------//
    int httpResponseCode = http.POST(requestBody);
    if(httpResponseCode>0){
      String response = http.getString();                       
      Serial.println(httpResponseCode);   
      Serial.println(response);
    }
    else {
      Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    status_msg = true;
  }
  vSensor = analogRead ( A0 ) ;  // lê o pino de entrada analógica 0 
  Serial. print ( vSensor, DEC ) ;  // imprime o valor lido 
  Serial. println ( " \n " ) ;  // imprime um espaço entre os números 
  delay ( 1000 ) ;  // espera 100ms pela próxima leitura 
}
