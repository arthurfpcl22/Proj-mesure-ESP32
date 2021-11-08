#include <Filters.h> 
#include <SimpleDHT.h>
#include <math.h>
#include <Arduino.h>
#include <driver/adc.h>
#include <WiFi.h>
#include <PubSubClient.h>


//#define ID_MQTT  "esp32_mqtt"
//#define ADC_BITS    10
//#define ADC_COUNTS  (1<<ADC_BITS)
//Definições para sensor DHT22
int pinDHT22=26;// Pino de entrada digital DHT22
SimpleDHT22 dht22(pinDHT22);// Inicialização biblioteca SimpleDHT22
//Definições para MQ-2
// Pino de entrada analógica MQ-2.
int porta_digital=22;
int Gas_analog = 35;
int tensao_analog=32;
int chuva_analog=33;
int corrente_analog=34;
int chamas_analog=25;
float valor_chamas=0;
float valor_sensor_gas=0;
float valor_tensao_gas=0;
float RS_arpuro=0;//Variável para armazenar valores de Resistência RS obtidos em ar puro.
float R0;//Variável a ser calibrada que oferece uma referência para análise das curvas no datasheet do MQ-2.
int cont_cali=0;// Contador de número de amostras na calibração.
float m = -0.473;;// Coeficiente angular da curva para CO disponível no datasheet.
float b = 1.412;;// Coeficiente linear  da curva para CO disponível no datasheet.
float RS = 0;// Variável que armazena a resistência sensível a atuação do gás.
float log_x = 0;// Variável que armazena valor log_x para posterior cálculo em ppm.
float ppm = 0;// Variável para armazenamento das concentrações em ppm.
float nivel_chama=0;
float nivel_chuva=0;
float testFrequency = 50;     // test signal frequency (Hz)
float windowLength = 100/testFrequency;
int RawValue_tensao = 0;     
float Volts_TRMS;     // estimated actual voltage in Volts
float intercept_tensao = 0;  // to be adjusted based on calibration testin
float slope_tensao = 2.52873;
int RawValue_corrente = 0;     
float Corrente_TRMS;     // estimated actual voltage in Volts
float intercept_corrente = 0;  // to be adjusted based on calibration testin
float slope_corrente= 0.0692;
float tensao=0;
float Irms=0;
float potencia=0;;
unsigned long previousMillis = 0;
const char* ssid="Zack2019";// Definir SSID da rede
const char* pass="Dani2808";// Senha da Rede
const char* broker="192.168.1.141";
RunningStatistics inputStats_tensao;
RunningStatistics inputStats_corrente;
//WiFiClient espClient;
//PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  pinMode(tensao_analog,INPUT);
  pinMode(corrente_analog,INPUT);
  pinMode(chamas_analog,INPUT);
  pinMode(Gas_analog,INPUT);
  pinMode(chuva_analog,INPUT);
  //setupWifi();
  //client.setServer(broker, 1883);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  analogReadResolution(10);
  inputStats_tensao.setWindowSecs( windowLength );
  inputStats_corrente.setWindowSecs( windowLength );
  CalibrateR0();// Calibração do sensor para se encontrar R0, ou seja resistência com ar limpo.

}

void loop() {
  // put your main code here, to run repeatedly:
    /*if (!client.connected()){
    reconnect();
    }
    client.loop();*/
   medicao_energia();
   unsigned long printPeriod = 4000; //Measuring frequency, every 1s, can be changed
   if ((unsigned long)(millis() - previousMillis) >= printPeriod) {
    previousMillis = millis();
    ler_DHT();// Leitura dos valores de temperatura e umidade.
    ler_MQ_2();// Leitura dos valores de CO em PPM.
    estado_porta();
    ler_chuva();
    medir_chamas();
    Serial.print("Valor tensão:");
    Serial.println(tensao);
    char msgBuffer[20];
    //client.publish("Tensao",dtostrf(tensao,5,2,msgBuffer));
    Serial.print("Valor corrente:");
    Serial.println(Irms);
    //client.publish("Corrente",dtostrf(Irms,5,2,msgBuffer));
    Serial.print("Valor potencia:");
    Serial.println(potencia);
    //client.publish("Potencia",dtostrf(potencia,5,2,msgBuffer));
    
   }
}

void ler_DHT(){
  Serial.println("========================");
  Serial.println("Sample DHT22");
  float temperature=0;// Variável de armazenamento de temperatura.
  float humidity=0;// Variável de armazenamento de Umidade.
  int err=SimpleDHTErrSuccess;// Variável para reconhecimento de erros na leitura digital dos dados.
   
 
    if ((err=dht22.read2(&temperature,&humidity,NULL)) != SimpleDHTErrSuccess){// Condição em que ocorre erro.
    Serial.print("Read DHT22 failed,err=");
    Serial.println(err);
    return;
    }
    char msgBuffer[20];
    Serial.print("Temperatura:");// Print dos valores de temperatura e umidade.
    Serial.print((float)temperature);Serial.println("C");
    Serial.print("Umidade:");
    Serial.print((float)humidity);Serial.println("RH");
    //client.publish("Temperatura",dtostrf(temperature,5,2,msgBuffer));
    //client.publish("Umidade",dtostrf(humidity,5,2,msgBuffer));
    
  
}

void CalibrateR0(){
  Serial.println("calibrando R0...");
  
  while (cont_cali <5) {// Coleta de 20 amostras de RS em um intervalo de 3 minutos e 20 segundos.
  valor_sensor_gas=analogRead(Gas_analog);
  valor_tensao_gas=((valor_sensor_gas * 3.3)/1023);
  Serial.print("Tensao Gás:");
  Serial.println(valor_tensao_gas);
  RS_arpuro +=(1023-(valor_sensor_gas))/(valor_sensor_gas);
  delay(10000);
  cont_cali++;
  }
  
  RS_arpuro=RS_arpuro/cont_cali;// Cálculo valor médio de RS
  R0=RS_arpuro/9.8;// Cálculo valor de RO de acordo com o gráfico do datasheet.
  
  Serial.print("Valor R0:");
  Serial.println(R0);
}

void ler_MQ_2(){
  
  valor_sensor_gas=analogRead(Gas_analog);
  valor_tensao_gas=((valor_sensor_gas * 3.3)/1023);// Calcula valor de tensão na entrada adc do esp
  Serial.print("Tensao Gás:");
  Serial.println(valor_tensao_gas);
  RS=(1023-(valor_sensor_gas))/(valor_sensor_gas); // Calculo valor de RS
  
  log_x=((log10(RS/R0)-b)/m);// Calculo linear do log_x
  ppm=pow(10,log_x);// Converção em ppm.
  Serial.print("Concentracao em ppm:");
  Serial.println(ppm);
  char msgBuffer[20];
  //client.publish("Gas",dtostrf(ppm,5,2,msgBuffer));
}
void medir_chamas(){
  valor_chamas=analogRead(chamas_analog);
  Serial.print("Nível de chama:");
  
  nivel_chama=log10(1023/valor_chamas)*50;
  if (nivel_chama>100){
    nivel_chama=100;
  }
  
  Serial.println(nivel_chama);
  char msgBuffer[20];
  //client.publish("Nível chamas",dtostrf(nivel_chama,5,2,msgBuffer));
}
void medicao_energia(){
 
  tensao = Medetensao();
  
  Irms = Medecorrente();
  
  potencia = tensao * Irms;;
  
  
}
float Medetensao() {
    RawValue_tensao = analogRead(tensao_analog);  // read the analog in value:
    inputStats_tensao.input(RawValue_tensao);
   
   // log to Stats function
      //Serial.println(inputStats_tensao.sigma());
      return inputStats_tensao.sigma()* slope_tensao + intercept_tensao;
     
}
float Medecorrente(){
    RawValue_corrente = analogRead(corrente_analog);  // read the analog in value:
    inputStats_corrente.input(RawValue_corrente);       // log to Stats function   // update time
      if (inputStats_corrente.sigma()<10){
        return 0;
      }
      else{
      return inputStats_corrente.sigma()* slope_corrente + intercept_corrente;
      }
}
void ler_chuva(){
  float leitura_chuva = analogRead(chuva_analog);
  nivel_chuva=log10(1023/leitura_chuva)*150;
  // Realizar o print da leitura no serial
  Serial.print("Leitura do Sensor de Chuva:");
  Serial.println(nivel_chuva);
  char msgBuffer[20];
  //client.publish("Nível chuva",dtostrf(nivel_chuva,5,2,msgBuffer));
}
void estado_porta(){
  int estado=digitalRead(porta_digital);
  Serial.print("Estado porta:");
  if (estado==0){
    Serial.println("Aberta");
  }
  else{
    Serial.println("Fechada");
  }
  char msgBuffer[20];
  //client.publish("Estado porta",dtostrf(estado,5,2,msgBuffer));
}
/*void setupWifi(){
  delay(100);
  Serial.print("\nCOnnecting to");
  Serial.println(ssid);

  WiFi.begin(ssid,pass);// Inicialiazaçãi WI-Fi
  int cont=0;
  while(WiFi.status() != WL_CONNECTED){// Rotina de verificação da conexão Wi-FI
    delay(1000);
    Serial.print("-");
    cont++;
    if (cont>10){
      ESP.restart();
      }
    
  }
  }
void reconnect(){// Verifica conexão MQTT
  while(!client.connected()){
    Serial.print("\n Connecting to ");
    Serial.println(broker);
    if(client.connect(ID_MQTT)){
      Serial.print("\nConnected to ");
      Serial.println(broker);
    }else {
      Serial.println("\n Trying connect again");
      delay(5000);
      
    }
  }
  
  }*/
