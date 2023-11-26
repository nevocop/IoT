/***********************************************************************************************************
*                                                                                                          *
*  Firmware base para el Laboratorio #1 del curso Diseño de Dispositivos IoT de la Maestría en Ingeniería  *
*                                                                                                          *
*                             Monitoreo de ambiente y control de dispositivos                              *
*                                                                                                          *
***********************************************************************************************************/
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include "DHT.h"
#include <SimpleTimer.h>

#define pinDHT 19  // Asignación del pin para el sensor
#define DHTTYPE DHT22 // Identificación del tipo de sensor a utilizar
DHT sensorTH(pinDHT, DHTTYPE); // Creación de objeto denominado sensorTH
const char *SSID = "FABABE";
const char *SSIDPass = "T9xz362021";
const int fan = 25;

//*******************
//SimpleTimer Timer;
//*******************

//---------------------------------------------------
/*****************************************************************************************************
 *                                Función control_LED y BUZZER                                              *
*****************************************************************************************************/
void control_LED(int LED_no, int LED_pin, boolean &estado){
  estado = !estado;
  digitalWrite(LED_pin, estado);
  String text = (estado)? "ON" : "OFF";
  Serial.println("LED"+String(LED_no)+" es "+text);
} 

void control_Buzzer(int Buzzer_no, int Buzzer_pin, boolean &estado){
  estado = !estado;
  digitalWrite(Buzzer_pin, estado);
  String text = (estado)? "ON" : "OFF";
  Serial.println("Buzzer"+String(Buzzer_no)+" es "+text);
}
//-------------------------------------------------------

// Definición de nombres de los dispositivos
char dispositivo1[] = "Led Blanco";
char dispositivo2[] = "Led Verde";
char dispositivo3[] = "Buzzer";
//-------------------------------------------------------
// Definir los GPIO conectados con los LED y sus respectivos botones
static uint8_t LED1 = 23;  //D23
static uint8_t LED2 = 22;  //D22
static uint8_t BZ1 = 26;   //D26
//-------------------------------------------------------
static uint8_t WIFI_LED    = 2;   //D2
static uint8_t gpio_reset = 0;    // Reset de la ESP32
//-------------------------------------------------------
/* Variables para leer los estados de los pines */
// Estado LED y BUZZER
bool ESTADO_LED1 = LOW; //Define el estado del LED 1
bool ESTADO_LED2 = LOW; //Define el estado del LED 2
bool ESTADO_BZ1 = LOW; //Defiene el estado del Buzzer 1
//---------------------------------------------------
//El framework proporciona algunos tipos de dispositivos estándar 
// como switch, lightbulb, fan, temperature sensor.
static Switch my_switch1(dispositivo1, &LED1);
static Switch my_switch2(dispositivo2, &LED2);
static Switch my_switch3(dispositivo3, &BZ1);

//************************************
static TemperatureSensor temperature("Visor Temperatura");
static TemperatureSensor humidity("Visor Humedad");
//***************************************

//---------------------------------------------------

/****************************************************************************************************
*                                    Función sysProvEvent                                           *
*****************************************************************************************************/
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {      
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
        Serial.printf("\nAprovisionamiento iniciado con la red llamada \"%s\" y SSIDPass \"%s\" en BLE\n", SSID, SSIDPass);
        printQR(SSID, SSIDPass, "ble");
#else
        Serial.printf("\nAprovisionamiento iniciado con la red llamada \"%s\" y SSIDPass \"%s\" en SoftAP\n", SSID, SSIDPass);
        printQR(SSID, SSIDPass, "softap");
#endif        
        break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.printf("\nConectado a Wi-Fi!\n");
        digitalWrite(WIFI_LED, HIGH);
        break;
    }
}

/****************************************************************************************************
*                                     Función write_callback                                        *
*****************************************************************************************************/
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *nombre_dispositivo = device->getDeviceName();
    const char *nombre_parametro = param->getParamName();
    //----------------------------------------------------------------------------------
    if(strcmp(nombre_dispositivo, dispositivo1) == 0) {
      
      Serial.printf("Lightbulb1 = %s\n", val.val.b? "true" : "false");
      
      if(strcmp(nombre_parametro, "Power") == 0) {
        ESTADO_LED1 = val.val.b;
        ESTADO_LED1 = !ESTADO_LED1;
        control_LED(1, LED1, ESTADO_LED1);
      }
    }
   
    //----------------------------------------------------------------------------------
    else if(strcmp(nombre_dispositivo, dispositivo2) == 0) {
      
      Serial.printf("Valor del pulsador = %s\n", val.val.b? "true" : "false");

      if(strcmp(nombre_parametro, "Power") == 0) {
        ESTADO_LED2 = val.val.b;
        ESTADO_LED2 = !ESTADO_LED2;
        control_LED(2, LED2, ESTADO_LED2);
      }
    }

    else if(strcmp(nombre_dispositivo, dispositivo3) == 0) {
      
      Serial.printf("Valor del pulsador = %s\n", val.val.b? "true" : "false");

      if(strcmp(nombre_parametro, "Power") == 0) {
        ESTADO_BZ1 = val.val.b;
        ESTADO_BZ1 = !ESTADO_BZ1;
        control_Buzzer(1, BZ1, ESTADO_BZ1);
      }
    }
   
    //----------------------------------------------------------------------------------      
}
/****************************************************************************************************
*                                       Función setup                                               *
*****************************************************************************************************/
void setup(){
  //------------------------------------------------------------------------------
  uint32_t chipId = 0;
  Serial.begin(115200);
  // Configuración de GPIO
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BZ1, OUTPUT);
  pinMode(gpio_reset, INPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(fan, OUTPUT);
  sensorTH.begin();
  digitalWrite(WIFI_LED, LOW);
  //------------------------------------------------------------------------------
  // Estados iniciales GPIO
  digitalWrite(LED1, !ESTADO_LED1);
  digitalWrite(LED2, !ESTADO_LED2);
  digitalWrite(BZ1, !ESTADO_BZ1);
  //------------------------------------------------------------------------------
  Node DispLab1;    
  DispLab1 = RMaker.initNode("Monitoreo de ambiente y control de dispositivos");
  //------------------------------------------------------------------------------
  // Dispositivo interruptor estándar
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);
  //------------------------------------------------------------------------------
  // Añadir un dispositivo de conmutación al nodo   
  DispLab1.addDevice(my_switch1);
  DispLab1.addDevice(my_switch2);
  DispLab1.addDevice(my_switch3);
  //------------------------------------------------------------------------------
  // Añadir visualizadores de temperatura y humedad al nodo
  DispLab1.addDevice(temperature);
  DispLab1.addDevice(humidity);
  //------------------------------------------------------------------------------
  //Opcional
  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  //------------------------------------------------------------------------------
  //Nombre de servicio
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  Serial.printf("\nChip ID:  %d Nombre de Servicio: %s\n", chipId, SSID);
  //------------------------------------------------------------------------------
  Serial.printf("\nIniciando ESP-RainMaker\n");
  RMaker.start();
  //------------------------------------------------------------------------------
  //****************
  //Timer for Sending Sensor's Data
  //Timer.setInterval(5000);
  //*****************
  
  WiFi.onEvent(sysProvEvent);
  #if CONFIG_IDF_TARGET_ESP32
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, SSIDPass, SSID);
  #else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, SSIDPass, SSID);
  #endif
  //------------------------------------------------------------------------------
  digitalWrite(LED1, ESTADO_LED1);
  digitalWrite(LED2, ESTADO_LED2);
  digitalWrite(BZ1, ESTADO_BZ1);
  
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, ESTADO_LED1);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, ESTADO_LED2);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, ESTADO_BZ1);

  Serial.printf("El estado del LED 1 es %s \n", ESTADO_LED1? "ON" : "OFF");
  Serial.printf("El estado del LED 2 es %s \n", ESTADO_LED2? "ON" : "OFF");
  Serial.printf("El estado del Buzzer 1 es %s \n", ESTADO_BZ1? "ON" : "OFF");
  //------------------------------------------------------------------------------
}

/****************************************************************************************************
*                                        Función cíclica                                            *
*****************************************************************************************************/
void loop()
{
  //********************************
  //if (Timer.isReady() && WL_CONNECTED) {                    // Check is ready a second timer
    //Serial.println("Sending Sensor's Data");
    delay(3000);
    sensor();
   // Timer.reset();
 // }     
  //------------------------------------------------------------------------------
  // Leer GPIO0, pulsador externo para reiniciar
  if(digitalRead(gpio_reset) == LOW) { //Pulsador RESET presionado
    Serial.printf("Pulsador Reset activado!\n");
    // Manejo de antirrebote
    delay(100);
    int tiempoInicial = millis();
    while(digitalRead(gpio_reset) == LOW) delay(50);
    int tiempoFinal = millis();
    //_______________________________________________________________________
    if ((tiempoFinal - tiempoInicial) > 10000) {
      // Si está presionado RESET por más de 10 segundos, eliminar todo
      Serial.printf("Restauración de fábrica.\n");
      RMakerFactoryReset(2);
    } 
    //_______________________________________________________________________
    else if ((tiempoFinal - tiempoInicial) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      // Si está presionado RESET por más de 3 segundos pero menos de 10 segundos, reinicia Wi-Fi
      RMakerWiFiReset(2);
    }
    //_______________________________________________________________________
  
  //------------------------------------------------------------------------------
  delay(100);
  
  if (WiFi.status() != WL_CONNECTED){
    Serial.println("WiFi No Conectado");
    digitalWrite(WIFI_LED, LOW);
  }
  else{
    Serial.println("WiFi Conectado");
    digitalWrite(WIFI_LED, HIGH);
  }
  }
}
void sensor()
{
   float temperatura = sensorTH.readTemperature(); // Se declara la variable temperatura
   float humedad = sensorTH.readHumidity(); // Se declara la variable húmedad
   
   Serial.print("Temperatura : ");
   Serial.print(temperatura);
   Serial.println("°C");
   Serial.print("Humedad : ");
   Serial.print(humedad);
   Serial.println("%");

   //Actualización y reporte de temperatura y húmedad
   temperature.updateAndReportParam("Temperature", temperatura);
   humidity.updateAndReportParam("Temperature", humedad);
   delay(2000);
   //Condición para encendido de ventilador
   if (temperatura < 28){  
    digitalWrite(fan, LOW);
   }else if (temperatura >= 28){
    digitalWrite(fan, HIGH);
    }
}