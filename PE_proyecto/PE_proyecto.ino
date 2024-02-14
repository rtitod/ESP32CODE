#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//usar WPA2
const char* ssid = "MOVISTAR_AF09";
const char* password = "12345678";
const int AnalogPin = 34; //sensor humedad yl-69
const int AnalogPin2 = 16; //led blanco
const int AnalogPin3 = 35; //ldr
const int AnalogPin5 = 17;  //led azul
const int AnalogPin6 = 18;  //led verde
const int UmbralHumedadMinimo = 10;
const int UmbralHumedadMaximo = 70;
const char* bajahumedad = "Alerta: La humedad está bajo ";
const char* altahumedad = "Alerta: La humedad está por encima de ";
const char* unidad = " HR.";
const char* normalhumedad = "El nivel de humedad está dentro de los parametros normales.";
const int TapTime = 1;
const int LedTime = 1;
const int DelayTime = 3;
const int LdrDelayTime = 2;
const int WifiTimeout = 30;
const int fadeAmount = 5;  // Cantidad de cambio en el brillo por iteración
const int channel = 0;     // Canal PWM
const int channel2 = 1;     // Canal PWM
const int channel3 = 2;     // Canal PWM

const char* mqttServerEdge = "192.168.31.61";
const int mqttEdgePort = 1883;

const char* sensoredgetopic = "edge/sensor";
const char* llaveedgetopic = "edge/tap";
const char* ldredgetopic = "edge/ldr";
const char* lededgetopic = "edge/led";

bool nuevomensajellave = false;
bool nuevomensajeled = false;
bool nowifiloopexe = true;

int ledmultiplicador = 2;

int nivelled;
int led_write_value;
int tiempoencendido;

int nivelllave;
int llave_write_value;
int tiempoapertura;

WiFiUDP ntpUDP;
WiFiClientSecure espClient;
WiFiClient espClient_edge;
NTPClient timeClient(ntpUDP);
PubSubClient cliente_edge(espClient_edge);

void setup() {
  //Configurar la salida serial
  Serial.begin(9600);
  // Configurar conexión Wi-Fi
  WiFi.begin(ssid, password);
  //Configurar Led
  pinMode(AnalogPin2, OUTPUT); // Establecer el pin del LED como salida
  ledcSetup(channel, 5000, 8); // Canal, Frecuencia, Resolución
  ledcAttachPin(AnalogPin2, channel); // Asignar el pin al canal

  pinMode(AnalogPin5, OUTPUT); // Establecer el pin del LED como salida
  ledcSetup(channel2, 5000, 8); // Canal, Frecuencia, Resolución
  ledcAttachPin(AnalogPin5, channel2); // Asignar el pin al canal

  pinMode(AnalogPin6, OUTPUT); // Establecer el pin del LED como salida
  ledcSetup(channel3, 5000, 8); // Canal, Frecuencia, Resolución
  ledcAttachPin(AnalogPin6, channel3); // Asignar el pin al canal
  // Esperar hasta que se conecte a la red WiFi
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conectándose al Access Point WiFi...");
    WiFi.begin(ssid, password);
    delay(1000);
    // Si no se conecta en un tiempo determinado, encender las luces
    if (millis() - startAttemptTime > (WifiTimeout * 1000)) { // n segundos
      Serial.println("No se pudo conectar a la red WiFi. Se encenderan las luces de alarma.");
      while (WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password);
        intermitent_lights();
      }
      Serial.println("Conexión a la red WiFi Recuperada!");
    }
  }
  nowifiloopexe = false;
  //Confirmar conexion exitosa
  Serial.println("Conectado al Access Point WiFi");
  //Luces Intermitentes
  intermitent_lights();
  ledcWrite(channel3, 125); // Asignar el valor a la intensidad del LED a 125
  //Configurar NTP
  timeClient.begin();
  timeClient.setTimeOffset(-18000);

  // Configurar cliente MQTT
  cliente_edge.setServer(mqttServerEdge, mqttEdgePort);  

  // Configurar Callback
  cliente_edge.setCallback(callback);

  // Subscribe to Topic2
  cliente_edge.subscribe(llaveedgetopic);

  //Hilos
  xTaskCreatePinnedToCore(sensorThread, "SensorThread", 8192, NULL, 1, NULL, 0); // Pinned al núcleo 0 //publish
  //xTaskCreatePinnedToCore(ldrThread, "ldrThread", 8192, NULL, 1, NULL, 1); // Pinned al núcleo 1 //publish
  xTaskCreatePinnedToCore(llaveThread, "llaveThread", 8192, NULL, 1, NULL, 0); // Pinned al núcleo 0
  //xTaskCreatePinnedToCore(ledThread, "ledThread", 8192, NULL, 1, NULL, 1); // Pinned al núcleo 1
}

void intermitent_lights() {
  //LED de Confirmacion de conexion
  ledcWrite(channel, 255); // Asignar el valor a la intensidad del LED al maximo
  ledcWrite(channel2, 255); // Asignar el valor a la intensidad del LED al maximo
  ledcWrite(channel3, 255); // Asignar el valor a la intensidad del LED al maximo
  delay(1000);
  ledcWrite(channel, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel2, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel3, 0); // Asignar el valor a la intensidad del LED a 0
  delay(1000);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Artifact: Mensaje recibido en el tópico: ");
  Serial.println(topic);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print(F("Error al deserializar el JSON: "));
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, lededgetopic) == 0) {
    //recuperamos los datos nivelled y tiempoencendido del topico
    nivelled = doc["nivelled"];
    led_write_value = 255 * (nivelled/100.0);
    tiempoencendido = doc["tiempoencendido"];
    nuevomensajeled = true;
  } 
  else if (strcmp(topic, llaveedgetopic) == 0) {
    //recuperamos los datos nivellave y tiempoapertura del topico
    nivelllave = doc["nivelllave"];
    llave_write_value = 255 * (nivelllave/100.0);
    tiempoapertura = doc["tiempoapertura"];
    nuevomensajellave = true;
  }
  else {
    Serial.println("Mensaje para un tópico no reconocido");
  }
}

void sensorThread(void *parameter) {
  int humedad_relativa;
  int valor_humedad_raw;
  while (1) {
    //Obtener el valor del sensor
    Serial.println("Sensor: leyendo sensor de Humedad...");
    valor_humedad_raw = analogRead (AnalogPin);
    humedad_relativa =  ((4095.0 - valor_humedad_raw) / 4095.0 ) * 100.0;
    Serial.println("Sensor: la humedad relativa obtenida es :" + String(humedad_relativa));
    // Crear el JSON
    DynamicJsonDocument jsonDoc(200);
    jsonDoc["Fecha"] = obtenerFechaActualizada(timeClient);
    jsonDoc["Hora"] = obtenerHoraActualizada(timeClient);
    jsonDoc["type"] = "mqtt";
    jsonDoc["medida"] = humedad_relativa;
    if (humedad_relativa < UmbralHumedadMinimo) {
      jsonDoc["comentario"] = bajahumedad + String(UmbralHumedadMinimo) + unidad;
    }else if(humedad_relativa > UmbralHumedadMaximo){
      jsonDoc["comentario"] = altahumedad + String(UmbralHumedadMaximo) + unidad;
    }
    else{
      jsonDoc["comentario"] = String(normalhumedad) +  unidad;
    }
    //Convertir array JSON a String
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    //Enviar el JSON
    if (cliente_edge.publish(sensoredgetopic, jsonString.c_str())) {
      Serial.println("Mqtt enviado correctamente.");
    } else {
      Serial.println("Error al enviar el mensaje mqtt.");
    }
    // Esperar antes de la siguiente lectura
    delay(DelayTime * 1000); 
  }
}

// Función para el hilo del ldr1
/**
void ldrThread(void *parameter) {
  int photocellReading;

  while (1) {
    Serial.println("Sensor: leyendo sensor LDR 1...");
    photocellReading = analogRead(AnalogPin3);
    photocellReading =  (photocellReading / 4095.0 ) * 255.0;
    printLightStatus("Sensor: Ldr", photocellReading);
    // Crear el JSON
    DynamicJsonDocument jsonDoc(200);
    jsonDoc["Fecha"] = obtenerFechaActualizada(timeClient);
    jsonDoc["Hora"] = obtenerHoraActualizada(timeClient);
    jsonDoc["type"] = "mqtt";
    jsonDoc["medida"] = photocellReading;
    jsonDoc["comentario"] = "LDR";
    //Convertir array JSON a String
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    //Enviar el JSON
    if (cliente_edge.publish(ldredgetopic, jsonString.c_str())) {
      Serial.println("JSON enviado correctamente.");
    } else {
      Serial.println("Error al enviar el JSON.");
    }
    // Esperar antes de la siguiente lectura
    delay(LdrDelayTime * 1000);
  }
}
**/

void llaveThread(void *parameter) {
  while (1) {
    if (nuevomensajellave){
      Serial.println("Llave: abriendo la llave " + String(tiempoapertura) + " segundos a una presión de " + String(nivelllave) + " ...");
      ledcWrite(channel2, llave_write_value); // Asignar el valor a la intensidad del LED
      nuevomensajellave = false;
    }
    delay(TapTime * 1000); // Esperar antes del siguiente loop
  }
}

/**
void ledThread(void *parameter) {
  while (1) {
    if (nuevomensajeled){
      Serial.println("LED: encendiendo el LED " + String(tiempoencendido) + " segundos a una intensidad de " + String(nivelled) + " ...");
      ledcWrite(channel, led_write_value); // Asignar el valor a la intensidad del LED
      nuevomensajeled = false;
    }
    delay(LedTime * 1000); // Esperar antes del siguiente loop
  }
}
**/

void printLightStatus (const String &ldrName, int value) {
  String status = (value <= 51) ? "DARK" :
    (value <= 127) ? "DIM LIGHT" :
    (value <= 203) ? "BRIGHT LIGHT" :
    "FULL DAY LIGHT";
  Serial.println(ldrName + ": " + status + " : El valor es = " + String(value));
}

String obtenerFechaActualizada(NTPClient& timeClient) {
  // Obtener el timestamp actualizado
  timeClient.update();
  // Obtener EpochTime
  unsigned long epochTime = timeClient.getEpochTime();
  // Obtener formattedTime
  String formattedTime = timeClient.getFormattedTime();
  // Obtener toda la fecha y Hora
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int currentYear = ptm->tm_year + 1900;
  int currentMonth = ptm->tm_mon + 1;
  int monthDay = ptm->tm_mday;
  // Formatear día y mes con ceros a la izquierda si es necesario
  String formattedDay = (monthDay < 10 ? "0" : "") + String(monthDay);
  String formattedMonth = (currentMonth < 10 ? "0" : "") + String(currentMonth);
  // Crear la cadena de fecha
  String currentDate = formattedDay + "/" + formattedMonth + "/" + String(currentYear);
  return currentDate;
}

String obtenerHoraActualizada(NTPClient& timeClient) {
  // Obtener el timestamp actualizado
  timeClient.update();
  // Obtener formattedTime
  String formattedTime = timeClient.getFormattedTime();
  return formattedTime;
}

void reconnectWiFi() {
  Serial.println("Perdida de conexión WiFi. Intentando reconectar...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("Conectándose al Access Point WiFi...");
    delay(1000);
    // Si no se conecta en un tiempo determinado, mostrar alerta
    if (millis() - startAttemptTime > (WifiTimeout * 1000)) { // n segundos
      Serial.println("No se pudo conectar a la red WiFi. Se encenderan las luces de alarma.");
      while (WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password);
        intermitent_lights();
      }
      Serial.println("Conexión a la red WiFi Recuperada!");
    }
  }
  Serial.println("Reconexión exitosa al WiFi");
  ledcWrite(channel, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel2, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel3, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel3, 125); // Asignar el valor a la intensidad del LED a 125
}

void loop() {
   //Obtener el timestamp actualizado
  if (!WiFi.isConnected() && !nowifiloopexe) {
    reconnectWiFi();
  }

  if (!cliente_edge.connected()) {
    reconnect_edge();
  }
  cliente_edge.loop();
}

void reconnect_edge() {
  while (!cliente_edge.connected()) {
    if (cliente_edge.connect("SensorLab")) {
      Serial.println("Conectado al servidor MQTT Edge");
      cliente_edge.subscribe(llaveedgetopic);
    } else {
      Serial.println("Error de conexión al servidor MQTT Edge, reintentando...");
      delay(2000);
    }
  }
}
