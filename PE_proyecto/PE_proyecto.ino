#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//usar WPA2
const char* ssid = "miwifi";
const char* password = "mipassword";
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
const int TapTime = 2;
const int LedTime = 2;
const int DelayTime = 15;
const int LdrDelayTime = 2;
const int WifiTimeout = 30000;
const int fadeAmount = 5;  // Cantidad de cambio en el brillo por iteración
const int channel = 0;     // Canal PWM
const int channel2 = 1;     // Canal PWM
const int channel3 = 2;     // Canal PWM

const char* mqttServer = "a85vkpsrzp7kv-ats.iot.us-west-2.amazonaws.com";
const char* mqttServerEdge = "10.0.0.190";

const int mqttPort = 8883;
const int mqttEdgePort = 1883;

bool ejecutarllave = false;
bool ejecutarled = false;

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
PubSubClient client(espClient);
PubSubClient cliente_edge(espClient_edge);

//AmazonRootCA1.pem
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

//device-certificate.pem.crt
const char* client_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAPflllb2yddGFt1phCj97zU8iTERMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzExMjYwNzU4
NDRaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCbXUUkT344OkMnsgum
WDcNXudcQxUEoe+X5ECWRxTSgCWzheBbLAnAN0BUy2ItAoH4xmozuZ4aoKQlwciS
4uJltcjfhEKyFbCTFd3dUjNdr/eb+vK70P/P1kO1Xr8W+zC5jl+m/reozofFRIht
v2QIUL96s5U+D3YmE8qVbY6rIp67yVEmQGUfSOeobFIp68WA376tOxZ9JZneZfP8
j6jLEfAUfTkA/cvZ3y794N0CxIOOkfLA11ERpZRPQxHtObgIIghrW36l/IKoviwD
WSV40fGCX9F7tuafGzFW4gjj5SRe+h2V/D+W/GtGossvFL48gd3SHB9fdyuKlc6w
G+ahAgMBAAGjYDBeMB8GA1UdIwQYMBaAFDmzFF0BKuoFDJ8CwGaI5Qh3y6EJMB0G
A1UdDgQWBBRINt2WvecJ2g/M/rrnCg4KFWI63zAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAsJgJh3F8iVO75WgGBF4/Sffs
dWlIRVlenyqpmETkFkkhuQaeN+/Z/oC/wc+106e+bZsFSDkd+J87IeR8DsGJd5Ui
ZrsXuuB84/e7pRkjYGfCjie3cXX+O0H+gatOf5kvQccPz3cm85sUMK7MUKiLq4+m
SVAz5Vc9qZCTwtiZgkszQM1+FYBaDeSKkTdsjfz93O7NWKWxlfva4hUZtYBlSlTX
qzE/7rt3ViIiXjARNvBKyo8HRnUB6TAvF+pvEesmhs7E3i9yVA3B4Zx7dR+B8Dof
2tiZPkJBp0LbG9p9miIMq9DNdvIK3ga7dS7Fcdst88MxPjHhI+DECurVjhozGw==
-----END CERTIFICATE-----
)EOF";

//private.pem.key
const char* client_key = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAm11FJE9+ODpDJ7ILplg3DV7nXEMVBKHvl+RAlkcU0oAls4Xg
WywJwDdAVMtiLQKB+MZqM7meGqCkJcHIkuLiZbXI34RCshWwkxXd3VIzXa/3m/ry
u9D/z9ZDtV6/FvswuY5fpv63qM6HxUSIbb9kCFC/erOVPg92JhPKlW2OqyKeu8lR
JkBlH0jnqGxSKevFgN++rTsWfSWZ3mXz/I+oyxHwFH05AP3L2d8u/eDdAsSDjpHy
wNdREaWUT0MR7Tm4CCIIa1t+pfyCqL4sA1kleNHxgl/Re7bmnxsxVuII4+UkXvod
lfw/lvxrRqLLLxS+PIHd0hwfX3cripXOsBvmoQIDAQABAoIBAAHGjhy9H9cqq0vP
2Bw9k9t+Qyylw5667w8QV8Cf7Vf+iO/FTuwb07yzpPOTI3saNTwPoHUY4Pqy7U1V
KqKrHyVEb5hGXbDFWLBFprR51QZ0FWNd+do1qEbKzxlKjOqJUmQ8gVFaReUS/zA8
+UYv0sfZjGatmL4N1S7YOQ9+IgxIo3IwCVvO3jP9E8afGnd7KFJm0vpnXRgHcHZf
UvTtcHLw5KXwtK9cB2a1jQje/TuDC3WU9VrVA169LlooGhYCFkIlYEdKtZ7V1acM
M81md8vHqJVthb/s+zlUvBYQDaP6VZIc6iXSMpNXFA+O2VP5Pz7yJpR66GIus3oH
hN4gj/UCgYEAzfHhs32VDek/SAG45d+XtPb50H/H7WLqqNLx846frA64xzZvDH9i
buCegKYIORkZp4sdBIGRA3xsBEd6MaWh881WKTfJSU7lcS+jdOspdlkibruSmi5+
wKUAYAbn/z/7ljY7yxaN8s8ZAosgijpW27ieTN1X51BbwPe+JeHUiV8CgYEAwSA2
YeYdJxFez0NIEjM3FHLyyd64tycJNtWGAQbscNBlmr12467SIZeEcVxIBzO9quxq
A9n6stZRdOnk0eMMObm5nKqEkRhjHtfUW6F+On0uMLe6A+FOUECHBazlSjpO4Q+g
f1zKygia0BO4oxwxVaHUsIbir1AEogkaV+w1j/8CgYBwCWe04tvk3oBbpvw0qif4
/ec3o1xba1WnlGKmEnARBU1GqvlMvjwJf88aw7hzf5EyUX7uxaHjWJvW1B/IxIfP
f2ShNcUeSXqO0dDFuW0sAh+2R/rgP+0a1mjW1Mr/VIqE1GpA+LsuXbYIaTHBoS1B
iTUtMbkxkMjpuJ1MNIgQVwKBgQCwa5jCPdALzFZrx+6ikZ6TU55uwsLCrSpBqTTM
42JCfF0Uylc9DopunVcjp0U1LxLofScrdYkKrbWjs7aP9XVJllv0veB0G28Q0eDw
A8qn64qyfmOy/7LmpMuzZ4neEP3VoszfDAlEHjAuPJXCixKWQ0tkJxQVSmexDde+
0czsSQKBgQC44zNv/vyrRiO6B1aFLda0ktoiSjtPs3fetyoZx8pUWpGZlHwCmzmH
w4IF9cG1Uw1rQuETqaTcaFePyJHyh3hgsUgWOOHIu97Vbq6wZA/EZPrapxxi0LYY
suYI4Dfq0/LoDnlUvU+3q5uijJVxmrkSPkfcZAkJJfMj5lbO2ebbgw==
-----END RSA PRIVATE KEY-----
)EOF";

void setup() {
  //Configurar la salida serial
  Serial.begin(9600);
  // Configurar conexión Wi-Fi
  WiFi.begin(ssid, password);

  // Esperar hasta que se conecte a la red WiFi
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectándose al Access Point WiFi...");
    
    // Si no se conecta en un tiempo determinado, detener el intento
    if (millis() - startAttemptTime > (WifiTimeout * 1000)) { // n segundos
      Serial.println("No se pudo conectar a la red WiFi. Reinicie y verifique la configuración.");
      while (1); // Bucle infinito
    }
  }
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

  //Confirmar conexion exitosa
  Serial.println("Conectado al Access Point WiFi");
  //LED de Confirmacion de conexion
  ledcWrite(channel, 255); // Asignar el valor a la intensidad del LED al maximo
  ledcWrite(channel2, 255); // Asignar el valor a la intensidad del LED al maximo
  ledcWrite(channel3, 255); // Asignar el valor a la intensidad del LED al maximo
  delay(1000);
  ledcWrite(channel, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel2, 0); // Asignar el valor a la intensidad del LED a 0
  ledcWrite(channel3, 125); // Asignar el valor a la intensidad del LED a 0

  //Configurar NTP
  timeClient.begin();
  timeClient.setTimeOffset(-18000);

  // Cargar certificados
  espClient.setCACert(ca_cert);
  espClient.setCertificate(client_cert);
  espClient.setPrivateKey(client_key);

  // Configurar cliente MQTT
  client.setServer(mqttServer, mqttPort);
  cliente_edge.setServer(mqttServerEdge, mqttEdgePort);  

  // Configurar Callback
  client.setCallback(callback);

  // Subscribe to Topic2
  client.subscribe("artifact/tap");
  client.subscribe("artifact/led");

  //Hilos
  xTaskCreatePinnedToCore(sensorThread, "SensorThread", 8192, NULL, 1, NULL, 0); // Pinned al núcleo 0 //publish
  //xTaskCreatePinnedToCore(ldrThread, "ldrThread", 8192, NULL, 1, NULL, 1); // Pinned al núcleo 1 //publish
  xTaskCreatePinnedToCore(llaveThread, "llaveThread", 8192, NULL, 1, NULL, 0); // Pinned al núcleo 0
  //xTaskCreatePinnedToCore(ledThread, "ledThread", 8192, NULL, 1, NULL, 1); // Pinned al núcleo 1
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

  if (strcmp(topic, "artifact/led") == 0) {
    if (!ejecutarled){
      nivelled = doc["nivelled"];
      led_write_value = 255 * (nivelled/100.0);
      tiempoencendido = doc["tiempoencendido"];
      ejecutarled = true;
    }
  } 
  else if (strcmp(topic, "artifact/tap") == 0) {
    if (!ejecutarllave){
      nivelllave = doc["nivelllave"];
      llave_write_value = 255 * (nivelllave/100.0);
      tiempoapertura = doc["tiempoapertura"];
      ejecutarllave = true;
    }
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
    if (cliente_edge.publish("edge", jsonString.c_str())) {
      Serial.println("Mqtt enviado correctamente.");
    } else {
      Serial.println("Error al enviar el mensaje mqtt.");
    }
    // Esperar antes de la siguiente lectura
    delay(DelayTime * 1000); 
  }
}

// Función para el hilo del ldr1
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
    if (client.publish("sensor/ldr", jsonString.c_str())) {
      Serial.println("JSON enviado correctamente.");
    } else {
      Serial.println("Error al enviar el JSON.");
    }
    // Esperar antes de la siguiente lectura
    delay(LdrDelayTime * 1000);
  }
}


void llaveThread(void *parameter) {
  while (1) {
    if (ejecutarllave){
      Serial.println("Llave: abriendo la llave " + String(tiempoapertura) + " segundos a una presión de " + String(nivelllave) + " ...");
      ledcWrite(channel2, llave_write_value); // Asignar el valor a la intensidad del LED
      //esperar tiempo de apertura
      delay(tiempoapertura * 1000);
      //cerrar la llave
      Serial.println("Llave: cerrando la llave ...");
      ledcWrite(channel2, 0); // Asignar el valor a la intensidad del LED
      ejecutarllave = false;
    }
    delay(TapTime * 1000); // Esperar antes del siguiente loop
  }
}

void ledThread(void *parameter) {
  while (1) {
    if (ejecutarled){
      Serial.println("LED: encendiendo el LED " + String(tiempoencendido) + " segundos a una intensidad de " + String(nivelled) + " ...");
      ledcWrite(channel, led_write_value); // Asignar el valor a la intensidad del LED
      //esperar tiempo de apertura
      delay(tiempoencendido * 1000);
      //cerrar la llave
      Serial.println("LED: apagando el LED ...");
      ledcWrite(channel, 0); // Asignar el valor a la intensidad del LED
      ejecutarled = false;
    }
    delay(LedTime * 1000); // Esperar antes del siguiente loop
  }
}

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

void loop() {
  // Obtener el timestamp actualizado
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (!cliente_edge.connected()) {
    reconnect_edge();
  }
  cliente_edge.loop();
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("SensorLab")) {
      Serial.println("Conectado al servidor MQTT");
      client.subscribe("artifact/tap");
      //client.subscribe("artifact/led");
    } else {
      Serial.println("Error de conexión al servidor MQTT, reintentando...");
      delay(2000);
    }
  }
}

void reconnect_edge() {
  while (!cliente_edge.connected()) {
    if (cliente_edge.connect("SensorLab")) {
      Serial.println("Conectado al servidor MQTT Edge");
    } else {
      Serial.println("Error de conexión al servidor MQTT Edge, reintentando...");
      delay(2000);
    }
  }
}
