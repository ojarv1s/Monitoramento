#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>


// Definições dos pinos
#define ACS712_PIN 14  // Pino de leitura do ACS712
#define DHT_PIN 26     // Pino de leitura do DHT22
#define SW420_PIN 32   // Pino de leitura do SW420 (sensor de vibração)
#define RELAY_PIN 5    // Pino do relé


// Credenciais da rede Wi-Fi e Adafruit IO
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASS "your_wifi_pass"
#define AIO_SERVER "io.adafruit.com"
#define AIO_PORT 1883
#define AIO_USERNAME "your_username"
#define AIO_KEY "your_adafruit_io"


#define DHTTYPE DHT22


float temperature_Val = 0.0;
float humidity_Val = 0.0;
float current_Val = 0.0;


DHT dht22(DHT_PIN, DHTTYPE);

// Declara o objeto "WiFiClient" como "client".
WiFiClient client;

// Configura a classe do cliente MQTT passando o cliente WiFi e o servidor MQTT e os detalhes de login. 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish current = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current");
Adafruit_MQTT_Publish vibration = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/vibration");
Adafruit_MQTT_Subscribe relay = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay");


void MQTT_Connect() {
  int8_t ret;
  
  // Para se já estiver conectado.
  if (mqtt.connected()) {
    return;
  }

  Serial.println();
  Serial.println("-------------MQTT_Connect()");
  Serial.println("Connecting to MQTT...");
  

  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 10 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  
  Serial.println("MQTT Connected.");
  Serial.println("-------------");
}


void read_DHT22() {
  Serial.println();
  Serial.println("-------------read_dht22()");
  
  // Lê umidade.
  humidity_Val = dht22.readHumidity();
  // Lê temperatura em Celcius.
  temperature_Val = dht22.readTemperature();

  // Checa se alguma leitura falhou.
  if (isnan(temperature_Val) || isnan(humidity_Val)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    Serial.println();
    humidity_Val = 0;
    temperature_Val = 0.00;
  }

  Serial.print(F("Humidity: "));
  Serial.print(humidity_Val);
  Serial.print(F("% ||  Temperature: "));
  Serial.print(temperature_Val);
  Serial.println(F("°C"));
  Serial.println("-------------");
}


void read_ACS712() {
  int rawValue = analogRead(ACS712_PIN);  // Leitura bruta do pino analógico
  float voltage = (rawValue / 1024.0) * 5.0;  // Converte a leitura para uma tensão
  float current = (voltage - 2.5) / 0.185;  // Converte a tensão para corrente (sensibilidade de 185 mV por A)

  current_Val = abs(current);

  Serial.print("Current: ");
  Serial.print(current_Val);
  Serial.println(" A");
}


void read_SW420() {
  int vibrationIntensity = analogRead(SW420_PIN);  // Lê a intensidade da vibração

  Serial.print("Vibration Intensity: ");
  Serial.println(vibrationIntensity);
}



void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(SW420_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  delay(500);


  Serial.println();
  Serial.println("-------------Set Wifi to STA mode");
  Serial.println("WIFI mode : STA");
  WiFi.mode(WIFI_STA);
  Serial.println("-------------");
  delay(500);


  Serial.println();
  Serial.println("-------------Connect to WiFi");
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(true);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int connecting_process_timed_out = 20;
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
    delay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      Serial.println();
      Serial.println("Failed to connect to WiFi. The ESP32 will be restarted.");
      Serial.println("-------------");
      delay(1000);
      ESP.restart();
    }
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("Successfully connected to : ");
  Serial.println(WIFI_SSID);
  Serial.println("-------------");

  delay(500);


  dht22.begin();
  
  mqtt.subscribe(&relay);
}

#define VIBRATION_THRESHOLD 800  // Limite de vibração para desativar o relé
#define CURRENT_THRESHOLD 14.0   // Limite de corrente (em amperes) para desativar o relé

void loop() {
  MQTT_Connect();

  // Recupera informação do botão Liga/Desliga do Adafruit IO (Server) para controlar o relay.
  Adafruit_MQTT_Subscribe *subscription;
  
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &relay) {
      Serial.println();
      Serial.print(F("Toggle Button Light Bulb 1 : "));
      Serial.println((char *)relay.lastread);
      
      if (!strcmp((char*) relay.lastread, "On")) {
        digitalWrite(RELAY_PIN, HIGH);
      } else if (!strcmp((char*) relay.lastread, "Off")) {
        digitalWrite(RELAY_PIN, LOW);
      }
    }
  }


  int vibrationIntensity = analogRead(SW420_PIN);
  if (!vibration.publish((int32_t)vibrationIntensity)) {
    Serial.println(F("Failed to send vibration intensity!"));
  } else {
    Serial.println(F("Vibration intensity sent successfully."));
  }

  read_DHT22();
  read_ACS712();
  read_SW420();


  // Verificação dos limites
  if (vibrationIntensity > VIBRATION_THRESHOLD || current_Val > CURRENT_THRESHOLD) {
    digitalWrite(RELAY_PIN, LOW);  // Desativa o relé
    Serial.println("Relé desativado devido a condições críticas!");
  } else {
    digitalWrite(RELAY_PIN, HIGH); // Mantém o relé ativado
    Serial.println("Relé ativado, condições normais.");
  }


  // Envia temperatura, umidade e corrente para Adafruit IO (Server).
  Serial.println();
  Serial.println("Sending temperature value...");
  
  if (!temperature.publish(temperature_Val)) {
    Serial.println(F("Failed to send temperature value !"));
  } else {
    Serial.println(F("Sending temperature value successfully."));
  }

  Serial.println();
  Serial.println("Sending humidity value...");
  
  if (!humidity.publish(humidity_Val)) {
    Serial.println(F("Failed to send humidity value !"));
  } else {
    Serial.println(F("Sending humidity value successfully."));
  }

  Serial.println();
  Serial.println("Sending current value...");
  if (!current.publish(current_Val)) {
    Serial.println(F("Failed to send current value!"));
  } else {
    Serial.println(F("Sending current value successfully."));
  }

  delay(5000);
}
