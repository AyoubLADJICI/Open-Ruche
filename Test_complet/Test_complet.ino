#include <MKRWAN.h>
#include <Wire.h>
#include "uFire_SHT20.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HX711.h"
#include <Digital_Light_TSL2561.h>

//Configuration du module LoRa
LoRaModem modem;

//Initialisation du capteur SHT20
uFire_SHT20 sht20; 

// Initialisation du capteur DS18B20
#define ONE_WIRE_BUS 0  // Broche DATA du DS18B20 (D0 sur le MKR WAN 1310)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// Définition du capteur DHT22
#define DHTPIN 2      // Broche de connexion du capteur
#define DHTTYPE DHT22 // Type du capteur DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

// Initialisation du capteur HX711 (Balance)
#define LOADCELL_DOUT_PIN 1
#define LOADCELL_SCK_PIN 6
HX711 scale;
float calibration_factor = -8950; // Ajuster ce facteur après calibration

#define VBAT_PIN A0  // Broche utilisée pour mesurer la tension de la batterie

// Facteur de conversion basé sur le diviseur de tension 100kΩ / 47kΩ
#define DIVISEUR_RATIO 3.13

// Plage de tension de la batterie (basée sur une Li-ion 3.7V)
#define VBAT_MAX 4.5  // Tension pleine charge
#define VBAT_MIN 3.2  // Tension déchargée (batterie critique)

//Clés LoRa pour OTAA
String AppEUI = "213D57ED00000000";
String AppKEY = "8EED2DFE0FA94091FC093C1EBBF382C8";

bool connected;
int err_count;
short con;

int readBatteryLevel() {
    int raw = analogRead(VBAT_PIN);
    float vA0 = (raw / 4095.0) * 3.3;  
    float vBat = vA0 * DIVISEUR_RATIO;
    if (vBat > 4.5 || vBat < 2.5) {  
        Serial.println("Erreur : Valeur anormale détectée !");
        Serial.println("Vérifiez la connexion du diviseur de tension.");
    }
    int batteryLevel = 0;
    if (vBat >= VBAT_MAX) {
        batteryLevel = 100;
    } else if (vBat <= VBAT_MIN) {
        batteryLevel = 0;
    } else {
        batteryLevel = (int)((vBat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0);
    }
    Serial.println("--------------------------------------");
    Serial.print("Tension mesurée sur A0 : ");
    Serial.print(vA0, 3);
    Serial.println(" V");
    Serial.print("Niveau de charge de la batterie : ");
    Serial.print(batteryLevel);
    Serial.println(" %");
    
    if (batteryLevel == 100) {
        Serial.println("Batterie pleine.");
    } else if (batteryLevel >= 80) {
        Serial.println("Batterie bien chargée.");
    } else if (batteryLevel >= 60) {
        Serial.println("Batterie moyenne, recharge bientôt.");
    } else if (batteryLevel >= 40) {
        Serial.println("Batterie faible.");
    } else if (batteryLevel >= 20) {
        Serial.println("Batterie très faible, recharge nécessaire.");
    } else {
        Serial.println("Batterie critique, recharge immédiate.");
    }
    return batteryLevel;
}

void printValue(const char* label, float value, const char* unit) {
    if (isnan(value)) {
        Serial.print("⚠️ Erreur de lecture : ");
        Serial.println(label);
    } else {
        Serial.print(label);
        Serial.print(" : ");
        Serial.print(value, 2);
        Serial.print(" ");
        Serial.println(unit);
    }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Welcome to MKR WAN 1300/1310 ");
  modem.begin(EU868);
  delay(1000);      // apparently the murata dislike if this tempo is removed...
  connected=false;
  err_count=0;
  con =0;
  
  // Initialisation des capteurs
  Wire.begin();
  sht20.begin();
  ds18b20.begin();
  dht.begin();
  TSL2561.init();
  analogReadResolution(12);
  
  // Initialisation de la balance
  //scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  //scale.tare(); // Remise à zéro
  //scale.set_scale(calibration_factor);
}

void loop() {
   if ( !connected ) {
    Serial.print("Join test : ");
    Serial.println(++con);
    int ret=modem.joinOTAA(AppEUI, AppKEY);
    if ( ret ) {
      connected=true;
      modem.minPollInterval(60);
      Serial.println("Connected");
      modem.dataRate(5);   // switch to SF7
      delay(100);          // because ... more stable
      err_count=0;
    }
  }

  if ( connected ) {
    Serial.println("📡 Lecture des capteurs..");
    
    // 🔹 Mesure Température et Humidité du SHT20
    sht20.measure_all();  // Mesure température et humidité
    float temp_sht20 = sht20.tempC;
    float hum_sht20 = sht20.RH;

    if (isnan(temp_sht20) || isnan(hum_sht20)) {
      Serial.println("❌ Erreur de lecture du capteur SHT20 !");
    }

    // 🔹 Mesure Température DS18B20
    ds18b20.requestTemperatures();
    float temp_ds18b20 = ds18b20.getTempCByIndex(0);

    if (temp_ds18b20 == DEVICE_DISCONNECTED_C) {
      Serial.println("❌ Erreur : Capteur DS18B20 non détecté !");
    }

    // Mesure Température et Humidité du DHT22
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    float temp_dht22 = event.temperature;
    dht.humidity().getEvent(&event);
    float hum_dht22 = event.relative_humidity;

    if (isnan(temp_dht22) || isnan(hum_dht22)) {
        Serial.println("⚠️ Erreur de lecture du capteur DHT22 !");
    }

    // 🔹 Mesure du Poids (Balance HX711)
    /*float poids = 0.0;
    for (int i = 0; i < 5; i++) {
      poids += scale.get_units() / 2.2046;  // Conversion lbs → kg
      delay(200);
    }
    poids /= 10; // Moyenne sur 10 mesures
    if (poids < 0) poids = 0;  // Correction si la balance affiche un poids négatif
*/
    // 🔹 Lecture du capteur de lumière TSL2561
    int lux = TSL2561.readVisibleLux();

    int batteryLevel = readBatteryLevel();

    // Convertir en short (multiplié par 100 pour garder 2 décimales)
    short temp_sht20_int = (short)(temp_sht20 * 100);
    short hum_sht20_int = (short)(hum_sht20 * 100);
    short temp_ds18b20_int = (short)(temp_ds18b20 * 100);
    short temp_dht22_int = (short)(temp_dht22 * 100);
    short hum_dht22_int = (short)(hum_dht22 * 100);
    //short poids_int = (short)(poids * 100);
    //debug
    float poids = 30.0;
    short poids_int = (short) (poids * 100);
    short lux_int = (short)(lux * 100);  
    short batterie_int = (short)(batteryLevel * 100);
    
    // 📊 Affichage des valeurs
    printValue("🌡 Température SHT20", temp_sht20, "°C");
    printValue("💧 Humidité SHT20", hum_sht20, "%");
    printValue("🌡 Température DS18B20", temp_ds18b20, "°C");
    printValue("🌡 Température DHT22", temp_dht22, "°C");
    printValue("💧 Humidité DHT22", hum_dht22, "%");
    printValue("⚖ Poids", poids, "kg");
    printValue("🌞 Luminosité", lux, "lux");
    printValue(" Batterie", batteryLevel, "%");
    

    Serial.println("📡 Envoi du message LoRa...");
    int err = 0;
    modem.beginPacket();
    modem.write((uint8_t*)&temp_sht20_int, sizeof(temp_sht20_int)); // Envoi de la température
    modem.write((uint8_t*)&hum_sht20_int, sizeof(hum_sht20_int));   // Envoi de l'humidité
    modem.write((uint8_t*)&temp_ds18b20_int, sizeof(temp_ds18b20_int));  // Température DS18B20
    modem.write((uint8_t*)&temp_dht22_int, sizeof(temp_dht22_int));  // Température DHT22
    modem.write((uint8_t*)&hum_dht22_int, sizeof(hum_dht22_int));  // Humidité DHT22
    modem.write((uint8_t*)&poids_int, sizeof(poids_int)); // Ajout de la mesure de poids
    modem.write((uint8_t*)&lux_int, sizeof(lux_int));
    modem.write((uint8_t*)&batterie_int, sizeof(batterie_int));
    err = modem.endPacket();

    if ( err <= 0 ) {
      Serial.print("Error : ");
      Serial.println(err);
      // Confirmation not received - jam or coverage fault
      err_count++;
      if ( err_count > 50 ) {
        Serial.println("Trop d'échecs, reconnexion...");
        connected = false;
        err_count = 0;
      }
      // wait for 2min for duty cycle with SF12 - 1.5s frame
      for ( int i = 0 ; i < 120 ; i++ ) {
        delay(1000);
      }
    } else {
      err_count = 0;
      // wait for 20s for duty cycle with SF7 - 55ms frame
      delay(15000);
      Serial.println("Message envoyé");   
    }
  }
}