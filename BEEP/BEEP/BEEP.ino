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

//Clés LoRa pour OTAA
String AppEUI = "213D57ED00000000";
String AppKEY = "8EED2DFE0FA94091FC093C1EBBF382C8";

bool connected;
int err_count;
short con;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("🚀 Initialisation du MKR WAN 1310 et des capteurs...");

    if (!modem.begin(EU868)) {
        Serial.println("❌ Échec d'initialisation du modem LoRa !");
        while (1);
    }

    Serial.println("📡 Connexion OTAA en cours...");
    if (!modem.joinOTAA(AppEUI, AppKEY)) {
        Serial.println("❌ Connexion OTAA échouée !");
        while (1);
    }
    Serial.println("✅ Connecté à LoRaWAN !");

    modem.minPollInterval(60);
    modem.dataRate(5);
    delay(100);

    // Initialisation des capteurs
    Wire.begin();
    sht20.begin();
    ds18b20.begin();
    dht.begin();
    TSL2561.init();

    // Initialisation de la balance HX711
    //scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    //scale.tare();
    //scale.set_scale(calibration_factor);
}

void loop() {
    if (!connected) {
        Serial.println("🔄 Tentative de reconnexion OTAA...");
        if (modem.joinOTAA(AppEUI, AppKEY)) {
            connected = true;
            Serial.println("✅ Reconnecté !");
        } else {
            Serial.println("❌ Reconnexion échouée !");
            return;
        }
    }

    Serial.println("📡 Lecture des capteurs...");

    // 🔹 Température et Humidité (SHT20)
    sht20.measure_all();
    float temp_sht20 = sht20.tempC;
    float hum_sht20 = sht20.RH;

    // 🔹 Température DS18B20
    ds18b20.requestTemperatures();
    float temp_ds18b20 = ds18b20.getTempCByIndex(0);

    // 🔹 Température et Humidité DHT22
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    float temp_dht22 = event.temperature;
    dht.humidity().getEvent(&event);
    float hum_dht22 = event.relative_humidity;

    // 🔹 Mesure du Poids (HX711)
    float poids = 0.0;
    /*
    for (int i = 0; i < 5; i++) {
        poids += scale.get_units() / 2.2046; // lbs → kg
        delay(200);
    }
    poids /= 5;
    if (poids < 0) poids = 0;
    */
    // 🔹 Luminosité (TSL2561)
    int lux = TSL2561.readVisibleLux();

    // Vérification des valeurs NAN
    if (isnan(temp_sht20) || isnan(hum_sht20)) Serial.println("⚠️ Erreur SHT20 !");
    if (temp_ds18b20 == DEVICE_DISCONNECTED_C) Serial.println("⚠️ Erreur DS18B20 !");
    if (isnan(temp_dht22) || isnan(hum_dht22)) Serial.println("⚠️ Erreur DHT22 !");
    if (lux == -1) Serial.println("⚠️ Erreur TSL2561 !");

    // 🔄 Conversion en short (x100 pour garder 2 décimales)
    short temp_sht20_int = (short)(temp_sht20 * 100);
    short hum_sht20_int = (short)(hum_sht20 * 100);
    short temp_ds18b20_int = (short)(temp_ds18b20 * 100);
    short temp_dht22_int = (short)(temp_dht22 * 100);
    short hum_dht22_int = (short)(hum_dht22 * 100);
    short poids_int = (short)(poids * 100);
    short lux_int = (short)(lux * 100);

    // 📡 Envoi LoRaWAN
    Serial.println("📤 Envoi des données LoRa...");
    modem.beginPacket();
    modem.write((uint8_t*)&temp_sht20_int, sizeof(temp_sht20_int)); // t_0
    modem.write((uint8_t*)&hum_sht20_int, sizeof(hum_sht20_int));   // h
    modem.write((uint8_t*)&temp_ds18b20_int, sizeof(temp_ds18b20_int));  // t_1
    modem.write((uint8_t*)&temp_dht22_int, sizeof(temp_dht22_int));  // t_2
    modem.write((uint8_t*)&hum_dht22_int, sizeof(hum_dht22_int));  // h_2
    modem.write((uint8_t*)&poids_int, sizeof(poids_int)); // weight_kg
    modem.write((uint8_t*)&lux_int, sizeof(lux_int)); // light
    int err = modem.endPacket();

    if (err > 0) {
        Serial.println("✅ Message envoyé !");
        err_count = 0;
    } else {
        Serial.print("❌ Échec de l'envoi, erreur ");
        Serial.println(err);
        err_count++;

        if (err_count > 10) {
            Serial.println("🔄 Trop d'échecs, tentative de reconnexion...");
            connected = false;
            err_count = 0;
        }
    }

    Serial.println("⏳ Attente avant le prochain envoi...");
    delay(15000); // Pause pour respecter le duty cycle
}
