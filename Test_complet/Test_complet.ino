#include <MKRWAN.h>
#include <Wire.h>
#include "uFire_SHT20.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//Configuration du module LoRa
LoRaModem modem;

//Initialisation du capteur SHT20
uFire_SHT20 sht20; 

// Initialisation du capteur DS18B20
#define ONE_WIRE_BUS 0  // Broche DATA du DS18B20 (D0 sur le MKR WAN 1310)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

//Clés LoRa pour OTAA
String AppEUI = "213D57ED00000000";
String AppKEY = "8EED2DFE0FA94091FC093C1EBBF382C8";

bool connected;
int err_count;
short con; 

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
      return;
    }

    // 🔹 Mesure Température DS18B20
    ds18b20.requestTemperatures();
    float temp_ds18b20 = ds18b20.getTempCByIndex(0);

    if (temp_ds18b20 == DEVICE_DISCONNECTED_C) {
      Serial.println("❌ Erreur : Capteur DS18B20 non détecté !");
      return;
    }

    // Convertir en short (multiplié par 100 pour garder 2 décimales)
    short temp_sht20_int = (short)(temp_sht20 * 100);
    short hum_sht20_int = (short)(hum_sht20 * 100);
    short temp_ds18b20_int = (short)(temp_ds18b20 * 100);

    // 📊 Affichage des valeurs
    Serial.print("🌡 Température SHT20 : "); Serial.print(temp_sht20); Serial.println(" °C");
    Serial.print("💧 Humidité SHT20 : "); Serial.print(hum_sht20); Serial.println(" %");
    Serial.print("🌡 Température DS18B20 : "); Serial.print(temp_ds18b20); Serial.println(" °C");

    Serial.println("📡 Envoi du message LoRa...");
    int err = 0;
    modem.beginPacket();
    modem.write((uint8_t*)&temp_sht20_int, sizeof(temp_sht20_int)); // Envoi de la température
    modem.write((uint8_t*)&hum_sht20_int, sizeof(hum_sht20_int));   // Envoi de l'humidité
    modem.write((uint8_t*)&temp_ds18b20_int, sizeof(temp_ds18b20_int));  // Température DS18B20
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