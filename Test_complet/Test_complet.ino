#include <MKRWAN.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HX711.h"
#include <BH1750.h>
#include "ArduinoLowPower.h"

//Configuration du module LoRa
LoRaModem modem;

/*Initialisation des capteurs*/
//Initialisation de la sonde SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();

//Initialisation de la sonde DS18B20
#define ONE_WIRE_BUS 0 //Connecteur D0 sur la carte MKR WAN 1310 (pour la connecter a la broche DATA de la sonde)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
DeviceAddress ds18b20_1, ds18b20_2;  // Adresses des deux sondes

//Initialiation du capteur DHT22
#define DHTPIN 2 //Connecteur D2 sur la carte
#define DHTTYPE DHT22 
DHT_Unified dht(DHTPIN, DHTTYPE);

//Initialisation du module de pesee
#define LOADCELL_DOUT_PIN 3 //Connecteur D3
#define LOADCELL_SCK_PIN 4 //Connecteur D4
HX711 scale;
float calibration_factor = -8950;

//Initialisation du capteur de lumiere
BH1750 lightMeter;

/*Batterie*/
#define VBAT_PIN A0  //Broche utilisée pour mesurer la tension de sortie du pont diviseur  
#define DIVISEUR_RATIO 1.47 // Facteur de conversion basé sur le diviseur de tension 100kΩ / 47kΩ
//Plage de tension de la batterie (basée sur une Accu LiPo 3.7V)
#define VBAT_MAX 4.2 //Tension pleine charge
#define VBAT_MIN 3.0 //Tension déchargée

// Clés LoRa OTAA
String AppEUI = "213D57ED00000000";
String AppKEY = "8EED2DFE0FA94091FC093C1EBBF382C8";

bool connected;
int err_count;
short con;

/* Fonction pour lire la tension de la batterie */
float readBatteryLevel() {
  analogReadResolution(12);
  float vBat = ((analogRead(VBAT_PIN) / 4095.0) * 3.3) * DIVISEUR_RATIO;
  return vBat;
  //si on souhaite retourner le pourcentage, decommentez la ligne suivante
  //return (vBat >= VBAT_MAX) ? 100 : (vBat <= VBAT_MIN) ? 0 : (int)(((vBat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN)) * 100);
}

void setup() {
  Serial.begin(115200);
  //decommentez la ligne suivante pour voir les messages dans le terminale
  //commentez la ligne suivante pour que notre prototype puisse fonctionner en eteignant l'interrupteur puis en le rallumant
  //while (!Serial); 
  modem.begin(EU868);
  delay(1000);
  connected = false;
  err_count = 0;
  con = 0;

  Wire.begin();
  
  if (!sht31.begin(0x44)) {
    //Serial.println("Erreur : Impossible de détecter le SHT31 !");
  }

  if (sht31.isHeaterEnabled()){
    //Serial.println("ENABLED");
  }else {
    //Serial.println("DISABLED");
  }
  ds18b20.begin();
  dht.begin();
  lightMeter.begin();

  // Vérifier la présence des deux sondes DS18B20
  if (ds18b20.getAddress(ds18b20_1, 0)) {
    ds18b20.setResolution(ds18b20_1, 12);  // Resolution max (12 bits)
  }
  if (ds18b20.getAddress(ds18b20_2, 1)) {
    ds18b20.setResolution(ds18b20_2, 12);
  }

  //Initialisation de la balance
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare(); //Remise a zero
  scale.set_scale(calibration_factor);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (!connected) {
    //Serial.print("Tentative de connexion LoRa : ");
    //Serial.println(++con);
    int ret=modem.joinOTAA(AppEUI, AppKEY);
    if (ret) {
      connected = true;
      modem.minPollInterval(60);
      modem.dataRate(5);  // switch to SF7
      delay(100);         // because ... more stable
      err_count = 0;
    }
  }

  if (connected) {
    //Serial.print("Connecte : ");

    /*Lecture des capteurs*/
    //🔹Mesure Température et Humidité du SHT31
    float temp_sht31 = sht31.readTemperature();
    float hum_sht31 = sht31.readHumidity();
    
    //🔹Mesure Température DS18B20
    ds18b20.requestTemperatures();
    float temp_ds18b20_1 = ds18b20.getTempC(ds18b20_1);
    float temp_ds18b20_2 = ds18b20.getTempC(ds18b20_2);
   
    //🔹Mesure Température et Humidité du DHT22
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    float temp_dht22 = event.temperature;
    dht.humidity().getEvent(&event);
    float hum_dht22 = event.relative_humidity;
    
    //🔹Mesure du Poids (Balance HX711)
    float poids = scale.get_units() / 2.2046;
    if (poids < 0) poids = 0;  // Correction si la balance affiche un poids négatif
    
    //🔹Lecture du capteur de lumière BH1750
    float lux = lightMeter.readLightLevel();

    //🔹Lecture du pourcentage de la batterie
    float batteryLevel = readBatteryLevel();
        
    //Conversion en short (multiplié par 100 pour garder 2 décimales)
    short temp_sht31_int = (short)(temp_sht31 * 100);
    short hum_sht31_int = (short)(hum_sht31 * 100);
    short temp_ds18b20_1_int = (short)(temp_ds18b20_1 * 100);
    short temp_ds18b20_2_int = (short)(temp_ds18b20_2 * 100);
    short temp_dht22_int = (short)(temp_dht22 * 100);
    short hum_dht22_int = (short)(hum_dht22 * 100);
    short poids_int = (short)(poids * 100);
    short lux_int = (short)(lux * 100);
    short batterie_int = (short)(batteryLevel * 100);

    //Envoi LoRa
    int err = 0;
    modem.beginPacket();
    modem.write((uint8_t*)&temp_sht31_int, sizeof(temp_sht31_int));
    modem.write((uint8_t*)&hum_sht31_int, sizeof(hum_sht31_int));
    modem.write((uint8_t*)&temp_ds18b20_1_int, sizeof(temp_ds18b20_1_int));
    modem.write((uint8_t*)&temp_ds18b20_2_int, sizeof(temp_ds18b20_2_int));
    modem.write((uint8_t*)&temp_dht22_int, sizeof(temp_dht22_int));
    modem.write((uint8_t*)&hum_dht22_int, sizeof(hum_dht22_int));
    modem.write((uint8_t*)&poids_int, sizeof(poids_int));
    modem.write((uint8_t*)&lux_int, sizeof(lux_int));
    modem.write((uint8_t*)&batterie_int, sizeof(batterie_int));
    err = modem.endPacket();

    if (err > 0) {
      err_count = 0;
      //Serial.println("Message envoyé avec succès");
    } else {
      //Serial.println("Erreur : ");
      //Serial.println(err);
      // Confirmation not received - jam or coverage fault
      err_count++;
      if (err_count > 50) {
        //Serial.println("Trop d'échecs, reconnexion...");
        connected = false;
        err_count = 0;
      }
      // wait for 2min for duty cycle with SF12 - 1.5s frame
      for (int i = 0 ; i < 120 ; i++ ) {
        delay(1000);
      }
    }
    
    /*Optimisation de notre prototype*/
    scale.power_down(); //Mettre le HX711 en mode veille 
    LowPower.deepSleep(600000); //Passage en Deep Sleep pendant 10 minutes
    //LowPower.deepSleep(15000); //decommentez juste pendant les tests
    scale.power_up(); //reactivation du hx711
    }
}
