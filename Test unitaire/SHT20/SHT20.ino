#include "uFire_SHT20.h"
uFire_SHT20 sht20;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sht20.begin();
}

void loop()
{
  sht20.measure_all();
  Serial.println((String)sht20.tempC + "°C");
  Serial.println((String)sht20.RH + " %RH");
  Serial.println();
  delay(5000);
}

//Pour lire l'adresse du TSL2561 (Il vaut 0x40)
/*
void setup() {
    Serial.begin(115200);
    while (!Serial); // Attendre l'ouverture du moniteur série
    Serial.println("Scan I2C en cours...");

    Wire.begin();
}

void loop() {
    byte error, address;
    int nDevices = 0;

    Serial.println("Recherche des périphériques I2C...");
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Périphérique I2C trouvé à l'adresse 0x");
            Serial.println(address, HEX);
            nDevices++;
        }
    }

    if (nDevices == 0) {
        Serial.println("Aucun périphérique I2C détecté");
    } else {
        Serial.println("Scan terminé");
    }

    delay(5000); // Attendre 5 secondes avant le prochain scan
}
*/