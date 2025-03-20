#include <Wire.h>
#include <Digital_Light_TSL2561.h>

void setup() {
    Wire.begin();
    Serial.begin(115200);
    TSL2561.init();
}

void loop() {
    Serial.print("The Light value is: ");
    Serial.println(TSL2561.readVisibleLux());
    delay(1000);
}

//Pour lire l'adresse du TSL2561 (Il vaut 0x29)
/*
#include <Wire.h>

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