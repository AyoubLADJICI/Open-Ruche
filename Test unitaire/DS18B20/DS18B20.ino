//source : https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/Simple/Simple.ino
//schema cablage : https://www.moussasoft.com/ds18b20-arduino/

// Include the libraries we need
/*
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 0 on the Arduino
#define ONE_WIRE_BUS 0

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/*
 * The setup function. We only start the sensors here
 */

/*
void setup(void) {
  // start serial port
  Serial.begin(11500);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
}

/*
 * Main function, get and show the temperature
 */
/*
void loop(void) {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  delay(1500);
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }
}
*/

//Adresse de la Sonde 1 : 28C63595F0013C51
//Adresse de la Sonde 2 : 281EEC75D0013C44

#include <OneWire.h>
#include <DallasTemperature.h>

// Définition du bus OneWire
#define ONE_WIRE_BUS 0  // Connecteur D0 sur la carte MKR WAN 1310
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

DeviceAddress ds18b20_1, ds18b20_2;  // Adresses des deux sondes

void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Serial.print("0");  // Ajouter un 0 pour le format hexadécimal
        Serial.print(deviceAddress[i], HEX);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    ds18b20.begin();
    
    // Vérifier combien de capteurs DS18B20 sont détectés
    int numDevices = ds18b20.getDeviceCount();
    Serial.print("Nombre de sondes DS18B20 détectées : ");
    Serial.println(numDevices);

    if (numDevices < 2) {
        Serial.println("⚠️ ATTENTION : Moins de 2 sondes détectées !");
    }

    // Récupérer et afficher les adresses des sondes
    if (ds18b20.getAddress(ds18b20_1, 0)) {
        Serial.print("Adresse Sonde 1 : ");
        printAddress(ds18b20_1);
        Serial.println();
    } else {
        Serial.println("⚠️ Sonde 1 non détectée !");
    }

    if (ds18b20.getAddress(ds18b20_2, 1)) {
        Serial.print("Adresse Sonde 2 : ");
        printAddress(ds18b20_2);
        Serial.println();
    } else {
        Serial.println("⚠️ Sonde 2 non détectée !");
    }
}

void loop() {
    ds18b20.requestTemperatures();
    
    // Lecture des températures des deux sondes
    float temp_ds18b20_1 = ds18b20.getTempC(ds18b20_1);
    float temp_ds18b20_2 = ds18b20.getTempC(ds18b20_2);
    
    Serial.print("Température Sonde 1 : ");
    Serial.print(temp_ds18b20_1);
    Serial.println(" °C");

    Serial.print("Température Sonde 2 : ");
    Serial.print(temp_ds18b20_2);
    Serial.println(" °C");

    delay(2000);  // Pause de 2 secondes avant la prochaine lecture
}

