#define VBAT_PIN A0  // Broche utilisée pour mesurer la tension de la batterie

// Facteur de conversion basé sur le diviseur de tension 100kΩ / 47kΩ
#define DIVISEUR_RATIO 3.13

// Plage de tension de la batterie (basée sur une Li-ion 3.7V)
#define VBAT_MAX 4.5  // Tension pleine charge
#define VBAT_MIN 3.2  // Tension déchargée (batterie critique)

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Attente de l'ouverture du port série
    analogReadResolution(12);  // Utilisation de l'ADC en 12 bits (valeurs de 0 à 4095)
}

void loop() {
    // Lecture brute de l'ADC (valeur entre 0 et 4095)
    int raw = analogRead(VBAT_PIN);
    Serial.println("test raw : ");
    Serial.println(raw);
    // Conversion en tension mesurée sur A0
    float vA0 = (raw / 4095.0) * 3.3;  
    Serial.println("test va0 : ");
    Serial.println(vA0);

    // Calcul de la tension réelle de la batterie
    float vBat = vA0 * DIVISEUR_RATIO; 
    Serial.println("test vBat : ");
    Serial.println(vBat);
    // Protection contre des valeurs aberrantes
    if (vBat > 4.5 || vBat < 2.5) {  
        Serial.println("Erreur : Valeur anormale détectée !");
        Serial.println("Vérifiez la connexion du diviseur de tension.");
    }

    // Calcul du pourcentage de charge (interpolation linéaire)
    int batteryLevel = 0;
    if (vBat >= VBAT_MAX) {
        batteryLevel = 100;
    } else if (vBat <= VBAT_MIN) {
        batteryLevel = 0;
    } else {
        batteryLevel = (int)((vBat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0);
    }

    // Affichage des résultats
    Serial.println("--------------------------------------");
    Serial.print("Tension mesurée sur A0 : ");
    Serial.print(vA0, 3);
    Serial.println(" V");

    Serial.print("Niveau de charge de la batterie : ");
    Serial.print(batteryLevel);
    Serial.println(" %");

    // Messages d'état de la batterie
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

    delay(5000);  // Rafraîchissement toutes les 5 secondes
}