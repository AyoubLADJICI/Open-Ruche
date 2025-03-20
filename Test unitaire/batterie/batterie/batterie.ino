#define BATTERYPIN A0  // Broche utilisée pour mesurer la tension de la batterie

// Facteur de conversion basé sur le diviseur de tension 147kOhm/100kOhm
#define DIVISEUR_RATIO 1.47  

// Plage de tension de la batterie Li-ion
#define VBAT_MAX 4.2  // Tension pleine charge
#define VBAT_MIN 3.0  // Tension critique, en dessous danger pour la batterie

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
}

void loop() {
    // Lecture de la tension brute (0 à 1023)
    int valeurA0 = analogRead(BATTERYPIN);
    
    // Vérification si la valeur de l'ADC est dans une plage correcte
    if (valeurA0 < 0 || valeurA0 > 1023) {
        Serial.println("Erreur : Valeur ADC hors limites !");
    }

    Serial.println("--------------------------------------");
    Serial.print("Valeur ADC brute : ");
    Serial.println(valeurA0);

    // Conversion en tension mesurée sur A0
    float vA0 = (valeurA0 / 1023.0) * 3.3;  
    Serial.print("Tension mesurée sur A0 : ");
    Serial.print(vA0, 3);
    Serial.println(" V");

    // Calcul de la tension réelle de la batterie
    float vBat = vA0 * DIVISEUR_RATIO;
    Serial.print("Tension estimée de la batterie : ");
    Serial.print(vBat, 3);
    Serial.println(" V");

    // Vérification des valeurs aberrantes après conversion
    if (vBat > 4.5 || vBat < 2.5) {
        Serial.println("Erreur : Valeur anormale détectée !");
        Serial.println("Vérifiez la connexion du diviseur de tension.");
    }

    // Calcul du pourcentage de charge
    int minValue = (int)((1023 * VBAT_MIN) / 3.3);
    int maxValue = (int)((1023 * VBAT_MAX) / 3.3);
    int PourcentageBatterie;

    if (vBat >= VBAT_MAX) {
        PourcentageBatterie = 100;
    } else if (vBat <= VBAT_MIN) {
        PourcentageBatterie = 0;
    } else {
        PourcentageBatterie = (int)(((float)(vBat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN)) * 100);
    }


    // Protection contre valeurs hors limites
    if (PourcentageBatterie > 100) PourcentageBatterie = 100;
    else if (PourcentageBatterie < 0) PourcentageBatterie = 0;

    Serial.print("Niveau de charge de la batterie : ");
    Serial.print(PourcentageBatterie);
    Serial.println(" %");

    // Affichage des messages d'état de la batterie
    if (PourcentageBatterie == 100) {
        Serial.println("Batterie pleine.");
    } else if (PourcentageBatterie >= 80) {
        Serial.println("Batterie bien chargée.");
    } else if (PourcentageBatterie >= 60) {
        Serial.println("Batterie moyenne, recharge bientôt.");
    } else if (PourcentageBatterie >= 40) {
        Serial.println("Batterie faible.");
    } else if (PourcentageBatterie >= 20) {
        Serial.println("Batterie très faible, recharge nécessaire.");
    } else {
        Serial.println("Batterie critique, recharge immédiate !");
    }

    delay(5000);  // Pause de 5 secondes
}
