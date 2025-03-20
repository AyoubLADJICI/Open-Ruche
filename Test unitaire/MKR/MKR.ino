void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
}

void loop() {
    if (Serial1.available()) {
        String message = Serial1.readStringUntil('\n');
        Serial.println("Reçu : " + message);
    }
}
