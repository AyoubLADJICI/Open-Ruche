void setup() {
    Serial1.begin(9600); // Communication série via Serial1
}

void loop() {
    Serial1.println("Talla B-Nahl");
    delay(1000);
}
