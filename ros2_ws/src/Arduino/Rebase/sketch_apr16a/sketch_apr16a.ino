#include <Servo.h>

Servo esc;
const int pinESC = 9;

void setup() {
  Serial.begin(115200);
  esc.attach(pinESC);
  delay(5000);
  esc.writeMicroseconds(1500); // Neutral
}

void loop() {
  if (Serial.available() >= sizeof(float)) {
    float received_value;
    byte* bytePtr = (byte*)&received_value;
    
    // Lee los 4 bytes
    for (int i = 0; i < sizeof(float); i++) {
      bytePtr[i] = Serial.read();
    }

    // Debug: Imprime los bytes en HEX
    Serial.print("Bytes recibidos: ");
    for (int i = 0; i < sizeof(float); i++) {
      Serial.print(bytePtr[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Procesa el valor
    if (received_value == 1540.0f || received_value == 1500.0f || received_value == 1390.0f) {
      esc.writeMicroseconds((int)received_value);
      Serial.print("Motor: ");
    } else {
      Serial.print("Valor no reconocido: ");
    }
    Serial.println(received_value, 1); // 1 decimal para claridad
  }
}
