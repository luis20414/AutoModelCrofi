#include <Servo.h>

Servo esc;
const int pinESC = 9;

void setup() {
  Serial.begin(115200);
  
  esc.attach(pinESC);
  delay(5000);  // Espera crítica para ESC
  esc.writeMicroseconds(1500);  // Neutral
}

void loop() {
  // Parte 1: Lectura de motor por string
  if (Serial.available() > 6) {  // "motor XXXX" son 10 caracteres
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("motor ")) {
      int pwmValue = input.substring(6).toInt();
      pwmValue = constrain(pwmValue, 1000, 2000);
      esc.writeMicroseconds(pwmValue);
      return;
    }
  }
}