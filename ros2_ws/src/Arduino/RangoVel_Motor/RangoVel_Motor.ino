#include <Servo.h>
Servo esc;

void setup() {
  esc.attach(9);
  esc.writeMicroseconds(1000);  // Inicia en STOP
  delay(2000);
}

void loop() {
  int potValue = analogRead(A0);  // Leer potenciómetro en A0
  int speed = map(potValue, 0, 1023, 1000, 2000);  // Mapear a 1000–2000us
  esc.writeMicroseconds(speed);  // Ajustar velocidad
  delay(20);
}
