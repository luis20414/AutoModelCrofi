#include <Servo.h>
Servo esc;

void setup() {
  Serial.begin(9600);
  esc.attach(9);  // Señal en pin D9
  delay(2000);
  Serial.println("Iniciando prueba...");
}

void loop() {
  // Prueba desde 1000µs (stop) hasta 2000µs (máximo) en pasos de 50µs
  for (int us = 1000; us <= 2000; us += 50) {
    esc.writeMicroseconds(us);
    Serial.print("Microsegundos: ");
    Serial.print(us);
    Serial.println(" - Ajusta y observa el motor");
    delay(5000);  // Espera 5 segundos por paso
  }
  esc.writeMicroseconds(1000);  // Detener motor al finalizar
  Serial.println("Prueba completada");
  while (1);  // Detener programa
}
