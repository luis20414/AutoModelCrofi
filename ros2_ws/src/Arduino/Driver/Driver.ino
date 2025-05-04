#include <Servo.h>

Servo esc;
const int pinESC = 9; // Pin D9 para el ESC
int pwmValue = 1500;  // Valor inicial (neutral)

void setup() {
  Serial.begin(115200);
  esc.attach(pinESC);
  delay(5000);  // Espera crítica para ESC
  esc.writeMicroseconds(1500);  // Neutral
  Serial.println("ESC initialized and set to neutral.");
}

void loop() {
  // Verificar si hay datos disponibles en el puerto serie
  if (Serial.available() >= 4) { // Se esperan 4 bytes para un int32
    int32_t receivedValue = 0;

    // Leer los 4 bytes del entero de 32 bits
    for (int i = 0; i < 4; i++) {
      ((uint8_t*)&receivedValue)[i] = Serial.read();
    }

    // Limitar el valor recibido al rango PWM permitido
    pwmValue = constrain(receivedValue, 1410, 1560);

    // Enviar el valor al ESC
    esc.writeMicroseconds(pwmValue);

    // Imprimir el valor recibido para depuración
    Serial.print("Received PWM: ");
    Serial.println(pwmValue);
  }
}
