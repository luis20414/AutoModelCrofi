#include <Servo.h>  // Usamos la biblioteca Servo para PWM de 50Hz
Servo esc;

void setup() {
  esc.attach(9);  // Conectar señal ESC al pin D9
  esc.writeMicroseconds(2000);  // Envía señal máxima (calibración)
  delay(5000);  // Espera 5 segundos (haz sonar el ESC 2 veces)
  esc.writeMicroseconds(1000);  // Envía señal mínima (calibración)
  delay(5000);  // Espera 5 segundos (calibración lista)
}

void loop() {
  // Ejemplo: Motor al 50% de velocidad
  esc.writeMicroseconds(1500);  // 1000 = STOP, 2000 = Máximo
  delay(3000);
  esc.writeMicroseconds(1000);  // Detener motor
  delay(2000);
}
