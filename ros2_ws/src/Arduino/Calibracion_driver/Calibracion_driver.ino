#include <Servo.h>
Servo esc;

void setup() {
  esc.attach(9);  // Conectar el cable de señal (blanco) al pin 9
  delay(2000);    // Espera inicial

  // Secuencia de calibración:
  esc.writeMicroseconds(2000);  // Envía pulso máximo (2000µs)
  delay(2000);                  // Espera 5 segundos (ESC emitirá sonidos)
  esc.writeMicroseconds(1000);  // Envía pulso mínimo (1000µs)
  delay(3000);                  // Espera 5 segundos (ESC confirmará con pitidos)
  esc.writeMicroseconds(1500);  // Neutral (motor detenido)
}

void loop() {
  // No es necesario agregar nada aquí.
}
