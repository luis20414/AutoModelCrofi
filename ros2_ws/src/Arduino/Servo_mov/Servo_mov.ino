#include <Servo.h>

// Configuración del servo
Servo servo;
int pinServo = 9;
float servoAngle = 90;
const byte numMaxChars = 32;
char recievedChars[numMaxChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Listo");
  servo.attach(pinServo);
  
}

void loop() {
  // Parte 1: Lectura del ángulo para el servo por Serial
  readSerialData();
}

void readSerialData() {
  static byte index = 0;
  char finalLinea = '\n';
  char rc;
  float goalAngle = 0;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != finalLinea) {
      recievedChars[index] = rc;
      index++;
      if (index >= numMaxChars) {
        index = numMaxChars - 1;
      }
    }
    else {
      recievedChars[index] = '\0';
      index = 0;
      newData = true;
    }
  }

  if (newData == true) {
    goalAngle = atof(recievedChars);
    goalAngle = goalAngle;// - 0.15;
    servoAngle = 90 - (goalAngle * 57.2957); // Conversión de radianes a grados
    servoAngle = constrain(servoAngle, 60, 120); // Limitar ángulo a rango válido
    
    Serial.print("Angulo servo: ");
    Serial.println(servoAngle);
    
    servo.write(servoAngle);
    newData = false;
  }
}
