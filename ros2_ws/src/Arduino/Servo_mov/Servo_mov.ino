#include <Servo.h>
#include <NewPing.h>  // Librería optimizada para ultrasonido

Servo servo;
const int pinServo = 5;
float targetAngle = 90;

const int pinTrigger1 = 6;
const int pinEcho1 = 7;
const int pinTrigger2 = 8;  // Segundo sensor
const int pinEcho2 = 9;     // Segundo sensor

const int pinLedDD = 10;
const int pinLedDT = 11;
const int pinLedID = 12;
const int pinLedIT = 13;

const int maxDistancia = 200;
const int distEstacionado = 15;

NewPing sonar1(pinTrigger1, pinEcho1, maxDistancia);
NewPing sonar2(pinTrigger2, pinEcho2, maxDistancia);
bool estacionado = false;

void setup() {
  Serial.begin(115200);
  servo.attach(pinServo);
  servo.write(targetAngle);
  pinMode(pinLedDD, OUTPUT);
  pinMode(pinLedDT, OUTPUT);
  pinMode(pinLedID, OUTPUT);
  pinMode(pinLedIT, OUTPUT);
}

void loop() {
  // --- Parte 1: Control del Servo (Automodelo) ---
  if (Serial.available() >= sizeof(float)) {
    float rc;
    Serial.readBytes((char*)&rc, sizeof(rc));
    targetAngle = 90 + (rc * 57.2957);
    targetAngle = constrain(targetAngle, 50, 130);
    servo.write(int(targetAngle));

    Serial.print("Ángulo servo: ");
    Serial.println(targetAngle);
  }

  // --- Parte 2: Encender LEDs según la letra recibida ---
  if (Serial.available() >= 5) {  // 1 letra + 4 dígitos
    char tipoDato = Serial.read();
    if (isalpha(tipoDato)) {  // Verifica si es una letra
      char digitos[4];
      Serial.readBytes(digitos, 4);
      int valor = atoi(digitos);

      if (valor > 0) {
        digitalWrite(pinLedDD, HIGH);
        digitalWrite(pinLedDT, HIGH);
        digitalWrite(pinLedID, LOW);
        digitalWrite(pinLedIT, LOW);
      } else {
        digitalWrite(pinLedDD, LOW);
        digitalWrite(pinLedDT, LOW);
        digitalWrite(pinLedID, HIGH);
        digitalWrite(pinLedIT, HIGH);
      }
    }
  }

  // --- Parte 3: Ultrasonido (Estacionado Autónomo) ---
  unsigned int distancia1 = sonar1.ping_cm();
  unsigned int distancia2 = sonar2.ping_cm();
  
  if ((distancia1 > 0 && distancia1 <= distEstacionado) || (distancia2 > 0 && distancia2 <= distEstacionado)) {
    if (!estacionado) {
      Serial.write('E');
      estacionado = true;
      Serial.println("¡Señal 'E' enviada!");
    }
  } else {
    estacionado = false;
  }
}