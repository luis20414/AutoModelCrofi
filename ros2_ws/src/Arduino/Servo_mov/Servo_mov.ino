Servo servo;
float targetAngle = 90; // Ángulo inicial del servo
const int pinServo = 5;

void setup() {
  Serial.begin(115200);
  servo.attach(pinServo);
  servo.write(targetAngle); // Inicializa en 90°
}

void loop() {
  if (Serial.available() >= sizeof(float)) { // Espera 4 bytes (tamaño de un float)
    float rc;
    Serial.readBytes((char*)&rc, sizeof(rc)); // Lee el float binario
    targetAngle = 90 - (rc * 57.2957); // Convierte radianes a grados
    servo.write(int(targetAngle));
  }
}