#include <Servo.h>

const int pinoTrig = 8;
const int pinoEcho = 9;
Servo meuservo;

void setup() {
  Serial.begin(9600);
  pinMode(pinoTrig, OUTPUT);
  pinMode(pinoEcho, INPUT);
  meuservo.attach(10);
}

void loop() {
  long duracao;
  float distanciaCm;
  float angulo_servo = 90;

  // Garante um pulso limpo em TRIG
  digitalWrite(pinoTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinoTrig, HIGH);
  delayMicroseconds(10); // Pulso de 10 microssegundos
  digitalWrite(pinoTrig, LOW);

  // Lê o tempo de resposta no pino ECHO
  duracao = pulseIn(pinoEcho, HIGH);

  // Calcula a distância em centímetros
  distanciaCm = (duracao * 0.034) / 2;

  // Trecho solicitado para calcular e aplicar o ângulo
  angulo_servo = (angulo_servo + distanciaCm - 20)*10;
  meuservo.write(angulo_servo);  

  Serial.print("Distancia: ");
  Serial.print(distanciaCm);
  Serial.print(" cm | Angulo: ");
  Serial.println(angulo_servo);

  delay(100);
}
