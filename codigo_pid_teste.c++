#include <Servo.h>

const int pinoTrig = 8;
const int pinoEcho = 9;
Servo meuservo;

// --- 1. VARIÁVEIS DE SINTONIA (OS GANHOS) ---
// Estes são os valores que vai alterar durante os testes físicos
float Kp = 3.0;  // O "Músculo" (Comece com 3.0 e vá subindo)
float Ki = 0.0;  // A "Memória" (Deixe em zero para já)
float Kd = 0.0;  // O "Amortecedor" (Ajuste depois do Kp)

float setpoint = 20.0; // O alvo: equilibrar a bola aos 20 cm do sensor

// --- 2. MEMÓRIA DO PID ---
float erro_anterior = 0;
float integral = 0;
unsigned long tempo_anterior = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinoTrig, OUTPUT);
  pinMode(pinoEcho, INPUT);
  meuservo.attach(10);
  
  meuservo.write(90); // Posição de repouso (Feedforward)
  delay(2000);        // Dá-lhe 2 segundos para colocar a bola no trilho
  
  tempo_anterior = millis(); // Inicia o relógio antes de entrar no loop
}

void loop() {
  // 1. LEITURA ROBUSTA DO SENSOR
  digitalWrite(pinoTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinoTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinoTrig, LOW);

  long duracao = pulseIn(pinoEcho, HIGH, 25000); // Timeout de segurança
  float distanciaCm = (duracao * 0.034) / 2;

  // Filtro: O PID só faz contas se a bola estiver fisicamente no trilho (ex: 3cm a 40cm)
  if (distanciaCm >= 3.0 && distanciaCm <= 40.0) {
    
    // 2. CALCULA O TEMPO EXATO QUE PASSOU (Delta Time)
    unsigned long tempo_atual = millis();
    float dt = (tempo_atual - tempo_anterior) / 1000.0; // Tempo em segundos (ex: 0.05s)
    
    if (dt > 0.0) { // Previne erro matemático de divisão por zero
      
      // 3. CALCULA O ERRO
      float erro = setpoint - distanciaCm;

      // 4. AÇÃO PROPORCIONAL (P)
      float P = Kp * erro;

      // 5. AÇÃO INTEGRAL (I) com proteção ANTI-WINDUP
      integral = integral + (erro * dt); // Acumula o erro no tempo
      if (integral > 40) integral = 40;  // Impede que a memória cresça ao infinito (Lim. Máx)
      if (integral < -40) integral = -40; // Lim. Mínimo
      float I = Ki * integral;

      // 6. AÇÃO DERIVATIVA (D)
      float velocidade_erro = (erro - erro_anterior) / dt; 
      float D = Kd * velocidade_erro;

      // 7. EQUAÇÃO FINAL DO PID
      float PID_Total = P + I + D;

      // 8. ATUALIZA A MEMÓRIA PARA O PRÓXIMO CICLO
      erro_anterior = erro;
      tempo_anterior = tempo_atual;

      // 9. APLICAÇÃO AO MOTOR (Soma o PID ao centro físico de 90 graus)
      float angulo_servo = 90 + PID_Total;

      // 10. ZONAS DE SATURAÇÃO (Travões físicos)
      // Substitua 135 e 45 pelos limites mecânicos reais da sua peça 3D
      if (angulo_servo > 135) angulo_servo = 135;
      if (angulo_servo < 45)  angulo_servo = 45;

      // Envia o comando final para o servo
      meuservo.write(angulo_servo);

      // 11. DEPURAÇÃO NO MONITOR SÉRIE
      Serial.print("Dist: "); Serial.print(distanciaCm);
      Serial.print(" | Erro: "); Serial.print(erro);
      Serial.print(" | PID: "); Serial.print(PID_Total);
      Serial.print(" | Ang: "); Serial.println(angulo_servo);
    }
  } else {

    tempo_anterior = millis(); 
  }

  // Pausa curta para não sobrecarregar as leituras do sensor HC-SR04
  delay(50);
}
