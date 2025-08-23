#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// === MAPEAMENTO DE PINOS ===
const int ENA = 16;
const int ENB = 23;
const int IN1 = 22;
const int IN2 = 21;
const int IN3 = 18;
const int IN4 = 17;
#define standby 19

const int numSensores = 6;
int pinoSensores[numSensores] = {14, 27, 26, 25, 33, 32};
int SensorF[numSensores];
const int MediaC = 12;

#define portaAtivacao 12
const int canal_PWM_A = 0;
const int canal_PWM_B = 1;
const int freq = 1000;
const int resolucao = 8;

// === CONTROLE PID ===
float kp = 1.0;
float ki = 0.0;
float kd = 0.25;
float erro = 0;
float UltimoErro = 0;
float somaErros = 0;
float correcao = 0;

float VelE = 0;
float VelD = 0;
float mediaE = 100;
float mediaD = 99;

int contadorD = 0;   // Contador do sensor direito
int contadorE = 0;   // Contador do sensor esquerdo
int alvoContador = 54; // Quantas detecções o robô busca antes de parar

float threshold = 0;

#define sensorD 36
//#define sensorE 13

unsigned long tempoAnterior = 0;
const unsigned long intervaloLeiturAA = 1000;
const unsigned long intervaloLeitura = 90;

bool pidAtivo = false; // Controle do modo PID

void setup() {
  Serial.begin(115200);
  SerialBT.begin("DeLoreanV2"); // Nome do Bluetooth
  SerialBT.println("Conecte-se no RoboRemo!");

  pinMode(portaAtivacao, INPUT_PULLUP);
  pinMode(standby, OUTPUT);
  digitalWrite(standby, HIGH);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(sensorD, INPUT);

  ledcSetup(canal_PWM_A, freq, resolucao);
  ledcAttachPin(ENA, canal_PWM_A);
  ledcSetup(canal_PWM_B, freq, resolucao);
  ledcAttachPin(ENB, canal_PWM_B);

  for (int i = 0; i < numSensores; i++) {
    pinMode(pinoSensores[i], INPUT);
  }

  calculaMediaSensores();
}

void calculaMediaSensores() {
  float contadorSF = 0;
  for (int i = 0; i < numSensores; i++) {
    contadorSF = 0;
    for (int m = 0; m < MediaC; m++) {
      contadorSF += analogRead(pinoSensores[i]);
    }
    SensorF[i] = contadorSF / MediaC;
  }

  contadorSF = 0;
  for (int i = 0; i < numSensores; i++) {
    contadorSF += SensorF[i];
  }
  threshold = contadorSF / numSensores;
}

void retornaErro() {
  if (SensorF[0] <= threshold && SensorF[1] > threshold && SensorF[2] > threshold && SensorF[3] > threshold && SensorF[4] > threshold && SensorF[5] > threshold)
    erro = 3;
  else if (SensorF[0] <= threshold && SensorF[1] <= threshold && SensorF[2] > threshold && SensorF[3] > threshold && SensorF[4] > threshold)
    erro = 2;
  else if (SensorF[0] <= threshold && SensorF[1] <= threshold && SensorF[2] <= threshold && SensorF[3] > threshold && SensorF[4] > threshold)
    erro = 1;
  else if (SensorF[0] > threshold && SensorF[1] > threshold && SensorF[2] <= threshold && SensorF[3] <= threshold && SensorF[4] > threshold && SensorF[5] > threshold)
    erro = 0;
  else if (SensorF[2] > threshold && SensorF[3] <= threshold && SensorF[4] <= threshold && SensorF[5] <= threshold)
    erro = -1;
  else if (SensorF[3] > threshold && SensorF[4] <= threshold && SensorF[5] <= threshold)
    erro = -2;
  else if (SensorF[4] > threshold && SensorF[5] <= threshold)
    erro = -3;
  else
    erro = 0;
}

float CalculoPID(float erro) {
  somaErros += erro;
  float PID_C = (kp * erro) + (ki * somaErros) + (kd * (erro - UltimoErro));
  UltimoErro = erro;
  return PID_C;
}

void loop() {
  // Recebe comandos do RoboRemo
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "pid") {
      pidAtivo = true;
      SerialBT.println("Modo PID ATIVADO");
    } 
    else if (cmd == "off") {
      pidAtivo = false;
      pararMotores();
      SerialBT.println("Robo DESLIGADO");
    } 
    else if (cmd.startsWith("velocidade")) {
      int val = cmd.substring(10).toInt();
      mediaE = constrain(val, 0, 255);
      mediaD = constrain(val - 1, 0, 255);
      SerialBT.printf("Velocidade ajustada para %d\n", val);
    }
    else if (cmd == "maisD") {
      alvoContador++;
      SerialBT.printf("ContadorD incrementado: %d (Alvo: %d)\n", contadorD, alvoContador);
    }
    else if (cmd == "menosD") {
      alvoContador--;
      SerialBT.printf("ContadorD decrementado: %d (Alvo: %d)\n", contadorD, alvoContador);
    }
  }

  // Lógica do sensor direito
  int sensordValorD = analogRead(sensorD);
  if (sensordValorD < 2000) {
    if (millis() - tempoAnterior > intervaloLeiturAA) {
      contadorD++;
      SerialBT.printf("Sensor D DETECTADO (%d/%d)\n", contadorD, alvoContador);
      tempoAnterior = millis();
    }
  }

   if (contadorD >= alvoContador) {
    pararMotores();
    SerialBT.println("Alvo atingido, motores parados!");
    return;
  }


  // Se PID ativo, faz leitura e correção
  if (pidAtivo) {
    float contadorSF = 0;
    for (int i = 0; i < numSensores; i++) {
      contadorSF = 0;
      for (int m = 0; m < MediaC; m++) {
        contadorSF += analogRead(pinoSensores[i]);
      }
      SensorF[i] = contadorSF / MediaC;
    }

    retornaErro();
    float PID_C = CalculoPID(erro);

    if (erro == 0) {
      moverFrente(100, 99);
    } else if (PID_C < 0) {
      VelE = mediaE;
      VelD = constrain(mediaD + PID_C * 100, 0, 255);
      moverFrente(VelE, VelD);
    } else if (PID_C > 0) {
      VelE = constrain(mediaE - PID_C * 100, 0, 255);
      VelD = mediaD;
      moverFrente(VelE, VelD);
    }
  }
}

void moverFrente(int velE, int velD) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(canal_PWM_A, velE);
  ledcWrite(canal_PWM_B, velD);
}

void pararMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(canal_PWM_A, 0);
  ledcWrite(canal_PWM_B, 0);
}
