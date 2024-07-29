// Variáveis sensor frontal
const int numSensores = 4;
int SensorFrontal[numSensores] = {A1, A2, A3, A4};
int SensorF[numSensores];
const int MediaC = 8; // Número de amostras para calcular a média
#define portaAtivacao 13
float erro = 0;
 
// Variáveis sensor lateral
#define sensorD A6
#define sensorE A7
unsigned long tempoAnterior = 0;
unsigned long tempoAnteriorE = 0;
const unsigned long intervaloLeitura = 100; // intervalo de 1 segundo
const unsigned long intervaloLeitura2 = 19004;
 
 
// Variáveis motor
const int PINO_IN1 = 6; // Primeiro motor (esquerda)
const int PINO_IN2 = 7;
const int PINO_IN3 = 9; // Segundo motor (direita)
const int PINO_IN4 = 10;
const int PINO_ENA = 5; 
const int PINO_ENB = 11;
#define standby 8
 
// Variáveis PID
float kp = 7.5;//9.35;//1.0;/0.6;/// 7.5;//19.0;
float kd = 0.382;//0.85;//.6425;//0.8429;// 0.382;//0.382;   //bateria 8.8V
float UltimoErro = 0;
float derivada = 0;
float correcao = 0;
float VelE = 0;
float VelD = 0;
float threshold = 0;
float mediaE =100;//180;//100;
float mediaD =100;//180;//100;
int contadorD = 0;
int contadorE = 0;
//int pwmBase = 110;
 
void setup() {
    pinMode(portaAtivacao, INPUT);
    digitalWrite(portaAtivacao, HIGH);
    pinMode(sensorD, INPUT);
    pinMode(sensorE, INPUT);  
 
    for (int i = 0; i < numSensores; i++) {
        pinMode(SensorFrontal[i], INPUT);
    }
    Serial.begin(9600);
 
    digitalWrite(PINO_IN1, LOW); 
    digitalWrite(PINO_IN2, LOW);
    digitalWrite(PINO_IN3, LOW);
    digitalWrite(PINO_IN4, LOW);
    digitalWrite(PINO_ENA, LOW);
    digitalWrite(PINO_ENB, LOW);
    digitalWrite(standby, HIGH);
 
    calculaMediaSensores(); //Só é chamada no setup, tem que dar um reset no robo antes
 
}
 
void calculaMediaSensores() {
    float contadorSF = 0;
    for (int i = 0; i < numSensores; i++) {
        contadorSF = 0;
        for (int m = 0; m < MediaC; m++) {
            contadorSF += analogRead(SensorFrontal[i]);
        }
        SensorF[i] = contadorSF / MediaC;     
    }
 
    contadorSF = 0;
    for (int c = 0; c < numSensores; c++) {
        contadorSF += SensorF[c];
    }
    threshold = contadorSF / numSensores;
 
    mediaLateral();
}
 
void mediaLateral() {
    float somaE = 0;
    float somaD = 0;
    for (int i = 0; i < numSensores / 2; i++) {
        somaE += SensorF[i];
    }
    for (int i = numSensores / 2; i < numSensores; i++) {
        somaD += SensorF[i];
    }
 
  //  mediaE = somaE / (numSensores / 2);
//   mediaD = somaD / (numSensores / 2);
/*   Serial.print("Media Esquerda: ");
    Serial.println(mediaE);
    Serial.print("Media Direita: ");
    Serial.println(mediaD);*/
}
 
void retornaErro() { // ATUALIZAR PRA 6 SENSORES
    // Faz a lógica dos ifs utilizando SensorF[i]
    // Threshold <= ta na linha branca
/*   if (SensorF[0] > threshold && SensorF[1] > threshold && SensorF[2] <= threshold && SensorF[3] <= threshold)
        erro = 0;*/ 
    // CURVA PARA A DIREITA: ERRO POSITIVO
    if (SensorF[0] <= threshold && SensorF[1] > threshold && SensorF[2] > threshold && SensorF[3] > threshold){
        erro = 2; 
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, LOW);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, LOW);
        analogWrite(PINO_ENA, 0);
        analogWrite(PINO_ENB, 0);
        delay(15);}
    if (SensorF[0] <= threshold && SensorF[1] <= threshold && SensorF[2] > threshold && SensorF[3] > threshold){
        erro = 1.5;} 
    if (SensorF[0] <= threshold && SensorF[1] <= threshold && SensorF[2] <= threshold && SensorF[3] > threshold){
        erro = 0.5;}   
    // CURVA PARA A ESQUERDA: ERRO NEGATIVO
    if (SensorF[0] > threshold && SensorF[1] <= threshold && SensorF[2] <= threshold && SensorF[3] > threshold){
        erro = 0; }
    if (SensorF[0] <= threshold && SensorF[1] <= threshold && SensorF[2] <= threshold && SensorF[3] <= threshold){
        erro = 0; }
    if (SensorF[0] > threshold && SensorF[1] > threshold && SensorF[2] > threshold && SensorF[3] <= threshold){
        erro = -2; 
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, LOW);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, LOW);
        analogWrite(PINO_ENA, 0);
        analogWrite(PINO_ENB, 0);
        delay(15);}
    if (SensorF[0] > threshold && SensorF[1] > threshold && SensorF[2] <= threshold && SensorF[3] <= threshold){
        erro = -1.5;} 
    if (SensorF[0] > threshold && SensorF[1] <= threshold && SensorF[2] <= threshold && SensorF[3] <= threshold){
        erro = -0.5; }
    // ROBÔ FORA DA CURVA
    else if (SensorF[0] > threshold && SensorF[1] > threshold && SensorF[2] > threshold && SensorF[3] > threshold) {
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, LOW);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, LOW);
        digitalWrite(PINO_ENA, LOW);
        digitalWrite(PINO_ENB, LOW); 
    }
    }


 
float CalculoPID(float erro) {
    float prop = erro;
    float PID_C = (kp * prop) + (kd * (erro - UltimoErro));
    UltimoErro = erro;
    Serial.println(PID_C);
    return PID_C;
}
 
void loop() {
  /*
    int sensordValor = analogRead(sensorD);
    if (sensordValor < 1000) {
        if (millis() - tempoAnterior > intervaloLeitura) { // verifica se já passou o intervalo desde a última leitura
            contadorD++;
            tempoAnterior = millis(); 
        }
            if(contadorD >= 3){
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, HIGH);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, HIGH);
        analogWrite(PINO_ENA, 20);
        analogWrite(PINO_ENB, 20);
      //  digitalWrite(standby, HIGH);
        delay(400);
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, LOW);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, LOW);
        digitalWrite(PINO_ENA, LOW);
        digitalWrite(PINO_ENB, LOW);
              
    }
  //          tempoAnterior = millis(); // atualiza o tempo da última leitura
        }*/
        
  analogWrite(PINO_ENA, mediaD);
  analogWrite(PINO_ENB, mediaE);
  
  int sensordValor = analogRead(sensorD);
  int sensoreValor = analogRead(sensorE);
  
  if (sensordValor < 1000) {
    if (millis() - tempoAnterior > intervaloLeitura) { // verifica se já passou o intervalo desde a última leitura
      contadorD++;
      Serial.println(sensordValor);
      Serial.println(contadorD);
      tempoAnterior = millis(); // atualiza o tempo da última leitura
    }
  }
  if (contadorD >= 2) {
    if (millis() - tempoAnterior > intervaloLeitura) { // verifica se já passou o intervalo desde a última leitura      
   // i = 0;
    digitalWrite(PINO_IN1, LOW); 
    digitalWrite(PINO_IN2, LOW);
    digitalWrite(PINO_IN3, LOW);
    digitalWrite(PINO_IN4, LOW);
    digitalWrite(PINO_ENA, LOW);
    digitalWrite(PINO_ENB, LOW);
  }
  }

  analogWrite(PINO_ENA, mediaD);
  analogWrite(PINO_ENB, mediaE);

      float contadorSF = 0;
    for (int i = 0; i < numSensores; i++) {
        contadorSF = 0;
        for (int m = 0; m < MediaC; m++) {
            contadorSF += analogRead(SensorFrontal[i]);
        }
        SensorF[i] = contadorSF / MediaC;     
    }
 
 
    // Coloque as funções na ordem correta para o funcionamento correto do PID e do controle dos motores
    //calculaMediaSensores();
    retornaErro();
    //sensorDireito();
    float PID_C = CalculoPID(erro);
 
    
    // Com os resultados obtidos até aqui, ajusta a velocidade e usa ifs para curvas
    if (PID_C < 0) { // Curva direita
        VelE = mediaE;
        VelD = abs((mediaD - PID_C * 100));
        digitalWrite(PINO_IN1, LOW); //Motor 1
        digitalWrite(PINO_IN2, LOW);
        digitalWrite(PINO_IN3, LOW); // Motor 2
        digitalWrite(PINO_IN4, HIGH);
        analogWrite(PINO_ENA, VelD);
        analogWrite(PINO_ENB, VelE);
        digitalWrite(standby, HIGH);
       // delay(9);
       /* Serial.print("Vel Esquerda: ");
        Serial.println(VelE);
        Serial.print("Vel Direita: ");
        Serial.println(VelD);
        Serial.println("Direita (>0)");*/
      //  delay(8);
    }
    else if (PID_C > 0) { // Curva esquerda
        VelE = abs((mediaE - PID_C * 100));
        VelD = mediaD;
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, HIGH);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, LOW);
      //  Serial.print("Vel Direita:");
     //   Serial.println(VelD);
        analogWrite(PINO_ENA, VelD);
      //  Serial.print("Vel Esquerda:");
      //  Serial.println(VelE);
        analogWrite(PINO_ENB, VelE);
        digitalWrite(standby, HIGH);
     //   Serial.println("Esquerda (<0)");
        //delay(8);
       // delay(9);
    }
     if(erro == 0){ //retas
        digitalWrite(PINO_IN1, LOW); 
        digitalWrite(PINO_IN2, HIGH);
        digitalWrite(PINO_IN3, LOW);
        digitalWrite(PINO_IN4, HIGH);
        analogWrite(PINO_ENA, mediaD);
        analogWrite(PINO_ENB, mediaE);
        digitalWrite(standby, HIGH);
   //     Serial.println("Reta");
        //delay(2);
    }
}

/*
void sensorDireito() {
    int sensordValor = analogRead(sensorD);
    if (sensordValor < 1000) {
        if (millis() - tempoAnterior > intervaloLeitura) { // verifica se já passou o intervalo desde a última leitura
            contadorD++;
         //   Serial.println(sensordValor);
           // Serial.println(contadorD);
            tempoAnterior = millis(); // atualiza o tempo da última leitura
        }
        if (contadorD >= 3) {
            mediaE = 0;
            mediaD = 0;
            digitalWrite(PINO_IN1, LOW); 
            digitalWrite(PINO_IN2, LOW);
            digitalWrite(PINO_IN3, LOW);
            digitalWrite(PINO_IN4, LOW);
            digitalWrite(PINO_ENA, 0);
            digitalWrite(PINO_ENB, 0);
        }
    }
}
    }}
void sensorEsquerdo() {
    int sensoreValor = analogRead(sensorE);
    if (sensoreValor < 1000) {
        if (millis() - tempoAnteriorE > intervaloLeitura) { // Verifica se já passou o intervalo desde a última leitura
            contadorE++;
            Serial.print("Sensor E valor: ");
            Serial.println(sensoreValor);
            Serial.print("Contador E: ");
            Serial.println(contadorE);
            tempoAnteriorE = millis(); // Atualiza o tempo da última leitura
        }
        if (contadorE % 2 != 0) {
             mediaE = 70;
             mediaD = 70;
             delay(11);
        } 
        if (contadorE == 0) {
             mediaE = 70;
             mediaD = 70;
             delay(11);
        }/*
        else {
            pwm = 100; // PWM para valores ímpares
        }*/
//}
///}
