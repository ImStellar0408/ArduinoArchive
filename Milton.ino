#include <QTRSensors.h>

// --- PINES DEL DRIVER TB6612FNG ---
const int PIN_PWMA = 11;
const int PIN_AIN1 = 10;
const int PIN_AIN2 = 9;

const int PIN_PWMB = 5;
const int PIN_BIN1 = 6;
const int PIN_BIN2 = 7;

const int PIN_STBY = 8;

// --- PINES DE PERIFÉRICOS Y EMISORES ---
const int PIN_LEDON  = 13;
const int PIN_BOTON  = 2;
const int PIN_LED    = 3;
const int PIN_BUZZER = 12;

// --- CONFIGURACIÓN QTR-8A ---
QTRSensors qtr;
const uint8_t NUM_SENSORES = 8;
unsigned int sensorValues[NUM_SENSORES];

// --- PARÁMETROS PID Y VELOCIDAD ---
float Kp = 0.20;   
float Ki = 0.00;    
float Kd = 3.40;   

int velMax = 210;        // Velocidad en recta
int ultimaDireccion = 1; // 1 = Derecha, -1 = Izquierda
int errorAnterior = 0;

// --- PROTOTIPOS DE FUNCIONES ---
void moverMotorIzq(int velocidad);
void moverMotorDer(int velocidad);
void moraBoton();

void setup() {
  pinMode(PIN_LEDON, OUTPUT);
  digitalWrite(PIN_LEDON, HIGH);

  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT); 
  digitalWrite(PIN_STBY, HIGH);

  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Overclock del ADC (Frecuencia de muestreo rápida)
  ADCSRA = (ADCSRA & 0xF8) | 0x04; 

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORES);

  // Pitido de inicialización
  tone(PIN_BUZZER, 1000, 100);
  digitalWrite(PIN_LED, HIGH);
  delay(200);
  digitalWrite(PIN_LED, LOW);

  // Primer pulso para iniciar calibración
  moraBoton();

  digitalWrite(PIN_LED, HIGH);
  tone(PIN_BUZZER, 1500, 200);
  delay(300);

  // Calibración manual
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }

  digitalWrite(PIN_LED, LOW);
  tone(PIN_BUZZER, 2000, 400);

  // Segundo pulso para arrancar la marcha
  moraBoton();

  // Conteo regresivo
  for (int i = 0; i < 3; i++) {
    tone(PIN_BUZZER, 800, 100);
    digitalWrite(PIN_LED, HIGH);
    delay(400);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  tone(PIN_BUZZER, 2200, 400);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  uint16_t posicion = qtr.readLineWhite(sensorValues);
  int error = posicion - 3500;

  // --- 1. REGISTRO RIGUROSO DE ÚLTIMA DIRECCIÓN ---
  if (error > 1800) {
    ultimaDireccion = 1;  // Tendencia a la Derecha
  } else if (error < -1800) {
    ultimaDireccion = -1; // Tendencia a la Izquierda
  }

  // --- 2. DETECCIÓN DE PÉRDIDA TOTAL DE LÍNEA (BLANCO TOTAL) ---
  bool lineaVisible = false;
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (sensorValues[i] > 180) { // Detección de negro
      lineaVisible = true;
      break;
    }
  }

  if (!lineaVisible) {
    // A) Freno seco para cortar inercia (40 ms)
    moverMotorIzq(-100);
    moverMotorDer(-100);
    delay(40);

    // B) Reversa UNIFORME para reenganchar la cinta en línea recta
    unsigned long tiempoInicio = millis();
    
    while (!lineaVisible) {
      // Reversa recta y pareja en ambos motores
      moverMotorIzq(-160);
      moverMotorDer(-160);

      // Vuelve a leer sensores en búsqueda de negro
      qtr.readLineWhite(sensorValues);
      for (uint8_t i = 0; i < NUM_SENSORES; i++) {
        if (sensorValues[i] > 200) {
          lineaVisible = true;
          break;
        }
      }

      // Timeout de seguridad de 1 segundo
      if (millis() - tiempoInicio > 1000) {
        moverMotorIzq(0);
        moverMotorDer(0);
        return;
      }
    }

    return; // Sale del bucle al recuperar la línea
  }

  // --- 3. CONTROL PID NORMAL EN PISTA ---
  int absError = abs(error);
  
  // Reducción dinámica de velocidad según el error
  int velActual = velMax - (absError * 0.035); 
  if (velActual < 70) velActual = 70;

  float P = error;
  float D = error - errorAnterior;
  errorAnterior = error;

  float correccion = (Kp * P) + (Kd * D);

  int motorIzq = velActual + correccion;
  int motorDer = velActual - correccion;

  moverMotorIzq(motorIzq);
  moverMotorDer(motorDer);
}

// =========================================================
// FUNCIONES DE CONTROL DE MOTORES Y BOTÓN
// =========================================================

void moverMotorIzq(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad >= 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, velocidad);
  } else {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, abs(velocidad));
  }
}

void moverMotorDer(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad >= 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, velocidad);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, abs(velocidad));
  }
}

void moraBoton() {
  // Espera a que presiones el botón (lea LOW)
  while (digitalRead(PIN_BOTON) == HIGH) {
    delay(10);
  }
  
  delay(50); // Antirrebote al presionar

  // Espera a que sueltes el botón (vuelva a HIGH) con timeout
  unsigned long tiempoEspera = millis();
  while (digitalRead(PIN_BOTON) == LOW) {
    if (millis() - tiempoEspera > 1500) {
      break; // Salida forzada por timeout para no congelar la placa
    }
    delay(10);
  }
  
  delay(100); // Antirrebote al soltar
}