// Lead me through Ameri, onwards to Stellaris' doors.

// Pins
// Multiplexer
const int OP = A3;  // A3 conectado a OP    SALIDA DEL MULTIPLEXOR
const int S0 = A0;  // A0 conectado a S0     S0 DEL MULTIPLEXOR
const int S1 = A1;  // A1 conectado a S1     S1 DEL MULTIPLEXOR
const int S2 = A2;  // A2 conectado a S2     S2 DEL MULTIPLEXOR
// // Motores
const int pini = 9;
const int pwmi = 6;
const int pind = 10;
const int pwmd = 5;
// // Otros
const int led = 13;  // LED del Arduino Nano
const int boton = 7;  // Boton 
#define BUZZER_PIN 3

// Variables de crucero
int velocidad = 56;  // Velocidad del vehiculo (0/255)
float kp = 0.60;  // Proporcional
float kd = 11.6;    //  Derivada
int vel_forw = 100;   // Frenos - Adelante
int vel_rev = 65;  // Frenos - Reversa
int color = 1;  // 1-2 (Negra-Blanca)

// Sensores
const int NUM_SENSORES = 8;
int mins[NUM_SENSORES];
int maxs[NUM_SENSORES];
int valorSensor[NUM_SENSORES];
bool calibrando = true;
int sensibilidad = 100, posicion = 0, modo = LOW;
double error_pos = 0, error_ult = 0, correccion = 0;
unsigned long ahora = 0, pasado = 0;

// Decisiones y eventos
const int MAX_DECISIONES = 1;  // Ajustar el número de eventos en pista antes de competir
// 0 = recto, 1 = derecha, 2 = izquierda
byte decisiones[MAX_DECISIONES] = { 0 };
int indiceDecision = 0;
enum TipoEvento {
  EVENTO_NINGUNO,
  EVENTO_BIFURCACION,
  EVENTO_BIFURCACION_T,
  EVENTO_CRUCE_DERECHA,
  EVENTO_CRUCE_IZQUIERDA
};
TipoEvento eventoActual = EVENTO_NINGUNO;

void detectarEvento() {
  int cuentaActivos = 0;
  // Conteo limpio usando el arreglo
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (valorSensor[i] > sensibilidad) cuentaActivos++;
  }
  // Patrones posicionales usando índices (0 al 7)
  bool extremosActivos = (valorSensor[0] > sensibilidad && valorSensor[7] > sensibilidad);
  bool crucetipoT = (valorSensor[1] > sensibilidad && valorSensor[2] > sensibilidad && 
                     valorSensor[3] > sensibilidad && valorSensor[4] > sensibilidad && 
                     valorSensor[5] > sensibilidad && valorSensor[6] > sensibilidad);              
  bool cruceIzquierda = (valorSensor[0] > sensibilidad && valorSensor[1] > sensibilidad && 
                         valorSensor[2] > sensibilidad && valorSensor[3] > sensibilidad && 
                         valorSensor[4] < sensibilidad && valorSensor[5] < sensibilidad && 
                         valorSensor[6] < sensibilidad && valorSensor[7] < sensibilidad);
  bool cruceDerecha = (valorSensor[0] < sensibilidad && valorSensor[1] < sensibilidad && 
                       valorSensor[2] < sensibilidad && valorSensor[3] < sensibilidad && 
                       valorSensor[4] > sensibilidad && valorSensor[5] > sensibilidad && 
                       valorSensor[6] > sensibilidad && valorSensor[7] > sensibilidad);
  // Reset evento
  eventoActual = EVENTO_NINGUNO;
  digitalWrite(BUZZER_PIN, LOW);
  // NOTA: Se prioriza cuentaActivos >= 6 para detectar cruces tipo T anchos
  if (crucetipoT || cuentaActivos >= 6) {
    digitalWrite(BUZZER_PIN, HIGH);
    eventoActual = EVENTO_BIFURCACION_T;
  } else if (extremosActivos) {
    digitalWrite(BUZZER_PIN, HIGH);
    eventoActual = EVENTO_BIFURCACION;
  } else if (cruceIzquierda && cuentaActivos <= 4) {
    digitalWrite(BUZZER_PIN, HIGH);
    eventoActual = EVENTO_CRUCE_IZQUIERDA;
  } else if (cruceDerecha && cuentaActivos <= 4) {
    digitalWrite(BUZZER_PIN, HIGH);
    eventoActual = EVENTO_CRUCE_DERECHA;
  }
}

void parpadearLED() {
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
}

void esperarBoton() {
  while (digitalRead(boton) == HIGH) {
  }
  while (digitalRead(boton) == LOW) {
  }
  delay(50); 
}

int leerAnalogico(int indice) {
  digitalWrite(S0, indice & 0x01);
  digitalWrite(S1, (indice >> 1) & 0x01);
  digitalWrite(S2, (indice >> 2) & 0x01);
  int val = analogRead(OP) / 4;
  if (color == 2) val = map(val, 0, 255, 255, 0);
  return val;
}

void calibrarSensores() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    mins[i] = 255;
    maxs[i] = 0;
  }
  unsigned long antes = millis();
  const long pausa = 70;
  int modo = LOW;
  while (calibrando) {
    if (digitalRead(boton) == LOW) {
      calibrando = false;
    }
    unsigned long presente = millis();
    if (presente - antes > pausa) {
      antes = presente;
      modo = (modo == LOW) ? HIGH : LOW;
      digitalWrite(led, modo);
      digitalWrite(BUZZER_PIN, modo);
    }
    for (int i = 0; i < NUM_SENSORES; i++) {
    int crudo = leerAnalogico(i);
    if (crudo < mins[i]) mins[i] = crudo;
    if (crudo > maxs[i]) maxs[i] = crudo;
  }
  }
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(led, LOW);
  delay(500);
}

void lectura() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    // 1. Configurar los pines S0, S1 y S2 contando en binario (0 al 7)
    digitalWrite(S0, i & 0x01);
    digitalWrite(S1, (i >> 1) & 0x01);
    digitalWrite(S2, (i >> 2) & 0x01);

    // 2. Leer la salida del multiplexor y bajar la resolución (0-1023 a 0-255)
    valorSensor[i] = analogRead(OP) / 4;

    // 3. Invertir los valores si es línea blanca sobre fondo negro
    if (color == 2) {
      valorSensor[i] = map(valorSensor[i], 0, 255, 255, 0);
    }

    // 4. Normalizar el valor usando los mínimos y máximos calibrados
    valorSensor[i] = map(valorSensor[i], mins[i], maxs[i], 0, 255);
    
    // 5. Clampeo de seguridad: map() no corta los excesos automáticamente
    valorSensor[i] = constrain(valorSensor[i], 0, 255);
  }
}

void rutinaPD() {
  lectura();

  // 1. Cálculo de error posicional normalizado
  long sumap = 0, suma = 0;
  int pesos[NUM_SENSORES] = {-100, -50, -30, -10, 10, 30, 50, 100};
  
  for (int i = 0; i < NUM_SENSORES; i++) {
    sumap += pesos[i] * valorSensor[i];
    suma  += valorSensor[i];
  }
  
  // Si no hay línea, mantiene el último error para no volverse loco
  error_pos = (suma > 0) ? (double)sumap / suma : error_ult; 

  // 2. Control PD con tiempo real (dt)
  ahora = millis();
  double dt = (ahora - pasado) / 1000.0; // Tiempo transcurrido en segundos
  if (dt <= 0.0) dt = 0.001; // Prevención de división por cero por iteraciones ultrarrápidas
  
  double error_d = (error_pos - error_ult) / dt; // Derivada real[cite: 1]
  float P = kp * error_pos;
  float D = kd * error_d;

  correccion = constrain(P + D, -255, 255); // Clampeo directo
  
  pasado = ahora;
  error_ult = error_pos;

  int pwmDerecha = velocidad + correccion;
  int pwmIzquierda = velocidad - correccion;

  // 3. Actualización de posición extrema
  if (valorSensor[1] > sensibilidad) posicion = 0;
  if (valorSensor[0] > sensibilidad && valorSensor[3] < sensibilidad && valorSensor[4] < sensibilidad) posicion = 1;
  if (valorSensor[5] > sensibilidad) posicion = 0;
  if (valorSensor[7] > sensibilidad && valorSensor[3] < sensibilidad && valorSensor[4] < sensibilidad) posicion = 16;

  // 4. Manejo de Eventos y Decisiones
  // detectarEvento(); // Por ahora comentado

  if (eventoActual != EVENTO_NINGUNO) {
    controlarMotores(0, 0); // Freno breve limpio
    delay(30);

    if (indiceDecision >= MAX_DECISIONES) {
      controlarMotores(-vel_rev, -vel_rev); // Frenado de motor final
      delay(50);
      controlarMotores(0, 0);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW);
      while (true) {} // Meta
    }

    byte accion = decisiones[indiceDecision];

    switch (eventoActual) {
      case EVENTO_BIFURCACION:
        if (accion == 1) { // Izquierda
          controlarMotores(-vel_rev, -vel_rev);
          delay(50); controlarMotores(0, 0); delay(10);
          posicion = 1; delay(150);
        }  
        if (accion == 2) { // Derecha
          controlarMotores(-vel_rev, -vel_rev);
          delay(50); controlarMotores(0, 0); delay(10);
          posicion = 16; delay(150);
        }  
        break;

      case EVENTO_BIFURCACION_T:
        if (accion == 2) { // Izquierda
          while (true) {
            controlarMotores(vel_forw, -vel_rev);
            lectura(); delay(10);
            if (valorSensor[7] > sensibilidad && valorSensor[1] < sensibilidad && 
                valorSensor[2] < sensibilidad && valorSensor[3] < sensibilidad && 
                valorSensor[4] < sensibilidad && valorSensor[5] < sensibilidad && 
                valorSensor[6] < sensibilidad && valorSensor[0] < sensibilidad) break;
          }
        }  
        if (accion == 1) { // Derecha
          while (true) {
            controlarMotores(-vel_rev, vel_forw);
            lectura(); delay(10);
            if (valorSensor[0] > sensibilidad && valorSensor[1] < sensibilidad && 
                valorSensor[2] < sensibilidad && valorSensor[3] < sensibilidad && 
                valorSensor[4] < sensibilidad && valorSensor[5] < sensibilidad && 
                valorSensor[6] < sensibilidad && valorSensor[7] < sensibilidad) break;
          }
        }  
        break;

      case EVENTO_CRUCE_DERECHA:
        if (accion == 0) posicion = 0;  
        if (accion == 1) { pwmDerecha = 0; pwmIzquierda = 0; }  
        if (accion == 2) { controlarMotores(0, 0); delay(500); }  
        break;

      case EVENTO_CRUCE_IZQUIERDA:
        if (accion == 0 || accion == 1 || accion == 2) { pwmDerecha = 0; pwmIzquierda = 0; }  
        break;
        
      default:
        break;
    }

    delay(10);
    indiceDecision++;
    eventoActual = EVENTO_NINGUNO;
  }

  // 5. Control Físico de Motores (Recuperación vs Línea)
  if (posicion == 16) {
    // Si salió por la derecha, frena la derecha en reversa y empuja la izquierda
    controlarMotores(vel_forw, -vel_rev);
  } else if (posicion == 1) {
    // Si salió por la izquierda, frena la izquierda en reversa y empuja la derecha
    controlarMotores(-vel_rev, vel_forw);
  } else {
    // Flujo del PD normal
    controlarMotores(pwmIzquierda, pwmDerecha);
  }
}

void controlarMotores(int velIzq, int velDer) {
  // Clampeo seguro de -255 a 255
  velIzq = constrain(velIzq, -255, 255);
  velDer = constrain(velDer, -255, 255);

  // Motor Izquierdo (Adelante = LOW, Atrás = HIGH)
  if (velIzq >= 0) {
    digitalWrite(pini, LOW);
    analogWrite(pwmi, velIzq);
  } else {
    digitalWrite(pini, HIGH);
    analogWrite(pwmi, abs(velIzq));
  }

  // Motor Derecho (Adelante = HIGH, Atrás = LOW) - ¡Respeta la inversión de hardware original!
  if (velDer >= 0) {
    digitalWrite(pind, HIGH);
    analogWrite(pwmd, velDer);
  } else {
    digitalWrite(pind, LOW);
    analogWrite(pwmd, abs(velDer));
  }
}

void setup() {
  // Configuración del Multiplexor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(OP, INPUT);

  // Configuración de Motores
  pinMode(pini, OUTPUT);
  pinMode(pwmi, OUTPUT);
  pinMode(pind, OUTPUT);
  pinMode(pwmd, OUTPUT);

  // Configuración de Periféricos
  pinMode(led, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(boton, INPUT_PULLUP); // Asegura el estado estable del botón

  // --- SECUENCIA DE ARRANGUE ---
  parpadearLED();       // 1. Destello inicial de encendido
  esperarBoton();       // 2. Espera a que presiones y sueltes para CALIBRAR
  calibrarSensores();   // 3. Pasa el robot por la línea blanca y negra para calibrar
  esperarBoton();       // 4. Espera de nuevo a que presiones y sueltes para ARRANCAR la carrera
}

void loop() {
  rutinaPD();
}

