#define s0       A7
#define s1       A6
#define s2       A5
#define s3       A4
#define s4       A3
#define s5       A2
#define s6       A1
#define s7       A0
#define LED_ON   13
#define LED       3
#define BOTON     2
#define PINBUZZER 12

//pines motores
#define  pwma  5
#define  der1  6
#define  der2  7
#define  pwmb  11
#define  izq1  10
#define  izq2  9 
#define  STBY  8

//int umbral = 120;
int digital[8];
int sensores[8];
int lectura_fondo[8];
int lectura_linea[8];
int umbral[8];

long int sumap, suma, pos, poslast, position;

//////////////////////PID////////////////////////
const float KP = 0.11;
const float KD = 1;
const float KI = 0.0055;
const float KV = 6.5;

int vmin = 130;
int vmax = 170;
int veladelante = 180;
int velatras = 10;
int vbase;
/////////////////////////////////////////////////

//////////DATOS PARA LA INTEGRAL///////////////
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;
/////////////////////////////////////////////////

/////////////////VARIABLES PID///////////////////
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop;
int setpoint = 350;
/////////////////////////////////////////////////
void setup() {

  pinMode(der1, OUTPUT);
  pinMode(der2, OUTPUT);
  pinMode(izq1, OUTPUT);
  pinMode(izq2, OUTPUT);

 //Serial.begin(9600);
  pinMode(s0, INPUT);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED_ON, OUTPUT);
  pinMode(BOTON, INPUT);

  digitalWrite(LED_ON, HIGH);

  digitalWrite(LED, 1);
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  for(int i = 0; i < 50; i++){
    fondos();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  for(int i = 0; i < 50; i++){
    lineas();
    digitalWrite(LED, 0);
    delay(20);
    digitalWrite(LED, 1);
    delay(20);
  }
  promedio();
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  while (digitalRead(BOTON));
  tone(PINBUZZER, 2000, 100);
  delay(200); 
  digitalWrite(LED, 0);
  delay(1000);
}

void loop() {
  //position = lectura();
//Serial.println(position);
  digitalWrite(STBY,HIGH);
  lectura(); 
  PID();
  inT(); 
  frenos();
}

void fondos(){
  lectura_fondo[0] = analogRead(s0);
  lectura_fondo[1] = analogRead(s1);
  lectura_fondo[2] = analogRead(s2);
  lectura_fondo[3] = analogRead(s3);
  lectura_fondo[4] = analogRead(s4);
  lectura_fondo[5] = analogRead(s5);
  lectura_fondo[6] = analogRead(s6);
  lectura_fondo[7] = analogRead(s7);
}

void lineas(){
  lectura_linea[0] = analogRead(s0);
  lectura_linea[1] = analogRead(s1);
  lectura_linea[2] = analogRead(s2);
  lectura_linea[3] = analogRead(s3);
  lectura_linea[4] = analogRead(s4);
  lectura_linea[5] = analogRead(s5);
  lectura_linea[6] = analogRead(s6);
  lectura_linea[7] = analogRead(s7);
}

void promedio(){
  for(int i = 0; i < 8; i++){
    umbral[i]= (lectura_fondo[i]+ lectura_linea[i])/2;
  }
}

void inT(){
  // giro derecha: los 3 sensores derechos ven línea, el resto no
  if (digital[0]==1 && digital[1]==1 && digital[2]==1 &&
      digital[3]==0 && digital[4]==0 && digital[5]==0 &&
      digital[6]==0 && digital[7]==0){
    digitalWrite(STBY, HIGH);
    motores(veladelante, -velatras);
  }
  // giro izquierda: los 3 sensores izquierdos ven línea, el resto no
  else if (digital[5]==1 && digital[6]==1 && digital[7]==1 &&
           digital[0]==0 && digital[1]==0 && digital[2]==0 &&
           digital[3]==0 && digital[4]==0){
    digitalWrite(STBY, HIGH);
    motores(-velatras, veladelante);
  }
}

int lectura(void){
  sensores[0] = analogRead(s0);
  sensores[1] = analogRead(s1);
  sensores[2] = analogRead(s2);
  sensores[3] = analogRead(s3);
  sensores[4] = analogRead(s4);
  sensores[5] = analogRead(s5);
  sensores[6] = analogRead(s6);
  sensores[7] = analogRead(s7);
  
  for(int i = 0; i < 8; i++){
    if (sensores[i]<= umbral[i]){
      digital[i] = 0;
    }
    else{digital[i] =1;}
    //Serial.print(digital[i]);
    //Serial.print("\t");
  }
  //Serial.println(" ");

  sumap = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);  
  suma = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);  
  pos = (sumap / suma);

  if(poslast <= 100 && pos == -1){
    pos = 0;
  }
  if(poslast >= 600 && pos == -1){
    pos = 700;
  }
  poslast = pos;
  
  return pos;
}

void PID(){
  proporcional = pos - setpoint;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

  if((proporcional*integral)<0) integral = 0;
  diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
  
  vbase = vmin+(vmax-vmin)*exp(-KV*abs(KP*proporcional));
  motores(vbase+diferencial, vbase-diferencial);
}

void frenos(){
  if (pos <= 100){
    digitalWrite(STBY,HIGH);
    motores(veladelante, -velatras);
  }
  if (pos >= 600){
    digitalWrite(STBY,HIGH);
    motores(-velatras, veladelante);
  }
}
void motores(int izq, int der){
  
  izq = constrain(izq, -255, 255);
  der = constrain(der, -255, 255);

  digitalWrite(izq1, izq<0);
  digitalWrite(izq2, izq>=0); 
  analogWrite(pwma, abs(izq));

  digitalWrite(der1, der<0);
  digitalWrite(der2, der>=0);
  analogWrite(pwmb, abs(der));
  
}