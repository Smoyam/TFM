// DEFINO LA PREESCALA DE LOS TIMER 1 Y 2
#define PREESCALA_1 8
#define PREESCALA_2 256

// TIEMPO DE PULSO DE CADA TIMER
float T_PULS_1=0.0000000625*PREESCALA_1; //T=(1/16000000)∗65535∗Escala
float T_PULS_2=0.0000159375*PREESCALA_2; //T=(1/16000000)∗255∗Escala

// DEFINO FUNCIONES PARA ACTIVAR O DESACTIVAR PINES
#define CLR(x, y) (x&=(~(1<<y)))
#define CLRR(x, y, z) (x&=(~((1<<y)|(1<<z))))
#define SET(x, y) (x|=(1<<y))
#define SETT(x, y, z) (x|=(1<<y)|(1<<z) )
#define SETTT(x, y, z, v) (x|=(1<<y)|(1<<z) |(1<<v) )

// VARIABLES GLOBALES
volatile float alfa =0; // VARIABLE QUE ALMACENA LA ACELERACIÓN QUE RECIBE DE LA RPI
volatile float vec[2]={0,0}; // VECTOR QUE ALMACENA LOS VALORES DE LA VELOCIDAD
float alfa_M1=0; // VARIABLE QUE ALMACENA LA ACELERACIÓN A APLICAR, DESPUÉS DE FILTRAR SI ESTA ENTRE LOS LIMITES ESTABLECIDOS EN +− xx RAD/S^2. EVITANDO ASI EL OVF AL ENVIARSE UN DATO CORRUPTO
float flag =0; // BANDERA QUE SE PONE A CERO CUANDO SE RECIBE UN DATO alfa DESDE LA RPI, UNA VEZ PROCESADO ESTE DATO SE PONE A UNO A LA ESPERA DE RECIBIR OTRO DATO
int comprueba;
int j = 0;
float vel; // VALOR DE LA VELOCIDAD MEDIA QUE LE ENVÍO A LA PI
float vel_M1, vel_M2;

// DECLARACION PINES MOTOR 1
#define STEP_MOTOR_1 1 // PORTB D9
#define DIR_MOTOR_1 0 // PORTB D8
int MS1_MOTOR_1 = 8 - 8; //PORTC A0
int MS2_MOTOR_1 = 9 - 8; // PORTC A1

// DECLARACION PINES MOTOR 2
#define STEP_MOTOR_2 6 // PORTD D7
#define DIR_MOTOR_2 7 // PORTD D6
int MS1_MOTOR_2 = 11 - 8; // PORTD D3
int MS2_MOTOR_2 = 12 - 8; // PORTD D4

// DECLARACIÓN DE LA CANTIDAD DE PASOS POR REVOLUCIÓN DEPENDIENDO DE LA CONFIGURACIÓN
uint16_t npuls_min[]={700, 600, 600, 600};
uint16_t npuls_max[]={65535, 65535, 65535, 65535};

// NUMERO MAX Y MIN DE PASOS/SEGUNDO
float f_timer_min []={1/( npuls_max[0]*T_PULS_1),1/(npuls_max[1]*T_PULS_1),1/(npuls_max[2]*T_PULS_1),1/(npuls_max[3]*T_PULS_1)};
float f_timer_max[]={1/(npuls_min[0]*T_PULS_1),1/(npuls_min[1]*T_PULS_1),1/(npuls_min[2]*T_PULS_1),1/(npuls_min[3]*T_PULS_1)};

// NUMERO DE PASOS PARA DAR UNA VUELTA Y AVANCE DE CADA PASO
float avance_grados [4]={1.8,0.9,0.145,0.225};
float pasos []={200,400,800,1600};

// FRECUENCIAS MAX Y MINIMAS, Nº MAX Y MINIMOS DE VUELTAS POR SEGUNDO. EXPERIMENTALMENTE LA VEL MAX 3000º/S
float f_motor_max[]={f_timer_max[0]/pasos[0],f_timer_max[1]/pasos [1], f_timer_max[2]/pasos [2],f_timer_max[3]/pasos [3]};
float f_motor_min[]={f_timer_min[0]/pasos [0], f_timer_min [1]/ pasos [1], f_timer_min [2]/ pasos [2],f_timer_min [3]/ pasos [3]};

// VELOCIDAD MAX Y MIN DEL MOTOR EN RAD/S.. W=2∗PI∗f
float omega_motor_max[]={2*PI*f_motor_max[0],2*PI*f_motor_max[1],2*PI*f_motor_max[2],2*PI*f_motor_max[3]};
float omega_motor_min[]={2*PI*f_motor_min[0],2*PI*f_motor_min[1],2*PI*f_motor_min[2],2*PI*f_motor_min[3]};

// NUMERO DE PULSOS DE CADA INTERRUPCION DEL TIMER 1
uint16_t npuls_M1=65535, npuls_M2=65535;

// DIRECCIÓN DE CADA MOTOR
int dir_M1=0, dir_M2=0;

// TRADUCE DE º/S A PULSOS/S
float omega2npuls[]={omega_motor_max[0]/(T_PULS_1*f_motor_max[0]*pasos[0]),omega_motor_max[1]/(T_PULS_1*f_motor_max[1]*pasos[1]),omega_motor_max[2]/(T_PULS_1*f_motor_max[2]*pasos[2]),omega_motor_max[3]/(T_PULS_1*f_motor_max[3]*pasos[3])};

// CONEXIÓN BLUETOOTH
float giro_M1=0, giro_M2;

// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

float TAU = 0.99;

float phi, dphi;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

// angulo
float angulo = 0;

// Aceleracion de las ruedas
float aceleracion = 0;

// Variables para la comunicacion serie
#pragma pack(push, 1)
struct miEstructura {
  uint16_t angByte;
  uint16_t gxByte;
};
#pragma pack(pop)

uint16_t vec1Byte;
uint16_t vec2Byte;

static miEstructura datos;

#define M_PI 3.1416

void setup() {
  Serial.begin(115200);

  // CONFIGURACIÓN MOTOR 1
  pinMode(MS1_MOTOR_1 + 8 , OUTPUT); //MS1
  pinMode(MS2_MOTOR_1 + 8, OUTPUT); //MS2
  pinMode(STEP_MOTOR_1 , OUTPUT); //STEP
  pinMode(DIR_MOTOR_1, OUTPUT); //DIR

  // CONFIGURACIÓN DEL MOTOR 2
  pinMode(MS1_MOTOR_2 + 8 , OUTPUT); //MS1
  pinMode(MS2_MOTOR_2 + 8 , OUTPUT); //MS2
  pinMode(STEP_MOTOR_2, OUTPUT); //STEP
  pinMode(DIR_MOTOR_2, OUTPUT); //DIR

  // CONFIGURACIÓN DE LAS INTERRUPCIONES DEL TIMER 1 PARA EL CONTROL DE LA VELOCIDAD
  TCCR1A=0; //INICIAMOS EN MODO NORMAL
  TCCR1B=0; //INICIAMOS EN MODO NORMAL
  switch (PREESCALA_1){
    case 1: // 4.095ms
      SET(TCCR1B,CS10);
    break;
    case 8: // 32.7675ms
      SET(TCCR1B,CS11);
    break;
    case 64: // 0.2621s
      SETT(TCCR1B,CS11,CS10);
    break;
    case 256: // 1.048s
      SETT(TCCR1B,WGM12,CS12);
    break;
    case 1024: // 4.1942
      SETT(TCCR1B,CS12,CS10);
    break;
    dir_M1=0;
    dir_M2=0;
  };

  OCR1A =65535; //ACTIVA INTERRUPCIÓN DEL TIMER 1A
  OCR1B =65535; //ACTIVA INTERRUPCION DEL TIMER 1B
  TIMSK1 =0; // LIMPIA LA MASCARA DEL TIMER
  SETT(TIMSK1,OCIE1A,OCIE1B); //ACTIVA LAS INTERRUPCIONES A Y B DEL TIMER 1

  // CONFIGURACIÓN DE LAS INTERRUPCIONES DEL TIMER 2 PARA IMPLEMENTAR LA ACELERACIÓN
  TCCR2A=0; // INICIAMOS EN MODO NORMAL
  TCCR2B=0; // INICIAMOS EN MODO NORMAL
  TCNT2=0;
  switch (PREESCALA_2){
    case 1: // 19.94us
      SET(TCCR2B,CS20);
    break;
    case 8: // 127us
      SET(TCCR2B,CS21);
    break;
    case 32: // 510us
      SETT(TCCR2B,CS21,CS20);
    break;
    case 64: // 1.02ms
      SET(TCCR2B,CS22);
    break;
    case 128: // 2.04ms
      SETT(TCCR2B,CS20,CS22);
    break;
    case 256: // 4.08ms
      SETT(TCCR2B,CS21,CS22);
    break;
    case 1024: // 16.32ms
      SETTT(TCCR2B,CS20,CS21,CS22);
    break;
  };
  OCR2A=250; // ACTIVO LA INTERRUPCIÓN DEL TIMER PARA QUE SALTE CADA 4 MS
  SET(TCCR2A,WGM21);
  SET(TIMSK2,OCIE2A); //ACTIVO LA INTERURPCIÓN A DEL TIMER 2

  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
}

void loop() {
  j++;
  
  // Envío de datos
  if (j == 1) {
    datos.angByte = float2int(ang_y);
    datos.gxByte = (uint16_t)gy;

    byte* tx = (byte*)&datos;
    for (int i = 0; i < sizeof(miEstructura); i++) {
      Serial.write(*tx++);
    }
  } else if (j == 2) {
    vec1Byte = float2int(vec[0]);

    byte* tx = (byte*)&vec1Byte;
    for (int i = 0; i < sizeof(uint16_t); i++) {
      Serial.write(*tx++);
    }
  } else if (j == 3) {
    vec2Byte = float2int(vec[1]);

    byte* tx = (byte*)&vec2Byte;
    for (int i = 0; i < sizeof(uint16_t); i++) {
      Serial.write(*tx++);
    }

    j = 0;
  }

  delay(20);
}

void serialEvent() {
  // Recepción de datos
  interrupts();
  volatile float fnum;
  volatile byte *rx = (byte*) &fnum;
  while (Serial.available()) {
    for (int i = 0; i < 4; i++) {
      *rx++= Serial.read();
    }
  }
  Serial.flush();
  alfa = (float) fnum;
  //j = 0;
  flag = 0;
}

uint16_t float2int(float valor) {
  valor = valor*100 + 32768;
  uint16_t salida = uint16_t(valor);
  return salida;
}

// INTERRUPCION A DEL TIMER 1 PARA MOVER EL MOTOR 1//
ISR(TIMER1_COMPA_vect) {
  // ACTIVA LA INTERRUPCIÓN A DEL TIMER 1 PARA DAR VELOCIDAD AL MOTOR 1
  if (dir_M1==0) // SI EL MOTOR NO SE ENCUENTRA GIRANDO EN NINGÚN SENTICO (CONDICIÓN INICIAL), EL MOTOR NO SE MOVERÁ
  return;
  SET(PORTB,STEP_MOTOR_1); //ACTIVA EL PIN STEEP
  OCR1A+=npuls_M1; // MANTIENE EL PIN EN ALTO DURANTE EL TIEMPO ESTABLECIDO POR NPULS_M1
  CLR(PORTB,STEP_MOTOR_1); //DESACTIVA EL PIN STEEP
}
// INTERRUPCION B DEL TIMER 1 PARA MOVER EL MOTOR 2//
ISR(TIMER1_COMPB_vect) {
  // FUNCIÓN ANÁLOGA A LA ANTERIOR
  if (dir_M2==0)
  return;
  SET(PORTD,STEP_MOTOR_2);
  OCR1B+=npuls_M2;
  CLR(PORTD,STEP_MOTOR_2);
}
// INTERRUPCION A DEL TIMER 2 PARA IMPLEMENTAR LA ACELERACIÓN//
// IMPLEMENTA LA VELOCIDAD ACELERADA CADA 4 MS. IMPLEMENTA UNA NUEVA ACELERACIÓN CADA XX MS QUE RECIBE DATO DE LA NUEVA ACELERAICÓN DE LA RPI
ISR(TIMER2_COMPA_vect) {
  interrupts () ; // FUNCIÓN QUE EVITA QUE EL PROGRAMA SE COLAPSE
  if (flag == 0) { // BANDERA QUE PONO A CERO CUANDO SE RECIBE UNA NUEVA ACELERACIÓN DESDE LA RPI
    flag =1; // BANDERA A UNO PARA TRABAJR CON LA ACELERACIÓN RECIBIDA Y NO VOLVER A CAMBIAR LA ACELERACIÓN HASTA QUE SE VUELVA A RECIBIR VALOR POR I2C
    comprueba=isnan(alfa ) ; // COMPRUEBA SI EL NUMERO QUE SE RECIBE ES REAL, EN CASOCONTRARIO DEVUELVE UN 1
    if ( alfa >100 || alfa <-100 || comprueba==1) { // FILTRO PARA OBVIAR LOS VALORES DE ACELERACIÓN +− 100 RAD/S^2, EVITANDO QUE SE PRODUZCA OVF AL RECIBIR DATO FUERA DE RANGO
      alfa_M1=alfa_M1;
    } else {
      alfa_M1=alfa; // EN CASO DE RECIBIR DATO DENTRO DEL RANGO, ACTUALIZO LA ACELERACIÓN QUE APLICO CON EL VALOR RECIBIDO
    }
  }

  // Serial . println (alfa_M1); // IMPRIMO LA ACELERACIÓN APLICADA PARA VER QUE LOS VALORES SON CORRECTOS
  vel_M1=vel_M1+alfa_M1*T_PULS_2+giro_M1*0.05; //ACTUALIZO LA VELOCIDAD DEL MOTOR 1 CADA T_PULS_2 MS
  vel_M2=vel_M2-alfa_M1*T_PULS_2+giro_M1*0.05; //ACTUALIZO LA VELOCIDAD 0DEL MOTOR 2 CADA T_PULS_2 MS
  vel_M1=constrain(vel_M1 ,-60,60); // SATURO LA VELOCIDAD ENTRE LOS LÍMITES ALCANZABLES
  vel_M2=constrain(vel_M2 ,-60,60); // SATURO LA VELOCIDAD ENTRE LOS LÍMITES ALCANZABLES
  comprueba_vel_1(vel_M1); //COMPRUEBO LA DIRECCIÓN DE GIRO DEL MOTOR 1 CON LA NUEVA VELOCIDAD
  comprueba_vel_2(-vel_M2); //COMPRUBEO LA DIRECCIÓN DE GIRO DEL MOTOR 2 CON LA NUEVA VELOCIDAD
  vel_M1-=giro_M1*0.05;
  vel_M2-=giro_M1*0.05;
  
  vec[0]=(vel_M1-vel_M2)/2.0; //CÁLCULO EL VALOR DE LA VELOCIDAD MEDIA QUE LE ENVÍO A LA PI

  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  gx = gx * (250.0/32768.0);
  gy = gy * (250.0/32768.0);
  gz = gz * (250.0/32768.0);

  dt = (millis() - tiempo_prev)/1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);

  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = TAU*(ang_x_prev+(gx/131)*dt) + (1-TAU)*accel_ang_x;
  ang_y = TAU*(ang_y_prev+(gy/131)*dt) + (1-TAU)*accel_ang_y;

  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}


void comprueba_vel_1(float vel) {
  if (vel > 0) {

    if (dir_M1 !=1) {
      SET(PORTB,DIR_MOTOR_1);
      dir_M1 = 1;
    }
    
    if ( vel<=omega_motor_max[3] && vel>=omega_motor_min[3]) {
      SETT(PORTC, MS1_MOTOR_1, MS2_MOTOR_1);
      npuls_M1=omega2npuls[3]/vel;
    }
    else if ( vel<=omega_motor_max[2]&& vel>=omega_motor_min[2]) {
      CLR(PORTC,MS1_MOTOR_1);
      SET(PORTC,MS2_MOTOR_1);
      npuls_M1=omega2npuls[2]/vel;
    }
    else if ( vel<=omega_motor_max[1]&& vel>=omega_motor_min[1]) {
      CLR(PORTC,MS2_MOTOR_1);
      SET(PORTC,MS1_MOTOR_1);
      npuls_M1=omega2npuls[1]/vel;
    }
    else if ( vel<=omega_motor_max[0]&& vel>=omega_motor_min[0]) {
      CLRR(PORTC,MS1_MOTOR_1,MS2_MOTOR_1);
      npuls_M1=omega2npuls[0]/vel;
    }
    else {
      SETT(PORTC,MS1_MOTOR_1,MS2_MOTOR_1);
      npuls_M1=npuls_max[3];
    }
  }
  if (vel < 0) {
    if (dir_M1 !=-1) {
      CLR(PORTB,DIR_MOTOR_1);
      dir_M1=-1;
    }
    
    if (-vel<=omega_motor_max[3] && -vel>=omega_motor_min[3]) {
      SETT(PORTC, MS1_MOTOR_1, MS2_MOTOR_1);
      npuls_M1=-omega2npuls[3]/vel;
    }
    else if (-vel<=omega_motor_max[2]&& -vel>=omega_motor_min[2]) {
      CLR(PORTC,MS1_MOTOR_1);
      SET(PORTC,MS2_MOTOR_1);
      npuls_M1=-omega2npuls[2]/vel;
    }
    else if (-vel<=omega_motor_max[1]&& -vel>=omega_motor_min[1]) {
      CLR(PORTC,MS2_MOTOR_1);
      SET(PORTC,MS1_MOTOR_1);
      npuls_M1=-omega2npuls[1]/vel;
    }
    else if (-vel<=omega_motor_max[0]&& -vel>=omega_motor_min[0]) {
      CLRR(PORTC,MS1_MOTOR_1,MS2_MOTOR_1);
      npuls_M1=-omega2npuls[0]/vel;
    }
    else {
      SETT(PORTC,MS1_MOTOR_1,MS2_MOTOR_1);
      npuls_M1=npuls_max[3];
    }
  }
}

// FUNCIÓNA ANALOGA A LA ANTERIOR PARA EL MOTOR 2
void comprueba_vel_2(float vel ) {
  if (vel > 0) {
    if (dir_M2 !=1) {
      SET(PORTD,DIR_MOTOR_2);
      dir_M2=1;
    }

    if ( vel<=omega_motor_max[3] && vel>=omega_motor_min[3]) {
      SETT(PORTD, MS1_MOTOR_2, MS2_MOTOR_2);
      npuls_M2=omega2npuls[3]/vel;
    }
    else if ( vel<=omega_motor_max[2]&& vel>=omega_motor_min[2]) {
      CLR(PORTD,MS1_MOTOR_2);
      SET(PORTD,MS2_MOTOR_2);
      npuls_M2=omega2npuls[2]/vel;
    }
    else if ( vel<=omega_motor_max[1]&& vel>=omega_motor_min[1]) {
      CLR(PORTD,MS2_MOTOR_2);
      SET(PORTD,MS1_MOTOR_2);
      npuls_M2=omega2npuls[1]/vel;
    }
    else if ( vel<=omega_motor_max[0]&& vel>=omega_motor_min[0]) {
      CLRR(PORTD,MS1_MOTOR_2,MS2_MOTOR_2);
      npuls_M2=omega2npuls[0]/vel;
    }
    else {
      SETT(PORTD,MS1_MOTOR_2,MS2_MOTOR_2);
      npuls_M2=npuls_max[3];
    }
  }
  if (vel < 0) {
    if (dir_M2 !=-1) {
      CLR(PORTD,DIR_MOTOR_2);
      dir_M2=-1;
    }
    if (-vel<=omega_motor_max[3] && -vel>=omega_motor_min[3]) {
      SETT(PORTD, MS1_MOTOR_2, MS2_MOTOR_2);
      npuls_M2=-omega2npuls[3]/vel;
    }
    else if (-vel<=omega_motor_max[2]&& -vel>=omega_motor_min[2]) {
      CLR(PORTD,MS1_MOTOR_2);
      SET(PORTD,MS2_MOTOR_2);
      npuls_M2=-omega2npuls[2]/vel;
    }
    else if (-vel<=omega_motor_max[1]&& -vel>=omega_motor_min[1]) {
      CLR(PORTD,MS2_MOTOR_2);
      SET(PORTD,MS1_MOTOR_2);
      npuls_M2=-omega2npuls[1]/vel;
    }
    else if (-vel<=omega_motor_max[0]&& -vel>=omega_motor_min[0]) {
      CLRR(PORTD,MS1_MOTOR_2,MS2_MOTOR_2);
      npuls_M2=-omega2npuls[0]/vel;
    }
    else {
      SETT(PORTD,MS1_MOTOR_2,MS2_MOTOR_2);
      npuls_M2=npuls_max[3];
    }
  }
}