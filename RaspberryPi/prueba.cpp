// dmesg | grep "tty"
// g++ -o code prueba.cpp -lwiringPi -pthread -lrt
// sudo rm /var/run/pigpio.pid
// sudo netstat -tuln | grep 8888

//DECLARACIÓN DE CABECERAS
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <chrono>
#include <math.h>

using namespace std::chrono;

//Para implementar el control LQRI se varía la cuarta componente del vector K a 0.3
float offset = 3.5;
float K[] = {-344.0464,-42.5310,-7.6876,0.0};

//FUNCIONES
float calculo_lqr(float A, float B, float gir);
float filtro(float accel, float gxy);

//CAMBIO DE RADIANES A GRADOS
#define DEG_TO_RAD 0.01745329252
float PI = 3.1415;
int cont;

//VARIABLES PARA COMUNICAR CON ARDUINO
float omega, omega_r;
uint16_t omegaByte, omega_rByte;
float alpha;
float aceleracion;
int fd;

int gy;
float gyRad, ang_x, ang_y, ang_z;
float gy_escalado, ax_escalado, ay_escalado, az_escalado;

#define A_OFF_X 0.0
#define A_OFF_Y 0.0
#define A_OFF_Z 0.0

#define ACCEL_SENS 835.1

#define G_OFF_X -3.55
#define G_OFF_Y	0.65
#define G_OFF_Z 3.7

#define GYRO_SENS 131.0

float angulo = 0.0;
float accel_ang_y;

float giros_ang_y;
float giros_ang_y_prev;
float angAnterior = 0.0;

float TAU = 0.98;

float dt;
long tiempo_prev;

uint16_t gyByte;

#pragma pack(push, 1)
struct miEstructura {
    uint16_t angByte_x = 0;
    uint16_t angByte_y = 0;
    uint16_t angByte_z = 0;
};
#pragma pack(pop)

//VARIABLES PARA CONTROL LQR
float ivr = 0, u;
float dtheta_r, phi,dphi,dtheta;

//VARIABLES PARA TEMPORIZADOR Y SENAL
timer_t temporizador; //declaración del nombre del temporizador
struct itimerspec periodo; //estructura para indicar cada cuanto tiempo se lanza el temporizador
struct timespec inicial;
struct timespec tiempo; //estructura para definirla duración del temporizador
struct sigevent aviso; //establezco una senal que avise que se ha cumplido el temporizador
sigset_t sign; //conjunto de senales
struct sigaction act;

//VARIABLE EJECUTIVO CICLICO
int ciclo=0;
double dtt;
clock_t iniciall;
int flag=0;

int uint2int(uint16_t valor) {
	int salida = (int)valor;
	if (salida > 32768) {
		salida -= 65536;
	}
	return salida;
}

float uint2float(uint16_t valor) {
	float salida = (float)valor;
	salida -= 32768;
	salida = salida/100;
	return salida;
}

auto start = 0;
auto stop = 0;
auto duration = 0;

static bool yaEjecutado = false;
char vaciar;

const float beta = 0.15;

unsigned char buffer[sizeof(miEstructura)];
miEstructura datos;

//FUNCIÓN MANEJADOR
void Manejador(int signo, siginfo_t *info, void*context) {
    ciclo++;
    //CADA VEZ QUE SE ACTIVA LA SENAL, CICLO AUMENTA EN UNA UNIDAD Y HACE QUE LE CORRESPONDA EN EL SWITCH-CASE
    switch(ciclo) {
        case 1:        
            for (int i = 0; i < sizeof(miEstructura); i++) {
                buffer[i] = serialGetchar(fd);
            }    
            memcpy(&datos, buffer, sizeof(miEstructura));
            
            ang_x = uint2int(datos.angByte_x);
            ang_y = uint2int(datos.angByte_y);
            ang_z = uint2int(datos.angByte_z);
            
            ax_escalado = round((ang_x - A_OFF_X)*1000.0 / ACCEL_SENS)/1000.0;
            ay_escalado = round((ang_y - A_OFF_Y)*1000.0 / ACCEL_SENS)/1000.0;
            az_escalado = round((ang_z - A_OFF_Z)*1000.0 / ACCEL_SENS)/1000.0;           
        break;
        
        case 2:
			for (int i = 0; i < sizeof(uint16_t); i++) {
                buffer[i] = serialGetchar(fd);
            }
            memcpy(&gyByte, buffer, sizeof(uint16_t));

            gy = uint2int(gyByte);
            gy_escalado = round((gy - G_OFF_Y)*1000.0 / GYRO_SENS)/1000.0;
            
            accel_ang_y = -atan2(ax_escalado, (sqrt((az_escalado*az_escalado) + (ay_escalado*ay_escalado))))*(180/PI);
            giros_ang_y = filtro(accel_ang_y,gy_escalado);
            
            std::cout << phi/DEG_TO_RAD <<  "  " << dphi/DEG_TO_RAD << "  " << omega << "  " << alpha << std::endl;
            std::cout << serialDataAvail(fd) << std::endl;
        break;
        
        case 3:
			for (int i = 0; i < sizeof(uint16_t); i++) {
                buffer[i] = serialGetchar(fd);
            }
            memcpy(&omegaByte, buffer, sizeof(uint16_t));

            omega = uint2float(omegaByte);
            
            //std::cout << "omega: " << omega << std::endl;
        break;
        
        case 4:
            alpha = calculo_lqr(omega, 0, giros_ang_y);
            if (alpha < -100) {
				alpha = -100;
			} else if (alpha > 100) {
				alpha = 100;
			}
            //aceleracion += alpha;
            //std::cout << alpha << std::endl;
        break;

        case 5:
            for (int i = 0; i < 4; i++) {
               serialPutchar(fd, *((char *)&alpha + i));
            }
            ciclo = 0;
        break;
    }
}

//FUNCIÓN PRINCIPAL MAIN
int main(void) {
    //CODIGO DE CONEXION CON ARDUINO	
    // Abrir comunicacion serial
    fd = serialOpen("/dev/ttyUSB0", 115200);
	
    if (fd < 0) {
	std::cerr << "Error al abrir puerto serie" << std::endl;
	return 1;
    }
    
    if (!yaEjecutado) {
		while (serialDataAvail(fd) > 0) {
			vaciar = serialGetchar(fd);
		}
		yaEjecutado = true;
	}
    
    //TRATAMIENTO DE SENALES
    sigemptyset(&sign); //crea una mascara vacía
    sigaddset(&sign, SIGUSR1); //anade la senal SIGALARM al conjunto
    pthread_sigmask(SIG_UNBLOCK,&sign, NULL); //bloqueo la senal para el proceso
    /* programo la senar SIGVTALRM para que salte la funcion manejador
    cuando se cumpla el temporizador */
    act.sa_sigaction=Manejador;
    sigemptyset(&(act.sa_mask));
    act.sa_flags=SA_SIGINFO;
    sigaction(SIGUSR1, &act, NULL);
    
    //configuro el temporizador
    inicial.tv_sec=1;
    tiempo.tv_nsec=4000000; //cada 4 ms salta el manejador
    periodo.it_value=inicial; //indico que el primer valor lo lance 1 segundo despues de llamarlo
    periodo.it_interval=tiempo; //indico que los peridos sean cada 4 ms
    aviso.sigev_notify=SIGEV_SIGNAL;
    aviso.sigev_signo=SIGUSR1;
    timer_create(CLOCK_REALTIME, &aviso, &temporizador); //creo el temporizador llamado tempo, de tiempo real y que avise con la senal configurada en el segundo argumento de entrada

    //inicio el temporizador
    timer_settime(&temporizador, TIMER_ABSTIME, &periodo , NULL);
    while(1){}
    exit(0);
}

//función que calcula la acción de control
float calculo_lqr(float A, float B, float gir) {
	phi = (gir + offset)*DEG_TO_RAD;
	dphi = gy_escalado*DEG_TO_RAD;
	dtheta = A;
	ivr = ivr + B - A;
    u = phi*K[0] + dphi*K[1] + (B+A)*K[2] + ivr*K[3];
    return (u);
}

float filtro(float accel, float gxy) {
	dtt = (double)(clock() - iniciall)/CLOCKS_PER_SEC;
	giros_ang_y_prev = TAU*(giros_ang_y_prev + dtt*gxy) + (1-TAU)*accel;
	iniciall = clock();
	return(giros_ang_y_prev);
}
