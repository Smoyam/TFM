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

using namespace std::chrono;

//Para implementar el control LQRI se varía la cuarta componente del vector K a 0.3
float K[] = {169.5105,20.9502};

//FUNCIONES
float calculo_lqr(float A, float B);

//CAMBIO DE RADIANES A GRADOS
#define DEG_TO_RAD 0.01745329252
float PI = 3.1415;
int cont;

//VARIABLES PARA COMUNICAR CON ARDUINO
float omega, omega_r;
float alpha;
uint16_t alphaByte;
int fd;

int gx;
float gxRad;
float ang;

float gxAnterior;
float angAnterior;

//float alfa = 0.15;

#pragma pack(push, 1)
struct miEstructura {
    uint16_t angByte = 0;
    uint16_t gxByte = 0;
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
	if (salida > 32768) {
		salida -= 65536;
	}
	salida = salida/100;
	return salida;
}

auto start = 0;
auto stop = 0;
auto duration = 0;

static bool yaEjecutado = false;
char vaciar;

//FUNCIÓN MANEJADOR
void Manejador(int signo, siginfo_t *info, void*context) {
    ciclo++;
    //CADA VEZ QUE SE ACTIVA LA SENAL, CICLO AUMENTA EN UNA UNIDAD Y HACE QUE LE CORRESPONDA EN EL SWITCH-CASE
    switch(ciclo) {
        case 1:
            alpha = calculo_lqr(ang,gxRad);
            //std::cout << alpha << std::endl;
        break;

        case 2:
            for (int i = 0; i < 4; i++) {
               serialPutchar(fd, *((char *)&alpha + i));
            }
        break;
        
        case 3:        
            unsigned char buffer[sizeof(miEstructura)];
            for (int i = 0; i < sizeof(miEstructura); i++) {
                buffer[i] = serialGetchar(fd);
            }
            
            miEstructura datos;
            memcpy(&datos, buffer, sizeof(miEstructura));
            
            gx = uint2int(datos.gxByte);
            ang = uint2float(datos.angByte);
            
            ang = ang*DEG_TO_RAD;
            gxRad = (float)gx*DEG_TO_RAD;
            
            //gxRad = gxRad*alfa + gxAnterior*(1-alfa);
            //ang = ang*alfa + angAnterior*(1-alfa);
            std::cout << ang << "  " << gxRad << "  " << alpha << std::endl;
            
            //gxAnterior = gxRad;
            //angAnterior = ang;            
            
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
float calculo_lqr(float A, float B) {
    u = A*K[0] + B*K[1];
    return (u);
}

