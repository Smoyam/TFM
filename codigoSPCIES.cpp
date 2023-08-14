// dmesg | grep "tty"
// g++ -o code codigoSPCIES.cpp -lpigpio -pthread -lrt
// sudo rm /var/run/pigpio.pid
// sudo netstat -tuln | grep 8888

//DECLARACIÓN DE CABECERAS
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <pigpio.h>
#include <iostream>
//#include <spcies.h>

char* puerto = "/dev/ttyUSB0";

//Para implementar el control MPC con SPCIES
float K[] = {-350.22,-47.23};
float offset = 3.5;

//FUNCIONES
float calculo_lqr(float A, float B);

//CAMBIO DE RADIANES A GRADOS
#define DEG_TO_RAD 0.01745329252
float PI = 3.1415;
int cont;

//VARIABLES PARA COMUNICAR CON ARDUINO
float omega, omega_r;
float alpha;
int fd;

char* angByte;
char* gxByte = 0;
char* alphaByte = 0;

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

//FUNCIÓN MANEJADOR
void Manejador(int signo, siginfo_t *info, void*context) {
    ciclo++;
    //CADA VEZ QUE SE ACTIVA LA SENAL, CICLO AUMENTA EN UNA UNIDAD Y HACE QUE LE CORRESPONDA EN EL SWITCH-CASE
    //printf("ciclo %d \n", ciclo);
    switch(ciclo) {
        case 1:
            serRead(fd, angByte, 1);
            std::cout << angByte << std::endl;
        break;

        case 2:
            serRead(fd, gxByte, 1);
            std::cout << gxByte << std::endl;
        break;

        case 3:
            alpha=calculo_lqr(omega,omega_r);
            std::cout << alpha << std::endl;
        break;

        case 4:
            serWrite(fd, alphaByte, 1);
            ciclo = 0;
        break;
    }
}

//FUNCIÓN PRINCIPAL MAIN
int main(void) {
    //CODIGO DE CONEXION CON ARDUINO
    // Inicializar
	if (gpioInitialise() < 0) {
		std::cerr << "Error al inicializar pigpio" << std::endl;
		return 1;
	}
	
	// Abrir comunicacion serial
	fd = serOpen(puerto, 115200, 0);
	
	if (fd < 0) {
		std::cerr << "Error al abrir puerto serie" << std::endl;
		gpioTerminate();
		return 1;
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

