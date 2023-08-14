//DECLARACIÓN DE CABECERAS
#include "MPU6050.h" //dentro de dicha libreria va declarada todas las cabeceras necesarias
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include "ARDUINO.h"


//Para implementar el control LQRI se varía la cuarta componente del vector K a 0.3
float K[] = {-350.22,-47.23,7.42,0};
float offset = 3.5;

//FUNCIONES
float calculo_lqr(float A, float B, float gir);
float filtro(float accel, float gxy);

//CAMBIO DE RADIANES A GRADOS
#define DEG_TO_RAD 0.01745329252
float PI = 3.1415;
int cont;

//VARIABLES PARA EL MPU
float ax,ay,az, ax_escalado, ay_escalado, az_escalado;
float accel_ang_y;
float gx,gy,gz, gx_escalado, gy_escalado, gz_escalado;
float giros_ang_y, giros_ang_y_prev = 3;
float TAU = 0.99;

//VARIABLES PARA COMUNICAR CON ARDUINO
float omega, omega_r;
float alpha;

//VARIABLES PARA CONTROL LQR
float ivr = 0, u;
float dtheta_r, phi,dphi,dtheta;

//VARIABLES PARA TEMPORIZADOR Y SENAL
timer_t temporizador; //declaración del nombre del temporizador
struct itimerspec periodo; //estructura para indicar cada cuanto tiempo se lanza el temporizador
struct timespec inicial;
struct timespec tiempo; //estructura para definirla duración del temporizador
struct sigevent aviso; //establezco una senal que avise que se ha cumplido el temporizador
sigset_t sigset; //conjunto de senales
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
        case 1 :
            getGyro(&gx_escalado, &gy_escalado, &gz_escalado);
            getAccel(&ax_escalado, &ay_escalado, &az_escalado); //lee valores en el sistema internacional g
            accel_ang_y = atan2(az_escalado , (sqrt((ay_escalado*ay_escalado) + (ax_escalado*ax_escalado))))*(180/PI); //da el angulo en grados
            giros_ang_y =filtro(accel_ang_y,gy_escalado);

            //printf("gx %f \n", gx_escalado);
            //printf("gy %f \n", gy_escalado);
            //printf("gz %f \n", gz_escalado);
            //printf("ay %f \n", ay_escalado);
            //printf("az %f \n", accel_ang_y);
            //printf("dphi %f \n", gy_escalado);
            //printf ("angulo %f \n", (giros_ang_y+offset));
        break;

        case 2:
            read(file, &omega, sizeof(omega));
            //printf("omega %f \n", omega_A);
        break;

        case 3:
            read(file, &omega_r, sizeof(omega_r));
            //printf ("ref %f \n", omega_r);
        break;

        case 4:
            alpha=calculo_lqr(omega,omega_r, giros_ang_y);
            //printf ("alfa %f \n", alpha);
        break;

        case 5:
            write(file, &alpha, sizeof(alpha));
            ciclo = 0;
        break;
    }
}

//FUNCIÓN PRINCIPAL MAIN
int main(void ) {
    inicio_mpu(0x68); //inicio la conexión del MPU6050
    inicio_arduino(0x05);//inicio la conexión con ARDUINO
    if (flag==0) {
        flag=1;
        iniciall=clock();
    }
    
    //TRATAMIENTO DE SENALES
    sigemptyset(&sigset); //crea una mascara vacía
    sigaddset(&sigset, SIGUSR1); //anade la senal SIGALARM al conjunto
    pthread_sigmask(SIG_UNBLOCK,&sigset, NULL); //bloqueo la senal para el proceso
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
float calculo_lqr(float A, float B,float gir) {
    phi=(gir+offset)*DEG_TO_RAD;
    dphi=gy_escalado*DEG_TO_RAD;
    dtheta=A;
    ivr=ivr+B-A;
    u=K[0]*(phi)+K[1]*(dphi)+K[2]*(B+A)+K[3]*ivr;
    return (u);
}

//función que calcula el filtro complementario
float filtro(float accel, float gxy) {
    dtt=(double)(clock()-iniciall)/CLOCKS_PER_SEC;
    giros_ang_y_prev=TAU*(giros_ang_y_prev+dtt*gxy)+(1-TAU)*accel;
    iniciall=clock();
    return (giros_ang_y_prev);
}

