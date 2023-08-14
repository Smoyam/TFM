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

int gx = 0;
float ang = 0;

struct miEstructura {
    uint16_t angByte = 0;
    uint16_t gxByte = 0;
};

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

//FUNCIÓN PRINCIPAL MAIN
int main(void) {
	while (true) {
	int fd = serialOpen("/dev/ttyUSB0", 115200);
	
	if (fd < 0) {
		std::cerr << "Error al abrir puerto serie" << std::endl;
		return 1;
	}
	
	unsigned char buffer[sizeof(miEstructura)];
    for (int i = 0; i < sizeof(miEstructura); i++) {
		buffer[i] = serialGetchar(fd);
    }
            
    miEstructura datos;
    memcpy(&datos, buffer, sizeof(miEstructura));
    gx = uint2int(datos.gxByte);
    ang = uint2float(datos.angByte);
    std::cout << ang << std::endl;
	}
}

