
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>
#include <signal.h>
#include <chrono>
#include <thread>
#include <math.h>


#define PORT "/dev/ttyUSB0"

enum states
{
  START_PACKET,
  READ_STATUS,
  READ_PAYLOAD,
  READ_CRC
};

enum commands
{
  NULLMSG,
  START_SEG,
  STOP_SEG,
  SETSPEEDS
};

enum status
{
  READY,
  NREADY,
  RUNNING,
  ERROR
};

#pragma pack(1)
typedef struct
{
  uint32_t start;
  uint8_t status;
  float phi;
  float dphi;
  float omega_A;
  float omega_B;
  uint32_t CRC;
} dataPacketRX;

#pragma pack(1)
typedef struct
{
  uint32_t start; // 4 bytes
  uint8_t command;// 1 byte
  float setpoint1;  // 4 bytes
  float setpoint2;  // 4 bytes
  uint32_t CRC;   // 4 bytes
} dataPacketTX;   // Total 17 bytes

typedef union
{
  dataPacketRX dataStructure;
  uint8_t dataArrray[25]; // Nuestra estructura tiene 25 bytes
} dataUnionRX;

typedef union
{
  dataPacketTX dataStructure; // 17 bytes struct 
  uint8_t dataArrray[17]; // Nuestra estructura tiene 17 bytes
} dataUnionTX;

dataUnionTX dataSend = {{0xFFFFFFFF,NULLMSG,0.0,0.0,0}};  
dataUnionRX dataRecieve;

uint8_t currentComsState = START_PACKET;
uint8_t segwayStatus = NREADY;

uint8_t cbytes = 0;
uint8_t byteIndex = 0;
uint8_t newPacket = false;

float angSP = 0;
float linSP = 0;

float incAngSP = 0.01;
#define maxAngSP 0.5

float incLinSP = 0.05;
#define maxLinSP 4.0

// Cosas de control
float dtheta = 0.0;   // Linear speed
float dtheta_r = 0.0; // Linear speed setpoint
float dpsi = 0.0;     // Angular speed
float dpsi_r = 0.0;   // Angular speed setpoint
float phi = 0.0;      // Segway angle with respect to vertical
float dphi = 0.0;     // Segway angle speed

float omega_A = 0.0; // Motor A speed
float omega_B = 0.0; // Motor B speed
float ivr = 0.0;     // Integral error

float t=0;

float u[2];

float K[] = { // LQR parameters
    -647.6717 * 0.6,
    -69.3064 * 2.5,
    -7.9098 * 1,
    -0.2,
    3};

// Port handler
int port_fd;

// Prototypes
void signal_callback_handler(int signum);
void read(void);
int write(void);
void computeLQR(void);

int main(int argc, const char *argv[])
{
  using namespace std::this_thread; // sleep_for, sleep_until
  using namespace std::chrono;      // nanoseconds, system_clock, seconds

  // Register SIGINT to our signal handler
  signal(SIGINT, signal_callback_handler);

  // Open Serial port
  if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
  {
    std::cout << "Cannot open serial port" << std::endl;
    return -1;
  }

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  struct termios options;
  tcgetattr(port_fd, &options);
  // Serial port settings
  // options.c_cflag = B57600 | CS8 | CLOCAL | CREAD; //<Set baud rate
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  // Flush the port of any data
  tcflush(port_fd, TCIFLUSH);
  // Set the configuration on our port
  tcsetattr(port_fd, TCSANOW, &options);

  // Make sure the system running the code defines a float a 4 bytes (I think all platforms do)
  if (sizeof(float) != 4)
  {
    printf("Incompatible system, float is not 4 bytes!\n");
    return -1;
  }

  // Start a timer
  auto start = std::chrono::high_resolution_clock::now();

  while (true)
  {

    read(); // As fast as possible

    // Calculate elapsed time
    auto elapsed = std::chrono::high_resolution_clock::now() - start;

    // Cada 20.48ms = 4.096ms * 5;
    if (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() > 20480)
    { 
      start = std::chrono::high_resolution_clock::now();
      
      dataSend.dataStructure.command = NULLMSG; // By default

      // If it's runnig we command it to move
      if (segwayStatus == RUNNING)
      {
        // Compute LQR
        computeLQR();
        // Set speeds
        dataSend.dataStructure.command = SETSPEEDS; 
      }

      // If its ready we start the motors
      if (segwayStatus == READY)
      {
        // start the motors
        dataSend.dataStructure.command = START_SEG;
      }

      // If its ready we start the motors
      if (segwayStatus == NREADY)
      {
        // Do nothing
        dataSend.dataStructure.command = NULLMSG;
      }
      // Send data
      write(); 
    }
    

    sleep_for(microseconds(500)); // Not sure if necesary
  }

    // It should never get here put just in case
  if (port_fd != -1)
    close(port_fd);
  
  return 0;
}

  // Es necesario cerrar el puerto serie antes de terminal el programa porque sino se queda colgado al intentar abrirlo otravez
  void signal_callback_handler(int signum)
  {
    std::cout << "Caught signal, stopping segway." << signum << std::endl;


    // Keep sending stop signal untill it reads READY (Means it's not running)
    while(dataRecieve.dataStructure.status != READY)
    {
      // start the motors
      dataSend.dataStructure.command = STOP_SEG;
      
      // Send data if its disconnected then exit loop and terminate
      if(write()==-1){
        break;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(2000)); // Not sure if necesary
      
      read();
    }
     
    // Close port
    if (port_fd != -1)
      close(port_fd);

    // Terminate program
    exit(signum);
  }

  void computeLQR()
  {
    // Setpoints
    dtheta_r = 0.0;
    dpsi_r = 0.0;

    // calculamos al acción del control
    dtheta = (omega_A + omega_B) / 2.0;    // Cinematica inversa de robot diferencial -> velocidad lineal
    dpsi = (omega_A - omega_B) * 5 / 13.3; // Cinematica inversa de robot diferencial -> velocidad angular
    ivr = ivr + (dtheta_r - dtheta);       // Error integral

    // LQR
    u[0] = K[0] * (phi) + K[1] * (dphi) + K[2] * (dtheta_r - dtheta) + K[3] * ivr;
    u[1] = K[4] * (dpsi_r - dpsi);

    // // Velocidad o Aceleracion de cada rueda
    dataSend.dataStructure.setpoint1 = u[0] + 0.5 * u[1] * 13.3 / 5;
    dataSend.dataStructure.setpoint2 = u[0] - 0.5 * u[1] * 13.3 / 5;

    // Update control signals
    // dataSend.dataStructure.setpoint1 = u[0];
    // dataSend.dataStructure.setpoint2 = u[1];

  

    // t+=0.02;

    // dataSend.dataStructure.alpha_A = 2*sin(2*M_PI*0.2*t);
    // dataSend.dataStructure.alpha_B = 2*sin(2*M_PI*0.2*t);
  }

  void read()
  {
    // If port is not open then something is wrong
    if (port_fd != -1)
    {

      uint8_t c;
      int i = 0, r = 0;
      uint8_t byteIndexs = 0;

      // Read the incomming bytes, MAY change this to an if in the future, not a good idea to han the porcess waiting for bytes
      while ((r = ::read(port_fd, &c, 1)) > 0 /* && i < 40*/)
      {
        switch (currentComsState)
        {
          case START_PACKET:
            if (c == 0xFF)
            {
              cbytes++;
            }
            else
            {
              cbytes = 0;
            }
            // We will stay in START state untill we get 4 consecutive 0xFF bytes indicating packet start condition
            if (cbytes == 4)
            {
              currentComsState = READ_STATUS;
              cbytes = 0;
              byteIndex = 0; // We already counted the first 4 bytes
            }
            break;
          case READ_STATUS:
            dataRecieve.dataStructure.status = c;
            currentComsState = READ_PAYLOAD;
            break;

          case READ_PAYLOAD:
            dataRecieve.dataArrray[4 /* 4 start bytes */ + 1 /* Status byte */ + byteIndex] = c; // Store byte in buffer
            byteIndex++;
            if (byteIndex == 16)
            { // float size = 4 bytes , 4 float variables = 16 bytes.
              byteIndex = 0;
              currentComsState = READ_CRC; // If we have read 40 bytes of paylod we can now check
            }
            break;
          case READ_CRC:
            dataRecieve.dataArrray[4+1+16+byteIndex] = c;
            byteIndex++;
            if(byteIndex == 4){
              byteIndex = 0;
              newPacket = true;
              currentComsState = START_PACKET;
            }
            break;

          default:
            break;
        }
      }

      /*
       * NEW PACKET
       */

      if (newPacket)
      {

        // Process new data
        switch (dataRecieve.dataStructure.status)
        {
        case NREADY: 
          if(segwayStatus != NREADY)
            printf("Waiting for segway.\n\r");
          segwayStatus = NREADY;
          break;

        case READY:
          // Send start command
          if(segwayStatus != READY)
            printf("Segway now ready.\n\r");
          segwayStatus = READY;

          break;

        case RUNNING: // Usually you wouldn't do anything here
          if(segwayStatus != RUNNING)
            printf("Segway is running.\n\r");
          segwayStatus = RUNNING;

          // Update control variables
          omega_A = dataRecieve.dataStructure.omega_A;
          omega_A = dataRecieve.dataStructure.omega_B;
          dphi = dataRecieve.dataStructure.dphi;
          phi = dataRecieve.dataStructure.phi;

          break;

        default:
          printf("Unknown status error: %d\n\r", dataRecieve.dataStructure.status);

          break;
        }

        // printf("Status: %d\tphi: %f\t dphi: %f\t omega_A: %f\t  omega_B: %f CRC: %d\n\r",
        printf("%f,%f,%f,%f,%f,%f\n\r",
           dataRecieve.dataStructure.phi,
           dataRecieve.dataStructure.dphi,
           dataRecieve.dataStructure.omega_A,
           dataRecieve.dataStructure.omega_B,
           dataSend.dataStructure.setpoint1,
           dataSend.dataStructure.setpoint2); 

        newPacket = false;
      }

      if (r < 0 && errno != EAGAIN)
      {
        std::cout << "Reading from serial" << PORT << "failed:" << r << std::endl;
      }
    }
  }

  int write()
  {
    int rc;
    
    if (port_fd == -1)
    {
      printf("Attempt to write on closed serial\n");
      return -1;
    }
    
    rc = ::write(port_fd, (const void *)dataSend.dataArrray, 17);
    
    if (rc < 0)
    {
      printf("Error writing to serial port\n");
      return -2;
    }
    return 0;
  }