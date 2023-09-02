float angulo = -10;
int gxt = -10;

float alfa = 0;

#pragma pack(push, 1)
struct miEstructura {
  uint16_t ang;
  uint16_t gx;
};
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
}

void loop() {
  static miEstructura datos;

  angulo += 0.1;
  gxt += 1;
  datos.ang = float2int(angulo);
  datos.gx = (uint16_t)gxt;

  byte* p = (byte*)&datos;
  for (int i = 0; i < sizeof(miEstructura); i++) {
    Serial.write(*p++);
  }
  
  delay(100);
}
/*
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
}*/

uint16_t int2uint16(int valor) {
  uint16_t salida = uint16_t(valor);
  return salida;
}

uint16_t float2int(float valor) {
  uint16_t salida = uint16_t(valor*100);
  return salida;
}