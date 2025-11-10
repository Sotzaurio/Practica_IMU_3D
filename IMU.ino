// Lectura IMU WitMotion por Serial1 en Arduino Mega 2560
// Autor: Ruben Raygosa + ChatGPT

// Tramas WitMotion inician con 0x55, seguidas de un ID de tipo de dato
// 0x51 = Aceleración, 0x52 = Velocidad angular, 0x53 = Ángulo

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

IMUData imu;

void setup() {
  Serial.begin(115200);     // Monitor serie PC
  Serial1.begin(9600);    // IMU en Serial1
  Serial.println("Leyendo IMU WitMotion por Serial1...");
}

void loop() {
  if (Serial1.available() >= 11) {
    if (Serial1.read() == 0x55) {     // Cabecera
      uint8_t type = Serial1.read();  // Tipo de trama
      uint8_t buffer[9];
      for (int i = 0; i < 9; i++) buffer[i] = Serial1.read();

      switch (type) {
        case 0x51: // Aceleración
          imu.ax = ((int16_t)(buffer[1] << 8 | buffer[0])) / 32768.0 * 16.0;
          imu.ay = ((int16_t)(buffer[3] << 8 | buffer[2])) / 32768.0 * 16.0;
          imu.az = ((int16_t)(buffer[5] << 8 | buffer[4])) / 32768.0 * 16.0;
          break;

        case 0x52: // Giroscopio
          imu.gx = ((int16_t)(buffer[1] << 8 | buffer[0])) / 32768.0 * 2000.0;
          imu.gy = ((int16_t)(buffer[3] << 8 | buffer[2])) / 32768.0 * 2000.0;
          imu.gz = ((int16_t)(buffer[5] << 8 | buffer[4])) / 32768.0 * 2000.0;
          break;

        case 0x53: // Ángulo
          imu.roll  = ((int16_t)(buffer[1] << 8 | buffer[0])) / 32768.0 * 180.0;
          imu.pitch = ((int16_t)(buffer[3] << 8 | buffer[2])) / 32768.0 * 180.0;
          imu.yaw   = ((int16_t)(buffer[5] << 8 | buffer[4])) / 32768.0 * 180.0;
          mostrarDatos();
          break;
      }
    }
  }
}

void mostrarDatos() {
  Serial.print("Acel[g]: ");
  Serial.print(imu.ax, 2); Serial.print(", ");
  Serial.print(imu.ay, 2); Serial.print(", ");
  Serial.print(imu.az, 2);

  Serial.print(" | Gyro[°/s]: ");
  Serial.print(imu.gx, 2); Serial.print(", ");
  Serial.print(imu.gy, 2); Serial.print(", ");
  Serial.print(imu.gz, 2);

  Serial.print(" | Ang[°]: ");
  Serial.print(imu.roll, 1); Serial.print(", ");
  Serial.print(imu.pitch, 1); Serial.print(", ");
  Serial.println(imu.yaw, 1);
}
