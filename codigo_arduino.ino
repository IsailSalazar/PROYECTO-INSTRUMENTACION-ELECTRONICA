// UNIVERSIDAD INDUSTRIAL DE SANTANDER
// INSTRUMENTAC！ON ELECTRONICA
// PROYECTO:
  // DISENO DE UN SISTEMA DE SENSORES INALAMBRICO DRONE-EQUIPABLE
// INTERFAZ GRAFICA: 
  // CORRER EL CODIGO CON EL ARDUINO CONECTADO MEDIANTE USB
// INTEGRANTES:
  // Fernandez Laude - 2122264,
  // Pico Jean Alejandro - 2112259,
  // Salazar Isail - 2122307,
  // Rovira Maria Fernanda - 2122256,
  // Vasquez Paula - 2120542.

//######################### ACCEL ############################################
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu(0x68); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false; //true;//false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void Accelerometer(){
    if (!dmpReady) return;
       mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("Posicion en grados [x,y,z]:\t");
            Serial.print(ypr[0] * 180/M_PI);
            //Serial.print(";");
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            //Serial.print(";");
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            //Serial.print(";");
            Serial.print("\t");
        #endif
        #ifdef OUTPUT_READABLE_REALACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif
        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif 
        #ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; 
        #endif
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

//################################TEMP HUMEDAD###########################################
#include "DHT.h"
#define DHTTYPE DHT22   
const int DHTPin = 8;    
DHT dht(DHTPin, DHTTYPE);
void temp_humedad(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();  
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
    return;
    }
    Serial.print(h);
    //Serial.print("h");
    //Serial.print(";");
    Serial.print("\t");  
    Serial.print(t);
    //Serial.print("tem1");
    //Serial.print(";");
    Serial.print("\t");
}

//################ CO2 ###################################################################
void co2_sensor(){ 
  int Rs,co2_gs;
  co2_gs=analogRead(3); //Ro = Rs * sqrt(a/ppm, b) = Rs * exp( ln(a/ppm) / b )
  float Ro=30370.81028;
  float voltaje = co2_gs * (5.0 / 1023.0); // RS=16550 Al ambiente. PPM=400 --> RO=25830.56221
  Rs=1000*((5-voltaje)/voltaje);          // RS=13013 Al ambiente. PPM=400 --> RO=20310.15747   
  double   ppm = 116.6020682*pow(Rs/Ro,-2.769034857 ); // RS=19459 Al ambiente. PPM=400 --> RO=30370.81028
  Serial.print(ppm);
  //Serial.print("co2");
  //Serial.println(";");
  Serial.println("\t");
}

//############################# TEMP- PRESION ###########################################
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 bmp180;
double PresionNivelMar=1013.25; //presion sobre el nivel del mar en mbar
void presion_sensor(){
  char status;
  double T,P,A; 
  status = bmp180.startTemperature();//Inicio de lectura de temperatura
  if (status != 0)
  {   
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3);//Inicio lectura de presión
      if (status != 0)
      {        
        delay(status);//Pausa para que finalice la lectura        
        status = bmp180.getPressure(P,T);//Obtenemos la presión
        if (status != 0)
        {                  
          Serial.print(T);
          //Serial.print("tem2");
          //Serial.print(";");
          Serial.print("\t");
          
          Serial.print(P);
          //Serial.print("pres");
          //Serial.print(";");
          Serial.print("\t"); 
          A= bmp180.altitude(P,PresionNivelMar);
          Serial.print(A);
          //Serial.print("altu");
          //Serial.print(";");
          Serial.print("\t");   
        }      
      }      
    }   
  } 
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif
    mpu.setI2CBypassEnabled(true);
    Serial.begin(9600);
    while (!Serial);
    //###################TEMP- PRESION###########################################
    if (bmp180.begin())
    Serial.println("BMP180 iniciado correctamenten");
  else
  {
    Serial.println("Error al iniciar el BMP180");
    while(1); // bucle infinito
  }
  //####################### ACCEL ###############################################
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-9);
    mpu.setYGyroOffset(67);
    mpu.setZGyroOffset(26);
    mpu.setZAccelOffset(1422); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);   
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    co2_sensor();
    presion_sensor();
    temp_humedad();
    Accelerometer();
}
