// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial 
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
SoftwareSerial BT(10,11);   
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

int analogPin = 3;    
int val = 0;  

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
void lectsensor();
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
int a=0;
void setup() {
  Serial.begin(9600);    //Iniciando puerto serial
  BT.begin(9600);  
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
 
    val = analogRead(analogPin);    
 

  //Mostrar los angulos separadas por un [tab]
  lectsensor();
  if(a==0 & (ang_x>=-89 & ang_x<=-75) & (ang_y>=6 & ang_y<=12)){
    a=1;
    Serial.print("Rotacion en X: \t");
    Serial.print(ang_x); 
    Serial.print("\t Rotacion en Y: \t");
    Serial.println(ang_y);
  
    }
  lectsensor();  
  if(a==1 & (ang_x>=-16 & ang_x<=7) & (ang_y>=-35 & ang_y<=-19)){
    a=2;
    Serial.print("Rotacion en X: \t");
    Serial.print(ang_x); 
    Serial.print("\t Rotacion en Y: \t");
    Serial.println(ang_y);
  }
  lectsensor();
  if(a==2 & (ang_x>=35 & ang_x<=50) & (ang_y>=-45 & ang_y<=-2)){
    a=3;
    Serial.print("Rotacion en X: \t");
    Serial.print(ang_x); 
    Serial.print("\t Rotacion en Y: \t");
    Serial.println(ang_y);
  }
  lectsensor();
  if(a==3){
  BT.print("Hola que tal");
  delay(2000);
  BT.print("Mi nombre es enrique");
  delay(3000);
  BT.print("es un gusto estar aqui");
  
  Serial.println("hola");
  
  a=0;
  }

  lectsensor();
 /* Serial.print("Rotacion en X: \t");
  Serial.print(ang_x); 
  Serial.print("\t Rotacion en Y: \t");
  Serial.println(ang_y);
  */
   
  //delay(10);
}

void lectsensor()
{
 // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  // deribamos el tiempo
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
    
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}
