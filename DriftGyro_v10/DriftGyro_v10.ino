
// Drift Gyro 
// Invensense MPU6050 Sensor und Arduino Pro Mini
//
// Version 1.0
// - SteeringIn Signal mit Kalman-Filter
//
// Version 0.9
// - RAW Daten Filter Mittelwert (bringt nichts)
//
// Version 0.8
// - RAW Daten Filter Kalman (bringt verzögerung)
//
// Version 0.6
// - ServoGeschwindigkeit einstellbar
//
// Version: 0.5
// -mit PinChangeInt.h zum Auslesen aller 3 RC-Channels
// -Bremslicht-Funktion hinzugefügt

#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
#include <PinChangeInt.h>
#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Assign your channel in pins
#define THROTTLE_IN_PIN 4
#define STEERING_IN_PIN 2
#define AUX_IN_PIN 3

// Assign your channel out pins
#define STEERING_OUT_PIN 9

// Assign your Brake-Light pin
#define BRAKE_PIN 8

// this is the duration in µs of min, neutral, max Position of Servo
#define MIN_STEERING 1000     //1000
#define MAX_STEERING 2000     //2000
#define NEUTRAL      1500     //1500

// These bit flags are set in UpdateFlags to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4

// holds the update flags defined above
volatile uint8_t UpdateFlags = 0;

// Servo Speed
#define SERVO_SPEED 80 // Geschwindigkeit in 0..100%
int SteeringInOld = NEUTRAL;
int SteeringInDifference = 0;

// Variabeles for the RC-Channel reading
volatile int ThrottleIn = NEUTRAL;
volatile int SteeringIn = NEUTRAL;
volatile int AuxIn = NEUTRAL;
volatile unsigned long ThrottleInStart;
volatile unsigned long SteeringInStart;
volatile unsigned long AuxInStart;

Servo SteeringServo; // Lenkservo
int GyroVal;         // Gyro-Eingriff in µs
int gain = 50;       // Verstärkung in % (0..100%)
int gainNew = 50;    // Neuer gain-Wert
MPU6050 mpu;         // Sensor Invensense MPU6050
int16_t gx, gy, gz;  // Gyroskop RAW-Daten

// Eindimensionales lineares Kalman-Filter
double q = 4; //process noise covariance 4, höher -> direkter
double r = 16; //measurement noise covariance 32, kleiner -> direkter
double p = 2000; //estimation error covariance, wird stets neu berechnet
double k = 0; //kalman gain, wird berechnet
int16_t x = 1500; //value 1500 Mittelstellung, Ergebnis wird berechnet
int16_t kalman_update(int16_t measurement); // Funktion gibt aktuellen Wert x aus


void setup()
{
  Serial.begin(115200);

  Serial.println("Drift Gyro");
  
  pinMode(BRAKE_PIN,OUTPUT);
  
  SteeringServo.attach(STEERING_OUT_PIN, MIN_STEERING, MAX_STEERING);
  
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);
  
  Wire.begin();
  Serial.print("Initialize MPU: ");
  mpu.initialize(); // Power on and prepare for general usage
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  
  // Setup Empfindlichkeit ///////////////////////////////////////////////////////
  mpu.setFullScaleGyroRange(0); // 0 = +/- 250 degrees/sec
                                // 1 = +/- 500 degrees/sec
                                // 2 = +/- 1000 degrees/sec
                                // 3 = +/- 2000 degrees/sec
  Serial.print("Empfindlichkeit-Stufe ");
  Serial.println(mpu.getFullScaleGyroRange());
  
  // Setup Filter ////////////////////////////////////////////////////////////////
  mpu.setDLPFMode(6);
  /*                      |   ACCELEROMETER    |       GYROSCOPE
                 DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
                 ---------+-----------+--------+-----------+--------+-------------
                 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
                 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
                 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
                 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
                 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
                 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
                 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
                 7        |   -- Reserved --   |   -- Reserved --   | Reserved
  */
  Serial.print("Filter-Stufe ");
  Serial.println(mpu.getDLPFMode());  
}


// HAUPTPROGRAMM //////////////////////////////////////////////////////////////////
void loop()
{
  if(UpdateFlags & STEERING_FLAG)  // Wenn neues Lenk-Signal vorhanden,...
  {
    //Serial.println(" ");
    //Serial.print("SteeringIn/us    "); 
    //Serial.print(SteeringIn); 
    //Serial.print("\t"); 
    //Serial.print("\t"); 
   
    //gx = mpu.getRotationX(); // X-Gyroskop-Daten einlesen
    gy = mpu.getRotationY(); // Y-Gyroskop-Daten einlesen
    //gz = mpu.getRotationZ(); // Z-Gyroskop-Daten einlesen
    //Serial.print("GyroRAWdata      ");
    //Serial.println(gy);  
   
    GyroVal = map(-gy, -32768, 32767, -10*gain, 10*gain); // Umrechnen in µs außerhalb Neutralposition
   
    SteeringIn += GyroVal;    // Servo Signal korrigieren
    //Serial.print("SteeringIn kor   ");
    Serial.print(SteeringIn);
    Serial.print("\t"); 
    Serial.print("\t"); 
    
    SteeringIn = kalman_update(SteeringIn); // Kalman-Filter
    Serial.println(SteeringIn);
     
    // Servo Geschwindigkeit
    SteeringInDifference = SteeringIn - SteeringInOld; // max 200µs je Iteration für Servo mit 0,05s Stellzeit
    SteeringIn = SteeringInOld + ((SteeringInDifference*SERVO_SPEED)/100);
    
    //Serial.print("SteeringIn speed ");
    //Serial.println(SteeringIn);
    
    // Begrenzungen
    if(SteeringIn<MIN_STEERING){
       SteeringIn=MIN_STEERING;}
    if(SteeringIn>MAX_STEERING){
       SteeringIn=MAX_STEERING;}
       
    SteeringServo.writeMicroseconds(SteeringIn); // Servo ansteuern mit korrigiertem Signal
    
    SteeringInOld = SteeringIn; // aktuellen Wert zwischenspeichern
   }

  if(UpdateFlags & AUX_FLAG)  // Wenn neues Aux-Signal vorhanden,...
  {
    //Serial.println(" ");
    //Serial.print("AuxIn/us     "); 
    //Serial.println(AuxIn);
   
    gainNew = map(AuxIn, 1000, 2000, 0, 100); //Umrechnen in %
   
    // Begrenzungen
    if(gainNew>100){
      gainNew=100;}
    if(gainNew<0){
      gainNew=0;}
   
    gain = gainNew;   //gain mit neuem gain-Wert überschreiben
   
    //Serial.print("gain         "); 
    //Serial.println(gain); 
  }

  if(UpdateFlags & THROTTLE_FLAG)  // Wenn neues Gas-Signal vorhanden,...
  {
    //Serial.println(" ");
    //Serial.print("ThrottleIn/us"); 
    //Serial.println(ThrottleIn); 
    
    if(ThrottleIn < (NEUTRAL-50)) // Bremse
    {
      digitalWrite(BRAKE_PIN, HIGH);
    }
    else
    {
      digitalWrite(BRAKE_PIN,LOW);
    }
  }

  UpdateFlags = 0;
}



// interrupt service routines ///////////////////////////////////////////////////
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    ThrottleInStart = micros();
  }
  else
  {
    ThrottleIn = (int)(micros() - ThrottleInStart);
    UpdateFlags |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    SteeringInStart = micros();
  }
  else
  {
    SteeringIn = (int)(micros() - SteeringInStart);
    UpdateFlags |= STEERING_FLAG;
  }
}

void calcAux()
{
  if(PCintPort::pinState)
  {
    AuxInStart = micros();
  }
  else
  {
    AuxIn = (int)(micros() - AuxInStart);
    UpdateFlags |= AUX_FLAG;  
  }
}


// Kalman-Filter Berechnung
int16_t kalman_update(int16_t measurement)
{
  p = p + q;
  k = p / (p + r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;
  return x;
}
