// // PID controller - V5

// ultrasound sensor
const int trigPin = 2;
const int echoPin = 3;
double duration, distance;
const double heightTube = 38.0 ;  

// PWM
const int PWM_PIN = 9;   // IN1 — PWM for speed (black wire)
const int DIR_PIN = 10;   // IN2 — LOW keeps direction fixed (white wire)
int duty = 0 ;

// PID controller
#include <PID_v1.h> // /home/talles/Arduino/libraries/PID/PID_v1.h
double iKp=15.0, iKi=5.0, iKd=7.5; // reference_var=20, [0:255]
double input_var=0.0 ;
double output_var=0.0 ;
double reference_var=0.0 ; // in inches

PID height_PID(&input_var, &output_var, &reference_var, iKp, iKi, iKd, DIRECT);
// reference_var: reference variable is the height you want 
// output_var: is the controller output, in this case the PWM you will impose to the fan
// input_var: is the measurement of the height

// potentiometer
const int analogPin = A5;
int analogRef = 0 ;
double analogVout = 0.0 ;

// filter (https://github.com/EmotiBit/EmotiBit_ArduinoFilters)
#include <Filters.h> // EmotiBit_ArduinoFilters
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
const double f_s = 0.05;            // Hz // Sampling frequency
const double f_c = 0.02;             // (Hz) // Cut-off frequency (-3 dB)
const double f_n = 2 * f_c / f_s;  // Normalized cut-off frequency
Timer<micros> timer = std::round(1e6 / f_s);
auto filter = butter<6>(f_n);      // Sixth-order Butterworth filter


void setup() {
  Serial.begin(115200);

  // ultrasound  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // PWM
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Fixed direction
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(PWM_PIN, HIGH);

  // PID controller
  height_PID.SetMode(AUTOMATIC);
  height_PID.SetOutputLimits(70, 255);
   


}

void loop() {

  // reference
  analogRef = analogRead(analogPin);
  analogVout = analogRef * (5.0 / 1023.0); // volts
  analogVout = (analogVout/4.89) * 25 ; 
  reference_var = analogVout ;
  

  // output measurements
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2; // in cm
  distance = distance * 0.393701 ; // in inches  
  distance = heightTube - distance ; // tube conversion (heght tube is 38")
  input_var = distance ;
  input_var =  filter(input_var);
  

  // PID controller
  height_PID.Compute();
 
  // input
  duty = 255 - (int)output_var ;
  analogWrite(PWM_PIN, duty);

  Serial.print("(");
  Serial.print(millis());
  Serial.print(" ; ");
  Serial.print(input_var); 
  Serial.print(" ; ");
  Serial.print(output_var);
  Serial.print(" ; ");
  Serial.print(reference_var);
  Serial.print(")\n");
  
  delay(30);
}
