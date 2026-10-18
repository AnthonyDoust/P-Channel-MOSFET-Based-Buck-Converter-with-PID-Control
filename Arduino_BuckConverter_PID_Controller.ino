#include <PID_v1.h>

// pin assignments
int inputpin = A0;
int PWMoutpin = 9;
int potpin = A1;
int potval;

unsigned long previoustime;
unsigned long time;
unsigned long timediff;
int interval = 50;
float VoltageOut;

float targetVoltage; // set to your desired voltage output
float voltage = 12.33; // Set to your voltage source input ***ADJUST VOLTAGE DIVIDER ACCORDINGLY
float cal = 1; // calibration value for voltage input, best to re calilbrate for new loads

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=100, Kd=0;  //Kp=4, Ki=40, Kd=0
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  pinMode(PWMoutpin, OUTPUT);  // OC1A output

  // Clear Timer1 Control Registers
  TCCR1A = 0;
  TCNT1  = 0;

  // Set Fast PWM Mode 14 (WGM13=1, WGM12=1, WGM11=1, WGM10=0)
  // Set non-inverting mode for both Channel A and Channel B (COM1A1=1, COM1B1=1)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // CS10=1 sets Prescaler to 1

  // Set the TOP value for 125kHz
  ICR1 = 127;

  Serial.begin(115200);
  Serial.println("Start");

  //initialize the variables we're linked to
  Input = analogRead(inputpin);
  
  Setpoint = targetVoltage;

  myPID.SetMode(AUTOMATIC);   //turn the PID on
  myPID.SetSampleTime(interval); // sample time of the PID controller
  myPID.SetOutputLimits(0, 127); // limits of the PID output

}

void loop() {

// keep track of current time and the difference between previous and current
time = millis();
timediff = time - previoustime;

// interval to check the voltage and serial print values
if (time - previoustime >= interval) {

  previoustime = time;

    //read the value of the the pot and then map it as a float to the target voltage, then set it as the setpoint for the PID
    potval = analogRead(potpin);
    targetVoltage = mapFloat(potval,0.0,1023.0,0.0,12.33);
    Setpoint = targetVoltage;

  VoltageOut = ((analogRead(inputpin)) * (voltage/1023))*cal; // equation converting input analog value to voltage

  Input = VoltageOut;

  float DutyCycleOut = (Output/127) * 100;

  // Serial plotter outputs for troubleshooting and tuning
     Serial.print(Setpoint);
     Serial.print(",");
     Serial.println(VoltageOut);
}

  // Compute and output PID output
  myPID.Compute();
  OCR1A = (int)Output;

    // if source voltage drops to 0, reset the accumulation of the integrator
  if (VoltageOut == 0) {
    myPID.SetMode(MANUAL);  
    Output = 0.0;            
    myPID.SetMode(AUTOMATIC); 
  }

}


