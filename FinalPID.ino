#include <ESP32Encoder.h>
#include <esp_timer.h>
#include <esp_attr.h>  //Provides IRAM_ATTR for placing ISR/timer code in IRAM

#define CHANNEL_A 15 //yellow wire
#define CHANNEL_B 16 //green wire
#define IN1 47 //yellow wire
#define PWM 48 //green wire
//#define PWM 5 //yellow wire

//Declare encoder
ESP32Encoder encoder;   

//Encoder specification
const float ENCODER_PPR = 187.95f; //Number of signal high's per revolution
const int QUADRATURE_MULTIPLIER = 4; //We're cointing 4 edges/signals per click on the encoder for high resolution
const long COUNTS_PER_REVOLUTION = (long)(ENCODER_PPR * QUADRATURE_MULTIPLIER);

//How often we read the encoder (50 ms)
const uint64_t READ_INTERVAL_MICROSECONDS = 50000;

//Encoder snapshot state
volatile int64_t latestEncoderCount = 0;
int64_t latestTimestampMicroseconds = 0;
int64_t newReadingAvailable = false;
volatile int64_t timeNow = 0;
volatile bool isNewReading = false;

//Previous reading (used to compute change)
int64_t prevEncoderCount = 0;
int64_t prevTimestampMicroseconds = 0;

//RPM Calculator and Filter Params
int prevRPM = 0;
float currentRPMWeight = 0.4; //ema filter decimal weigth
float cleanRPM = 0;
int64_t countChange = 0; 

//PID params
double integral, previous, pidOutput = 0; 
double Kp, Ki, Kd; //for now just following a setup tutorial, we can tune this later
double setpoint = 50;
double dt;

//Periodic-timer Callback
//Function called to get current time and encoder counts for RPM readings.
static void IRAM_ATTR timerCallback(void* arg) {
  timeNow = esp_timer_get_time();
  isNewReading = true;
}

void setup() {
  //PID set up stuff
  Serial.begin(115200);
  Serial.print("Setup");
  Serial.println("Encoder RPM - cleaned variable names");

  Kp = 1;
  Ki = 0.1;
  Kd = 0.01;

  integral = 0;
  previous = 0;
  //RPM Calc Setup

  //Set encoder channels as input pins
  pinMode(CHANNEL_A, INPUT_PULLUP);
  pinMode(CHANNEL_B, INPUT_PULLUP); //Attach

  //Attach PWM and Specific motor pin as Output channels
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);

  //Set up LED control, the way esps can write to the motor.
  ledcAttach(PWM, 20000, 8); //Attach PWM pin, 20kHz frequency, 8-bit resolution 
  // 20 kHz, 8-bit resolution

  encoder.attachFullQuad(CHANNEL_A, CHANNEL_B);
  encoder.clearCount();
  delay(10);

  //PERIODIC TIMER SETUP
  //Create periodic timer argument that will run on the esp32, using the callback from timerCallback
  const esp_timer_create_args_t timerArgs = {
    .callback = &timerCallback,
    .name = "encoder_reader"
  };

  esp_timer_handle_t periodicTimer;
  esp_timer_create(&timerArgs, &periodicTimer);

  //Start periodic timer
  esp_timer_start_periodic(periodicTimer, READ_INTERVAL_MICROSECONDS);

}

void loop() {
  //--- Copy variables from timer callback ---- 
  latestTimestampMicroseconds = timeNow;
  newReadingAvailable = isNewReading;
  //---------------------------------

  if (!newReadingAvailable) return;
  
  latestEncoderCount = encoder.getCount(); // Get encoder count from encoder object

  //float setpoint = 50; //this changes from jetson comm

  //Calculates the error in the system
  double actualRpm = rpmCalculator();// from our rpm calc
  double error = setpoint - actualRpm;
  pidOutput = pid(error, dt);

  //Scale / constrain PID output to 0–255 for PWM
  //use `fabs` for absolute value
  //NOTE: Direction is noted by `dir` so this doesn't affect signal accuracy
  int pwr = constrain((int)fabs(pidOutput), 0, 255);

  //Determine direction
  int dir = 1;
  if(pidOutput < 0) dir = -1;  // Where voltage and direction get set

  //Send to motor
  setMotor(dir, pwr);

  //print out values for validation
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print(", Actual: "); Serial.print(actualRpm);
  Serial.print(", Error: "); Serial.print(error);
  Serial.print(" Encoder count"); Serial.println(countChange); 
  
  newReadingAvailable = false;
}

double rpmCalculator() {

  //Compute changes in encoder count and time
  countChange = latestEncoderCount - prevEncoderCount;
  int64_t timeChangeMicroseconds = latestTimestampMicroseconds - prevTimestampMicroseconds;

  //set dt for pid in seconds
  dt = timeChangeMicroseconds / 1e6;

  //Only calculate RPM when there's new data
  if (timeChangeMicroseconds > 0) { //Zero-division guard
    //Next iteration setup
    prevEncoderCount = latestEncoderCount;
    prevTimestampMicroseconds = latestTimestampMicroseconds;
  }

  //Integer RPM calculation
  //RPM = (countChange * 60,000,000) / (COUNTS_PER_REVOLUTION * timeChangeMicroseconds)
  int64_t currentRPM = (countChange * 60LL * 1000000LL) / ((int64_t)COUNTS_PER_REVOLUTION * timeChangeMicroseconds); //returns unfiltered RPM 

  //Clean with low-pass ema filter
  cleanRPM = fabs(emaFilter(currentRPM, prevRPM, currentRPMWeight));
  
  Serial.print("RPM: ");
  Serial.print(cleanRPM);

  //Store for next reading
  prevRPM = cleanRPM;
  
  return cleanRPM;
}

double pid(double error, double dt){
  //PID calculation
  //First gets the proportional part of PID, which is just the error.
  double proportional = error;

  //The integral part sums up all the error and multiplies it by change in time. (Rough approx)
  integral += error * dt;
  
  //derivative finds the slope of the errors (Rough approx)
  //Guard against zero-division runtime error
  double derivative = dt > 0 ? (error - previous) / dt : 0; // defaults to 0 
  
  //Reset variable for next iteration
  previous = error;
  
  //Calculates new output
  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

double emaFilter(double currentRPM, double prevRPM, float currentRPMWeight){
  //EMA Low pass filter for noise reduction
  //Exponential Moving Average: currentRPMWeight = 0.4
  cleanRPM = ((currentRPM) * currentRPMWeight) + (prevRPM * (1 - currentRPMWeight));
  return cleanRPM; 
}

//function that takes the direction, the pwrm, and the two motor pins and changes the speed of motor accordingly. 
void setMotor(int dir, int pwmVal){
  ledcWrite(PWM, pwmVal);  // PWM pin
  if(dir == 1){
    digitalWrite(IN1, HIGH);
  } 
  else if(dir == -1){
    //if opposite direction, revese signal 
    digitalWrite(IN1, LOW);
  } 
  else {
    //defaults to off-state
    //Use in troubleshooting:
    //- if off-state regardless of setpoint, test pid function and rpm calculator
    digitalWrite(IN1, LOW);
  }
}
