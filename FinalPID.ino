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
volatile int64_t latestTimestampMicroseconds = 0;
volatile bool newReadingAvailable = false;



//Previous reading (used to compute change)
int64_t prevEncoderCount = 0;
int64_t prevTimestampMicroseconds = 0;

//RPM Calculator and Filter Params
float prevRPM = 0.0f;
float currentRPMWeight = 0.4; //ema filter decimal weigth
float cleanRPM = 0;
int64_t countChange = 0; 

//PID params
double integral, previous, pidOutput = 0; 
double Kp, Ki, Kd; //for now just following a setup tutorial, we can tune this later
double setpoint = 50;
const double DT_SECONDS = 0.05;


//Periodic-timer Callback
//Function called to get current time and encoder counts for RPM readings.
static void IRAM_ATTR timerCallback(void* arg) {
  latestTimestampMicroseconds = esp_timer_get_time();
  latestEncoderCount = encoder.getCount();  // SNAPSHOT HERE
  newReadingAvailable = true;

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
  int64_t encoderCountSnapshot;
  int64_t timestampSnapshot;
  
  if (!newReadingAvailable) return;
  
  noInterrupts();
  encoderCountSnapshot = latestEncoderCount;
  timestampSnapshot = latestTimestampMicroseconds;
  interrupts();
  //---------------------------------

  //float setpoint = 50; //this changes from jetson comm

  //Calculates the error in the system
  double actualRpm = rpmCalculator(encoderCountSnapshot, timestampSnapshot);// from our rpm calc
  double error = setpoint - actualRpm;
  pidOutput = pid(error, DT_SECONDS);

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

double rpmCalculator(int64_t encoderCount) {

  // Compute change in encoder counts
  countChange = encoderCount - prevEncoderCount;
  prevEncoderCount = encoderCount;  // store for next iteration

  // Compute RPM assuming fixed DT
  float currentRPM = (countChange * 60.0f) / (COUNTS_PER_REVOLUTION * DT_SECONDS);

  // Apply EMA filter
  cleanRPM = fabs(emaFilter(currentRPM, prevRPM, currentRPMWeight));
  prevRPM = cleanRPM;

  // Debug print
  Serial.print("RPM: ");
  Serial.println(cleanRPM);

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
  // Constrain PWM to 0-255 just to be safe
  pwmVal = constrain(pwmVal, 0, 255);
  
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
