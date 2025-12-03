#include <ESP32Encoder.h>
#include <esp_timer.h>

#define CHANNEL_A 11 //yellow wire
#define CHANNEL_B 12 //green wire
#define IN1 47 //yellow wire
#define IN2 48 //yellow wire
#define PWM 5 //green wire

ESP32Encoder encoder;

//Encoder specification
const float ENCODER_PPR = 751.8f;
const int QUADRATURE_MULTIPLIER = 4; //We're cointing 4 edges/signals per click on the encoder(higher-res)
const long COUNTS_PER_REVOLUTION = (long)(ENCODER_PPR * QUADRATURE_MULTIPLIER);

//How often we read the encoder (100 ms)
const uint64_t READ_INTERVAL_MICROSECONDS = 100000ULL;

//These values change spontaneously
volatile int64_t latestEncoderCount = 0;
volatile int64_t latestTimestampMicroseconds = 0;
volatile bool newReadingAvailable = false;
volatile int64_t countChange = 0; 

//Previous reading (used to compute change)
int64_t prevEncoderCount = 0;
int64_t prevTimestampMicroseconds = 0;

int prevRPM = 0;
float currentRPMWeight = 0.4;
float cleanRPM = 0;

//Periodic-timer Callback
//Function called to get current time and encoder counts for RPM readings.
static void IRAM_ATTR timerCallback(void* arg) {
  latestTimestampMicroseconds = esp_timer_get_time();
  latestEncoderCount = encoder.getCount();
  newReadingAvailable = true;
}

double dt, last_time;
double integral, previous, pidOutput = 0; 
//PID params
double Kp, Ki, Kd; //for now just following a setup tutorial, we can tune this later
double setpoint = 50;

void setup() {
  //PID set up stuff
  Kp = 1;
  Ki = 0.1;
  Kd = 0.01;

  last_time = millis();
  integral = 0;
  previous = 0;
  //RPM Calc Setup
  Serial.begin(9600);
  Serial.println("Encoder RPM - cleaned variable names");

  pinMode(CHANNEL_A, INPUT_PULLUP);
  pinMode(CHANNEL_B, INPUT_PULLUP);

  encoder.attachFullQuad(CHANNEL_A, CHANNEL_B);
  encoder.clearCount();
  delay(10);

  prevTimestampMicroseconds = esp_timer_get_time();
  prevEncoderCount = encoder.getCount();

  //Create periodic timer that will run on the esp32, using the callback from timerCallback
  const esp_timer_create_args_t timerArgs = {
    .callback = &timerCallback,
    .name = "encoder_reader"
  };

  esp_timer_handle_t periodicTimer;
  esp_timer_create(&timerArgs, &periodicTimer);
  esp_timer_start_periodic(periodicTimer, READ_INTERVAL_MICROSECONDS);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
 
  // setpoint = 50;
  // for(int i = 0; i < 50; i++)
  // {
  //   Serial.print(setpoint);
  //   Serial.print(",");
  //   Serial.println(0);
  // }
  
  //Set up LED control, the way esps can write to the motor.
  ledcSetup(0, 20000, 8);
  ledcAttachPin(PWM, 0); // 0 is placeholder for what pin we attach the 
   // 20 kHz, 8-bit resolution

}

void loop() {

 if (!newReadingAvailable) return;
  
  //float setpoint = 50; //this changes from jetson comm

  //Calculates the error in the system
  double actualRpm = rpmCalculator();// from our rpm calc
  double error = setpoint - actualRpm;
  pidOutput = pid(error, dt);

  //Scale / constrain PID output to 0–255 for PWM
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

}

double rpmCalculator() {
  //Pause the rest of the code to refresh start and end encodercount and seconds
  //The periodic timer runs asynchronously on the esp32, which can cause interrupts
  noInterrupts();
  int64_t currentEncoderCount = latestEncoderCount;
  int64_t currentTimestampMicroseconds = latestTimestampMicroseconds;
  newReadingAvailable = false;
  interrupts();

  //Compute changes in encoder count and time
  countChange = currentEncoderCount - prevEncoderCount;
  int64_t timeChangeMicroseconds = currentTimestampMicroseconds - prevTimestampMicroseconds;

  //Only calculate RPM when there's new data
  //Use 10 microsecond intervals to reduce overheads on the esp32
  if (timeChangeMicroseconds > 0) { //Zero-division guard
    
    prevEncoderCount = currentEncoderCount;
    prevTimestampMicroseconds = currentTimestampMicroseconds;
  }

  //Integer RPM calculation:
  //RPM = (countChange * 60,000,000) / (COUNTS_PER_REVOLUTION * timeChangeMicroseconds)
  int64_t numerator = countChange * (60LL * 1000000LL);
  int64_t denominator = (int64_t)COUNTS_PER_REVOLUTION * timeChangeMicroseconds;

  bool isNegative = false;
  if (numerator < 0) {
    isNegative = true;
    numerator = -numerator;
  }

  //convert to RPM
  int64_t currentRPM = numerator/ denominator; //returns unfiltered RPM 

  //Clean with low-pass ema filter
  cleanRPM = emaFilter(currentRPM, prevRPM, currentRPMWeight);
  
  if (isNegative) currentRPM = -currentRPM;

  Serial.print("RPM: ");
  Serial.print(currentRPM);
  Serial.print('.');
  Serial.println(llabs(currentRPM % 10));

  //Store for next reading
  prevRPM = currentRPM;

 //Timer keeping track of time elapsed.  
  double now = millis();
  dt = (now-last_time)/1000.00;
  last_time = now;
  
  return actualRPM;
}

double pid(double error, double dt){
  //PID calculation
  //First gets the proportional part of PID, which is just the error.
  double proportional = error;

  //The integral part sums up all the error and multiplies it by change in time. (Rough approx)
  integral += error * dt;
  
  //derivative finds the slope of the errors (Rough approx)
  double derivative = dt > 0 ? (error - previous) / dt : 0; // defaults to 0 
  
  //low pass filter
  previous = error;
  
  //Calculates new output
  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

double emaFilter(double currentRPM, double prevRPM, float currentRPMWeight){
  //EMA Low pass filter for noise reduction
  //Exponential Moving Average: currentRPM_Weight = 0.4
  cleanRPM = ((currentRPM) * currentRPMWeight) + (prevRPM * (1 - currentRPMWeight));
  return cleanRPM; 
}

//function that takes the direction, the pwrm, and the two motor pins and changes the speed of motor accordingly. 
void setMotor(int dir, int pwmVal){
  ledcWrite(0, pwmVal);  // channel 0
  if(dir == 1){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } 
  else if(dir == -1){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
