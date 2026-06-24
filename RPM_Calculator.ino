#include <ESP32Encoder.h>
#include <esp_timer.h>

#define CHANNEL_A 11
#define CHANNEL_B 12

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

//Previous reading (used to compute change)
int64_t prevEncoderCount = 0;
int64_t prevTimestampMicroseconds = 0;

int prevRPM = 0;

//Periodic-timer Callback
static void IRAM_ATTR timerCallback(void* arg) {
  latestTimestampMicroseconds = esp_timer_get_time();
  latestEncoderCount = encoder.getCount();
  newReadingAvailable = true;
}



void setup() {
  Serial.begin(115200);
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
}


void loop() {

  if (!newReadingAvailable) return;
  
  //Pause the rest of the code to refresh start and end encodercount and seconds
  noInterrupts();
  int64_t currentEncoderCount = latestEncoderCount;
  int64_t currentTimestampMicroseconds = latestTimestampMicroseconds;
  newReadingAvailable = false;
  interrupts();

  //Compute changes in encoder count and time
  int64_t countChange = currentEncoderCount - prevEncoderCount;
  int64_t timeChangeMicroseconds = currentTimestampMicroseconds - prevTimestampMicroseconds;

  //Only calculate RPM when there's new data
  if (timeChangeMicroseconds <= 0) {
    prevEncoderCount = currentEncoderCount;
    prevTimestampMicroseconds = currentTimestampMicroseconds;
    return;
  }

  //Integer RPM calculation:
  //RPM = (countChange * 100,000) / (COUNTS_PER_REVOLUTION * timeChangeMicroseconds)
  int64_t numerator = countChange * 100000LL;
  int64_t denominator = (int64_t)COUNTS_PER_REVOLUTION * timeChangeMicroseconds;

  bool isNegative = false;
  if (numerator < 0) {
    isNegative = true;
    numerator = -numerator;
  }

  //Compute RPM*10 (one decimal place)
  int64_t rpmTimes10 = (numerator * 10 + denominator / 2) / denominator;
  //Exponential Moving Average: currentRPM_Weight = 0.4
  currentRPMWeight = 0.4;
  cleanRPM = (rpmTimes10 * 10)currentRPMWeight + (prevRPM * (1 - currentRPMWeight));


  if (isNegative) rpmTimes10 = -rpmTimes10;

  Serial.print("RPM: ");
  Serial.print(rpmTimes10 / 10);
  Serial.print('.');
  Serial.println(llabs(rpmTimes10 % 10));

  //Store for next reading
  prevEncoderCount = currentEncoderCount;
  prevTimestampMicroseconds = currentTimestampMicroseconds;
  prevRPM = rpmTimes10 * 10;
}
