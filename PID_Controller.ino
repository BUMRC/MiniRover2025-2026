#include <RPMCalc.h> 
#include //wtv the jetson is telling
#define IN1 6
#define IN2 7
#define PWM 5

double dt, last_time;
double integral, previous, pidOutput = 0; 
//PID params
double Kp, Ki, Kd; //for now just following a setup tutorial, we can tune this later
double setpoint = 10;

void setup() {
  // put your setup code here, to run once:
  Kp = 0;
  Ki = 0;
  Kd = 0;
  Serial.begin(9600);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  //idk random rpm right now
  setpoint = 10;
  for(int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
  }

}

void loop() {
 //Timer keeping track of time elapsed.  
  double now = millis();
  dt = (now-last_time)/1000.00;
  last_time = now;

  float setpoint = //we need to request a setpoint

  //Calculates the error in the system
  double actualRpm = motorRPM;// from our rpm calc
  double error = setpoint - actualRPM;
  pidOutput = pid(error, dt);

  //Where voltage and direction get set
  int dir = 1;
  if (output<0){ dir=-1; }
  //pwr is the new voltage, which is the absolute value of the output. 
  int pwr = (int) fabs(output);
  if(pwr > 255) pwr = 255;
  setMotor(dir,pwr,PWM,IN1,IN2);

  //print out values
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print(", Actual: "); Serial.print(actualRpm);
  Serial.print(", Error: "); Serial.println(error);

}

double pid(double error, double dt){
  //PID calculation
  //First gets the proportional part of PID, which is just the error.
  double proportional = error;

  // The integral part sums up all the error and multiplies it by change in time. (Rough approx)
  // add clamping
  integral += error * dt;
  //derivative finds the slope of the errors (Rough approx)
  double derivative = (error - previous) / dt;
  //creates the variable derivativeLPF which is used as the refrence point in LPF calculation
  //set equal to 0 only once. 
  static double derivativeLPF = 0;
  //weight is what effects how much the real derivative effects the value we pass to the PID. 0.1 is used because it offers a smooth but not too smooth value. 
  double weight = 0.1; //this val can be tuned
  //LPF is the preivous lpf times the difference in previous derivative and current derivative multiplied by a small amount. 
  derivativeLPF = derivativeLPF + weight * (rawDerivative - derivativeLPF);
  double derivative  = derivativeLPF
  previous = error;
  //Calculates new output
  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

//function that takes the direction, the pwrm, and the two motor pins and changes the speed of motor accordingly. 
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir==1){ digitalWrite(in1,HIGH); digitalWrite(in2,LOW); }
  else if(dir==-1){ digitalWrite(in1,LOW); digitalWrite(in2,HIGH); }
  else{ digitalWrite(in1,LOW); digitalWrite(in2,LOW); }
}
