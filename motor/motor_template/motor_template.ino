#include <ESP32Encoder.h>
#define BIN_1 26
#define BIN_2 25
#define LED_PIN 13
#define BTN 12
#define POT 14

ESP32Encoder encoder;

int omegaSpeed = 0;
int omegaDes = 0;
int D = 0;
byte state = 0;


//Setup interrupt variables ----------------------------
volatile int count = 0; // encoder count
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
volatile bool buttonIsPressed = false;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
    buttonIsPressed = true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(BTN, INPUT); // configures the specified pin to behave either as an input or an output
  pinMode(POT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // sets the initial state of LED as turned-off
  attachInterrupt(BTN, isr, RISING);

  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

  // initilize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 2000000, true); // 2000000 * 1 us = 2 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (state) {
    
    case 0 : // motor is stopped
      if (buttonPressEvent()) {
        state = 1;
        startMotorResponse();
      }
      break;

    case 1 : // motor running
      if (deltaT) {
        portENTER_CRITICAL(&timerMux1);
        deltaT = false;
        portEXIT_CRITICAL(&timerMux1);
        
        omegaSpeed = count; 

        //EDIT THIS SECTION FOR A5 ASSIGNMENT
        //For A5 you will need to decide how to define omegaDes
        //It is recommended to measure the maximum speed (in counts per time loop)
        //while under no-load when NOM_PWM_VOLTAGE is applied.
        //Then, take this term and use it to compute the difference between
        //real and desired speed. Use this difference to implement
        //the various control strategies in A5.

        //Stand-in mapping between the pot reading and motor command.
        D = map(analogRead(POT), 0, 4095, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);
        // omegaDes = map(analogRead(POT), 0, 4095, -omega_max, omega_max); // for bang-bang and feedback control, please specify omega_max value.
        
        //END A5 CONTROL SECTION

        //Ensure that you don't go past the maximum possible command
        if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
        }
        else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
        }

      //Map the D value to motor directionality
      //Assumes encoder direction is same as GSI
      if (D > 0) {
        ledcWrite(ledChannel_1, LOW);
        ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
        ledcWrite(ledChannel_2, LOW);
        ledcWrite(ledChannel_1, -D);
      }
      else {
        ledcWrite(ledChannel_2, LOW);
        ledcWrite(ledChannel_1, LOW);
      }

      plotControlData();
      }
      
      if (buttonPressEvent()) {
      state = 0;
      stopMotorResponse();
      }
      break;
      
      default: // should not happen
      Serial.println("SM_ERROR");
      break;
  }
}

//Event Checkers
bool buttonPressEvent() {
  if (buttonIsPressed == true){
    buttonIsPressed = false;
    return true;
  }
  else {
    return false;
  }
}

//Event Service Responses
void stopMotorResponse() {
  ledcWrite(ledChannel_2, LOW);
  ledcWrite(ledChannel_1, LOW);
  digitalWrite(LED_PIN, LOW);
}

void startMotorResponse() {
  ledcWrite(ledChannel_1, LOW);
  ledcWrite(ledChannel_2, D);
  digitalWrite(LED_PIN, HIGH);
}

//Other functions

void plotControlData() {
  Serial.println("Speed, Desired_Speed, PWM_Duty");
  Serial.print(omegaSpeed);
  Serial.print(" ");
  Serial.print(omegaDes);
  Serial.print(" ");
  Serial.println(D/10);  //PWM is scaled by 1/10 to get more intelligible graph
}
