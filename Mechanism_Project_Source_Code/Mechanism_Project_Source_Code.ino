/*
 * PROJECT: Robot Joint Control 
 * * DESCRIPTION:
 * This system implements a PID-based position control for a robotic joint.
 * * STATUS INDICATORS:
 * - RED LED   : Far from target (Error > 15 degrees) - High Speed/Coarse Adjustment
 * - YELLOW LED: Approaching (2 < Error < 15 degrees) - Fine Adjustment
 * - GREEN LED : Target Reached (Error <= 2 degrees)  - Position Locked
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- PIN ASSIGNMENTS ---
#define PHASE_A 2       // Encoder Output A
#define PHASE_B 3       // Encoder Output B
#define MOTOR_PWM 9     // Motor PWM Speed Control
#define AIN1 8          // Motor Direction Input 1
#define AIN2 7          // Motor Direction Input 2
#define POT_PIN A0      // Potentiometer for Setpoint

// --- STATUS LED INDICATORS ---
#define RED_LED 5       // Mode: Far / Fast Response
#define GREEN_LED 6     // Mode: Target Reached / Locked
#define YEL_LED 10      // Mode: Approaching / Precision

// --- SPEED & SAFETY CONSTRAINTS ---
const int MIN_SPEED = 70;      // Minimum PWM duty cycle to overcome friction
const int MAX_SPEED = 180;     // Maximum PWM duty cycle limit
const int MECHANICAL_LIMIT = 180; // Physical range of motion in degrees

// --- PID CONTROL PARAMETERS ---
float Kp = 3.5;    // Proportional Gain
float Ki = 0.15;   // Integral Gain
float Kd = 4.0;    // Derivative Gain

// --- GLOBAL VARIABLES & OBJECTS ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile long encoderCount = 0;   // Encoder ticks (modified in ISR)
float currentAngle = 0.0;
float targetAngle = 0.0;
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Input/Output Configuration
  pinMode(PHASE_A, INPUT_PULLUP);
  pinMode(PHASE_B, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  // LED Configuration
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YEL_LED, OUTPUT);
  
  // Startup Animation (System Health Check)
  digitalWrite(RED_LED, HIGH); delay(200); digitalWrite(RED_LED, LOW);
  digitalWrite(YEL_LED, HIGH); delay(200); digitalWrite(YEL_LED, LOW);
  digitalWrite(GREEN_LED, HIGH); delay(200); digitalWrite(GREEN_LED, LOW);

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  
  // Interrupt Attachment for Encoder
  attachInterrupt(digitalPinToInterrupt(PHASE_A), readEncoder, RISING);
}

void loop() {
  unsigned long currentTime = millis();

  // 1. READ SETPOINT
  int potValue = analogRead(POT_PIN);
  targetAngle = map(potValue, 0, 1023, 0, MECHANICAL_LIMIT); 

  // 2. PID CONTROL LOOP (Sampling Interval: 10ms)
  if (currentTime - lastTime >= 10) {
    long currentPosition = encoderCount;
    currentAngle = currentPosition; // Assuming 1 tick = 1 degree for this prototype

    float error = targetAngle - currentAngle;
    float absError = abs(error); // Magnitude of error

    // --- INTEGRAL ANTI-WINDUP STRATEGY ---
    // Reset integral on zero-crossing to prevent overshoot
    if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) integral = 0;

    // Accumulate integral only when close to target to avoid saturation
    if (absError < 10) integral += error * (currentTime - lastTime);
    else integral = 0; 
    
    // Clamp integral value
    if (integral > 300) integral = 300;
    if (integral < -300) integral = -300;

    // --- STATE MACHINE: LED INDICATORS & CONTROL LOGIC ---

    // STATE 1: TARGET REACHED (Green Zone)
    // Deadband is set to 2 degrees to prevent oscillation
    if (absError <= 2) {
       error = 0;
       integral = 0;
       
       digitalWrite(GREEN_LED, HIGH);
       digitalWrite(YEL_LED, LOW);
       digitalWrite(RED_LED, LOW);
       
       stopMotorHard(); // Engage electronic braking
    } 
    // STATE 2: APPROACHING (Yellow Zone)
    // Fine control region
    else if (absError < 15) {
       digitalWrite(GREEN_LED, LOW);
       digitalWrite(YEL_LED, HIGH); 
       digitalWrite(RED_LED, LOW);
       
       runPID(error, currentTime); // Execute PID control
    }
    // STATE 3: FAR FROM TARGET (Red Zone)
    // Coarse control region
    else {
       digitalWrite(GREEN_LED, LOW);
       digitalWrite(YEL_LED, LOW);
       digitalWrite(RED_LED, HIGH); 
       
       runPID(error, currentTime); // Execute PID control
    }

    previousError = error;
    lastTime = currentTime;
  }
  
  // 3. UPDATE DISPLAY (Refresh Rate: 250ms)
  static unsigned long printTime = 0;
  if (millis() - printTime > 250) {
    lcd.setCursor(0, 0); lcd.print("Target  Angle:"); lcd.print((int)targetAngle); lcd.print("   ");
    lcd.setCursor(0, 1); lcd.print("Current Angle:"); lcd.print((int)currentAngle); lcd.print("   ");
    printTime = millis();
  }
}

// --- HELPER FUNCTIONS ---

// Calculates PID output and drives the motor
void runPID(float error, unsigned long currentTime) {
    float pTerm = Kp * error;
    // Calculate derivative term based on change in error over time
    float dTerm = Kd * (error - previousError) / (currentTime - lastTime);
    
    float controlSignal = pTerm + (Ki * integral) + dTerm;
    setMotorSpeed(controlSignal);
}

// Sets motor direction and speed with constraints
void setMotorSpeed(float speed) {
  int pwmVal = (int)fabs(speed);
  
  // Apply saturation limits
  if (pwmVal > MAX_SPEED) pwmVal = MAX_SPEED;
  if (pwmVal < MIN_SPEED) pwmVal = MIN_SPEED;

  // Determine direction
  if (speed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  }
  analogWrite(MOTOR_PWM, pwmVal);
}

// Applies active braking (Electronic Stop)
void stopMotorHard() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(MOTOR_PWM, 0); 
}

// Interrupt Service Routine (ISR) for Encoder
void readEncoder() {
  if (digitalRead(PHASE_B) == LOW) encoderCount--; 
  else encoderCount++; 
}