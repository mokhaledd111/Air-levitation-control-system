/* * Air Levitation Control System
 * Targets: 10kHz PWM, I2C LCD, 100Hz PID Loop, Serial Plotting
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

/*** Pin Definitions ***/
#define PWM_PIN   9    // Fan PWM (Timer 1)
#define TRIG_PIN  10   // Ultrasonic Trig
#define ECHO_PIN  11   // Ultrasonic Echo
#define PIN_DIR_A 2    // Motor Direction A (HIGH)
#define PIN_DIR_B 3    // Motor Direction B (LOW)

/*** PID Parameters (From Simulink Parallel Form) ***/
float P = 5.0;
float I = 23.0;
float D = 0.2;
float Ts = 0.01; // Sampling time 0.01s [cite: 82]

// Discrete coefficients [cite: 78, 117]
float alpha = I * Ts;  
float beta  = D / Ts;

/*** State Variables [cite: 119-135] ***/
float ek = 0;    
float ek_1 = 0;  
float pk_1 = 0;  
float sk = 50.0; // Set-point (Target height in cm)
float yk = 0;    // Feedback signal
float distance_meters = 0;
float val = 0;   // PID total output
int uk = 0;      // Final PWM count (0-1600)

/*** Signal Path Constants ***/
const float GAIN_SENSOR     = 139.785;
const float OFFSET_SENSOR   = 12.935;
const float OFFSET_FEEDBACK = 94.0;
const float GAIN_OUTPUT     = 0.025;

// LCD Instance (Address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // 1. Digital Pin Setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  // Set Motor Direction as requested
  digitalWrite(PIN_DIR_A, HIGH); 
  digitalWrite(PIN_DIR_B, LOW);  
  
  // Start Serial (Match your monitor baud rate)
  Serial.begin(115200);

  // 2. LCD Initialization
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Levitation Sys");

  // 3. Timer 1: 10kHz High-Speed PWM on Pin 9
  TCCR1A = _BV(COM1A1) | _BV(WGM11); 
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); 
  ICR1 = 1600; // Period for 10kHz

  // 4. Timer 2: Precise 0.01s Sampling Interrupt [cite: 88, 222-225]
  cli(); 
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK2 = 0x01; // Enable overflow interrupt
  TCNT2 = 157;   // Load value for 0.01s delay [cite: 90]
  sei(); 
}

// ISR: High-priority PID Math (Runs every 0.01s) [cite: 255-298]
ISR(TIMER2_OVF_vect) {
  // Read sensor
  distance_meters = Read_Sensor_Meters();
  
  // Process Feedback (The "Feed")
  yk = OFFSET_FEEDBACK - ((distance_meters * GAIN_SENSOR) - OFFSET_SENSOR);

  // Error Calculation [cite: 265]
  ek = sk - yk;

  // Discrete PID Math [cite: 73-80, 270-274]
  float wk = P * ek;                // Proportional
  float qk = beta * (ek - ek_1);    // Derivative
  float pk = (alpha * ek) + pk_1;    // Integral

  // Total Control Signal with Output Gain
  val = (wk + pk + qk) * GAIN_OUTPUT;

  // PWM Mapping (Scale 0-100 to 0-1600 count)
  uk = constrain(map(val, 0, 100, 0, 1600), 0, 1600);
  OCR1A = uk;

  // Update states for next cycle [cite: 293-295]
  pk_1 = pk;
  ek_1 = ek;
  TCNT2 = 157; // Reset timer [cite: 297]
}

void loop() {
  // 1. DATA FOR SERIAL PLOTTER
  // Format: Setpoint, Feedback
  Serial.print(sk); 
  Serial.print(",");
  Serial.println(yk); 

  // 2. LCD UPDATE (Every 400ms to prevent flickering)
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate > 400) {
    lcd.setCursor(0,1);
    lcd.print("S:"); lcd.print((int)sk);
    lcd.print("cm Feed:"); lcd.print((int)yk);
    lcd.print("  "); // Buffer to clear old digits
    lastLCDUpdate = millis();
  }
}

// Ultrasonic Sensor Driver
float Read_Sensor_Meters() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); 
  if (duration == 0) return 0;
  return (duration / 1000000.0 * 343.0) / 2.0; // Distance in meters
}