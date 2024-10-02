#include <Arduino.h>
#include <stdint.h>

#include <Adafruit_ADS1X15.h>
#include <SPI.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define PIN_SCL 1
#define PIN_SDA 3
#define PIN_ZERO_CROSS 7
#define PIN_HEATER 8

#define PIN_HEATER_ON HIGH
#define PIN_HEATER_OFF LOW

// initialize the Thermocouple
Adafruit_ADS1015 ads;

uint32_t interrupt_count = 0;
uint32_t power_percent = 50;

// PID constants
const double Kp = 1.0;
const double Ki = 0.5;
const double Kd = 0.0;

// PID variables
double previous_error = 0;
double integral = 0;
double setpoint = 300.0;             // Desired temperature in Celsius
const double integral_limit = 100.0; // Maximum integral value in power %
uint8_t error_flags = 0;

#define ERROR_NO_READING 1    // Thermocouple fault
#define ERROR_BAD_FREQUENCY 2 // Frequency out of range
const char *error_names[] = {
    "NO_READING",
    "BAD_FREQUENCY",
};

const float GAIN =
    201.0; // Amplification gain from the LTC2063 R1 = 200 kΩ, R2 = 1 kΩ
const float THERMOCOUPLE_SENSITIVITY =
    41.0; // Type K thermocouple sensitivity in µV/°C

// Function to correct the temperature using the polynomial formula
float correct_temperature(float reported_temperature) {
  // Linear fit coefficients based on the fit
  float slope = 1.17912088;
  float intercept = 29.91208791;

  // Apply the linear correction formula
  return (slope * reported_temperature) + intercept;
}

// State machine
// 1. Heating for X interrupt based on power_percent
// 2. Reading temperature
// 3. Idle for X interrupts based on power_percent
// 4. Error state
enum state_e {
  STATE_HEATING,
  STATE_READING_TEMPERATURE,
  STATE_IDLE,
};
static volatile enum state_e state = STATE_HEATING;
static volatile uint32_t state_time = 0;
static volatile uint32_t state_count = 0;

void handlePin10Interrupt() {
  // Your interrupt handling code here
  // For example, toggle an LED or set a flag
  if (digitalRead(PIN_ZERO_CROSS) == HIGH) {
    interrupt_count++;
    state_count++;
    switch (state) {
    case STATE_HEATING: {
      if (error_flags == 0) {
        digitalWrite(PIN_HEATER, PIN_HEATER_ON);
      }
      uint32_t heat_time = power_percent;

      if (state_count >= heat_time) {
        state = STATE_READING_TEMPERATURE;
        state_time = millis();
        state_count = 0;
      }
      break;
    }
    case STATE_READING_TEMPERATURE: {
      digitalWrite(PIN_HEATER, PIN_HEATER_OFF);
      break;
    }
    case STATE_IDLE: {
      digitalWrite(PIN_HEATER, PIN_HEATER_OFF);
      uint32_t idle_time = 100 - power_percent;
      if (state_count >= idle_time) {
        state = STATE_HEATING;
        state_time = millis();
        state_count = 0;
      }
      break;
    }
    }
  }
}

// put function declarations here:
void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // print "Hello, World!" to the serial monitor:

  Serial.println("Hello, World!");

  Wire.begin(PIN_SDA, PIN_SCL);

  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    delay(1000);
  }

  pinMode(PIN_HEATER, OUTPUT);

  // Attach the new interrupt to pin 10
  pinMode(PIN_ZERO_CROSS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), handlePin10Interrupt,
                  CHANGE);
}

static void set_error_flag(uint8_t flag) {
  error_flags |= flag;

  // Turn off the PIN_HEATER
  digitalWrite(PIN_HEATER, PIN_HEATER_OFF);

  // Print the error
  Serial.print("Error: ");
  for (uint8_t i = 0; i < ARRAY_SIZE(error_names); i++) {
    if (flag & (1 << i)) {
      Serial.print(error_names[i]);
      Serial.println("");
    }
  }
}

static void clear_error_flag(uint8_t flag) { error_flags &= ~flag; }

void loop() {

  // Print current heater state
  static enum state_e previous_heater_state;
  if (state != previous_heater_state) {
    Serial.print("Heater is ");
    Serial.println(state == STATE_HEATING ? "HEAT" : state == STATE_READING_TEMPERATURE ? "TEMP" : "IDLE");
    previous_heater_state = state;
  }

  // Calculate frequency in Hz
  static unsigned long previous_ms = 0;
  static uint32_t previous_interrupt_count = 0;
  unsigned long current_ms = millis();
  unsigned long count = interrupt_count - previous_interrupt_count;
  unsigned long interval = current_ms - previous_ms;
  if (interval > 1000.0) {
    double frequency = count / (interval / 1000.0);
    previous_ms = current_ms;
    previous_interrupt_count = interrupt_count;

    if (frequency > 40 && frequency < 70) {
      clear_error_flag(ERROR_BAD_FREQUENCY);
    } else {
      set_error_flag(ERROR_BAD_FREQUENCY);
    }

    // Print the frequency
    //Serial.print("Intterupt count: ");
    //Serial.print(interrupt_count);
    //Serial.print("Interrupt frequency: ");
    //Serial.print(frequency);
    //Serial.println(" Hz");
  }

  // Only read the temperature if heater is off
#define ADC_FILTERING 1

  // Require 200ms to settle the voltage
  if (state == STATE_READING_TEMPERATURE && current_ms > (state_time + 200)) {
    // unsigned long start_ms = millis();
    int32_t adc_sum = 0;
    for (int i = 0; i < ADC_FILTERING; i++) {
      adc_sum += ads.readADC_SingleEnded(0);
    }
    float voltage_measured =
        ads.computeVolts((float)adc_sum / (float)ADC_FILTERING);

    // Calculate thermocouple voltage before amplification
    float thermocouple_voltage = voltage_measured / GAIN; // Voltage in volts

    // Convert thermocouple voltage to temperature in °C
    float reported_temperature =
        thermocouple_voltage * 1e6 / THERMOCOUPLE_SENSITIVITY; // µV/°C to °C

    // Apply polynomial correction to the reported temperature
    float actual_temperature = correct_temperature(reported_temperature);
    // Serial.print("Time to read: ");
    // Serial.print((millis() - start_ms));
    // Serial.println(" ms");

    Serial.print("Off time: ");
    Serial.print((current_ms - state_time));
    Serial.println(" ms");

    // Print results
    Serial.print("Measured Voltage: ");
    Serial.print(voltage_measured, 6);
    Serial.println(" V");

    // Serial.print("Thermocouple Voltage: ");
    // Serial.print(thermocouple_voltage * 1e3, 6);  // Convert back to mV for
    // display Serial.println(" mV");

    Serial.print("Temperature: ");
    Serial.print(actual_temperature, 2);
    Serial.println(" °C");

    if (actual_temperature < 20 || actual_temperature > 400) {
      Serial.println("Thermocouple fault(s) detected!");
      set_error_flag(ERROR_NO_READING);
    } else {
      clear_error_flag(ERROR_NO_READING);

      // Set power level based on temperature using a PID controller
      // PID controller
      double error = setpoint - actual_temperature;
      integral += error;
      integral = constrain(integral, -integral_limit,
                           integral_limit); // Windup protection
      double derivative = error - previous_error;
      double output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;

      // Print all pid values
      Serial.print("P: ");
      Serial.print(Kp * error);
      Serial.print(" I: ");
      Serial.print(Ki * integral);
      Serial.print(" D: ");
      Serial.print(Kd * derivative);
      Serial.print(" O: ");
      Serial.println(output);

      // Set power percentage based on PID output
      power_percent = constrain(output, 0, 98);
      // Print the power percentage
      Serial.print("Power: ");
      Serial.println(power_percent);
    }
    state = STATE_IDLE;
  }
  delay(1);
}
