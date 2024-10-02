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
const double Kp = 2.0;
const double Ki = 0.5;
const double Kd = 1.0;

// PID variables
double previous_error = 0;
double integral = 0;
double setpoint = 300.0;             // Desired temperature in Celsius
const double integral_limit = 100.0; // Maximum integral value in power %
uint8_t error_flags = 0;
static volatile uint8_t reading_temperature = 0;

#define ERROR_NO_READING 1 // Thermocouple fault
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


uint32_t off_time = 0;
void handlePin10Interrupt() {
  // Your interrupt handling code here
  // For example, toggle an LED or set a flag
  if (digitalRead(PIN_ZERO_CROSS) == HIGH) {
    interrupt_count++;

    if ((interrupt_count % 100) >= (100 - power_percent) && error_flags == 0 &&
        reading_temperature == 0) {
      if (off_time > 0)
        off_time = 0;
      digitalWrite(PIN_HEATER, PIN_HEATER_ON);

    } else {
      if (off_time == 0)
        off_time = millis();
      digitalWrite(PIN_HEATER, PIN_HEATER_OFF);
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
  uint8_t heater_state = digitalRead(PIN_HEATER);
  static uint8_t previous_heater_state = PIN_HEATER_OFF;
  if (heater_state != previous_heater_state) {
    Serial.print("Heater is ");
    Serial.println(heater_state == PIN_HEATER_ON ? "on" : "off");
    previous_heater_state = heater_state;
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
    Serial.print("Intterupt count: ");
    Serial.print(interrupt_count);
    Serial.print(" mod: ");
    Serial.println((interrupt_count % 100));
    Serial.print("Interrupt frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
  }

  // Only read the temperature if heater is off

  // Prevent start up of the mosfet while reading the temperature
  reading_temperature = 1;
  // Require 10ms to settle the voltage
  if (off_time > 0 && current_ms > (off_time + 10)) {
    //unsigned long start_ms = millis();
    int32_t adc_sum = 0;
    for (int i = 0; i < 5; i++) {
      adc_sum += ads.readADC_SingleEnded(0);
    }
    float voltage_measured = ads.computeVolts(adc_sum/5);

    // Calculate thermocouple voltage before amplification
    float thermocouple_voltage = voltage_measured / GAIN; // Voltage in volts

    // Convert thermocouple voltage to temperature in °C
    float reported_temperature =
        thermocouple_voltage * 1e6 / THERMOCOUPLE_SENSITIVITY; // µV/°C to °C

    // Apply polynomial correction to the reported temperature
    float actual_temperature = correct_temperature(reported_temperature);
    //Serial.print("Time to read: ");
    //Serial.print((millis() - start_ms));
    //Serial.println(" ms");



    Serial.print("Off time: ");
    Serial.print((current_ms - off_time));
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
      Serial.print(error);
      Serial.print(" I: ");
      Serial.print(integral);
      Serial.print(" D: ");
      Serial.print(derivative);
      Serial.print(" O: ");
      Serial.println(output);

      // Set power percentage based on PID output
      power_percent = constrain(output, 0, 50);
      // Print the power percentage
      Serial.print("Power: ");
      Serial.println(power_percent);
    }
    reading_temperature = 0;
    delay(1000); // wait for a second
  } else {
    reading_temperature = 0;
    delay(1);
  }
}
