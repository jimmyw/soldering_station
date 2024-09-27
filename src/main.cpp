#include <Arduino.h>
#include <stdint.h>

#include "Adafruit_MAX31855.h"
#include <SPI.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define PIN_MAX_CS 1
#define PIN_MAX_DO 2
#define PIN_MAX_CLK 3
#define PIN_ZERO_CROSS 10
#define PIN_HEATER 8

#define PIN_HEATER_ON LOW
#define PIN_HEATER_OFF HIGH

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(PIN_MAX_CLK, PIN_MAX_CS, PIN_MAX_DO);

uint32_t interrupt_count = 0;
uint32_t power_percent = 50;

// PID constants
const double Kp = 2.0;
const double Ki = 0.5;
const double Kd = 1.0;

// PID variables
double previous_error = 0;
double integral = 0;
double setpoint = 100.0;             // Desired temperature in Celsius
const double integral_limit = 100.0; // Maximum integral value in power %
uint8_t error_flags = 0;

#define ERROR_MAX31855_FAULT_OPEN 1
#define ERROR_MAX31855_FAULT_SHORT_GND 2
#define ERROR_MAX31855_FAULT_SHORT_VCC 4
#define ERROR_BAD_FREQUENCY 8 // No zero cross detected

const char *error_names[] = {
    "MAX31855_FAULT_OPEN",
    "MAX31855_FAULT_SHORT_GND",
    "MAX31855_FAULT_SHORT_VCC",
    "BAD_FREQUENCY",
};

void handlePin10Interrupt() {
  // Your interrupt handling code here
  // For example, toggle an LED or set a flag
  if (digitalRead(PIN_ZERO_CROSS) == HIGH) {
    interrupt_count++;

    if ((interrupt_count % 100) >= power_percent && error_flags == 0)
      digitalWrite(PIN_HEATER, PIN_HEATER_ON);
    else
      digitalWrite(PIN_HEATER, PIN_HEATER_OFF);
  }
}

// put function declarations here:
void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // print "Hello, World!" to the serial monitor:

  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1)
      delay(10);
  }
  pinMode(PIN_HEATER, OUTPUT);
  // Attach the new interrupt to pin 10
  pinMode(PIN_ZERO_CROSS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(10), handlePin10Interrupt, CHANGE);
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
      Serial.print(" ");
    }
  }

}

static void clear_error_flag(uint8_t flag) {
  error_flags &= ~flag;
}

void loop() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  // Calculate frequency in Hz
  unsigned long count = interrupt_count;
  unsigned long interval = currentMillis - previousMillis;
  interrupt_count = 0; // Reset the count
  double frequency = count / (interval / 1000.0);
  previousMillis = currentMillis;

  if (frequency > 40 && frequency < 70) {
    clear_error_flag(ERROR_BAD_FREQUENCY);
  } else {
    set_error_flag(ERROR_BAD_FREQUENCY);
  }

  // Print the frequency
  Serial.print("Interrupt frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  // Print the power percentage
  Serial.print("Power: ");
  Serial.println(power_percent);

  // basic readout test, just print the current temp
  Serial.print("Internal Temp = ");
  Serial.println(thermocouple.readInternal());

  double c = thermocouple.readCelsius();
  if (isnan(c)) {
    Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    if (e & MAX31855_FAULT_OPEN) {
      Serial.println("FAULT: Thermocouple is open - no connections.");
      set_error_flag(ERROR_MAX31855_FAULT_OPEN);
    }
    if (e & MAX31855_FAULT_SHORT_GND) {
      Serial.println("FAULT: Thermocouple is short-circuited to GND.");
      set_error_flag(ERROR_MAX31855_FAULT_SHORT_GND);
    }
    if (e & MAX31855_FAULT_SHORT_VCC) {
      Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
      set_error_flag(ERROR_BAD_FREQUENCY);
    }
  } else {
    clear_error_flag(ERROR_MAX31855_FAULT_OPEN | ERROR_MAX31855_FAULT_SHORT_GND |
                     ERROR_MAX31855_FAULT_SHORT_VCC);
    Serial.print("C = ");
    Serial.println(c);

    // Set power level based on temperature using a PID controller
    // PID controller
    double error = setpoint - c;
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
    power_percent = constrain(output, 0, 100);
  }

  delay(1000); // wait for a second
}
