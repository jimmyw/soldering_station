#include <Arduino.h>
#include <stdint.h>

#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Wire.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define PIN_SCL 1
#define PIN_SDA 3
#define PIN_ZERO_CROSS 7
#define PIN_HEATER 8
#define PIN_DETECT 6
#define LCD_SCL 21
#define LCD_SDA 20

#define PIN_HEATER_ON HIGH
#define PIN_HEATER_OFF LOW

#define TEMP_DELAY_MS 50

// initialize the Thermocouple
Adafruit_ADS1015 ads;

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(/* rotation =*/U8G2_R0,
                                          /* clock=*/LCD_SCL,
                                          /* data=*/LCD_SDA,
                                          /* reset=*/U8X8_PIN_NONE);

uint32_t frequency_count = 0;
float power = 0.0; // Number between 0.0 and 1.0

// PID constants
const float Kp = 0.5;
const float Ki = 0.5;
const float Kd = 0.5;

// PID variables
float previous_error = 0;
float integral = 0;
float heater_setpoint = 350.0;      // Desired temperature in Celsius
const float integral_limit = 100.0; // Maximum integral value in power %
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
// 1. Heating for X interrupt based on power
// 2. Reading temperature
// 3. Idle for X interrupts based on power
// 4. Error state
enum state_e {
  STATE_HEATING,
  STATE_READING_TEMPERATURE,
  STATE_IDLE,
};
static volatile enum state_e state = STATE_HEATING;
static volatile uint32_t state_time =
    0; // Time in milliseconds since state change
static volatile uint32_t state_count = 0; // Number of interrupts in the state
static uint32_t full_cycle_count =
    200; // The amount of half cycles we will max do before taking a temperature
         // meassurement 200 half cycles = 1 full second at 50Hz
static float actual_temperature = 0;

void handlePin10Interrupt() {

  if (digitalRead(PIN_ZERO_CROSS) == LOW) {
    frequency_count++;
  }

  state_count++;
  switch (state) {
  case STATE_HEATING: {
    if (error_flags == 0) {
      digitalWrite(PIN_HEATER, PIN_HEATER_ON);
    }
    const uint32_t heat_time = round(power * full_cycle_count);

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
    const uint32_t idle_time = round((1.0 - power) * full_cycle_count);
    if (state_count >= idle_time) {
      state = STATE_HEATING;
      state_time = millis();
      state_count = 0;
    }
    break;
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

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Hello, World!");
  u8g2.sendBuffer();

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

  // Pin detect is to see if iron is in the stand. active low, pullup needed
  pinMode(PIN_DETECT, INPUT_PULLUP);
}

static void set_error_flag(uint8_t flag) {
  error_flags |= flag;

  // Turn off the PIN_HEATER
  digitalWrite(PIN_HEATER, PIN_HEATER_OFF);
  if (state == STATE_HEATING) {
    state = STATE_IDLE;
    state_time = millis();
    state_count = 0;
  }

  // Print the error
  Serial.print("Error: ");
  for (uint8_t i = 0; i < ARRAY_SIZE(error_names); i++) {
    if (flag & (1 << i)) {
      Serial.print(error_names[i]);
      Serial.println("");
    }
  }
}

static float target_temp() {
  if (digitalRead(PIN_DETECT) == LOW) {
    return 150.0;
  }
  return heater_setpoint;
}

static void render_lcd() {
  u8g2.firstPage();
  do {
    if (error_flags) {
      u8g2.setFont(u8g2_font_ncenR08_tr);
      u8g2.setCursor(0, 10);
    } else {
      u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(0, 12);

    }
    u8g2.print(actual_temperature, 2);
    u8g2.print(" -> ");
    u8g2.print(target_temp(), 0);
    u8g2.print("C");
    u8g2.setCursor(0, 20);
    for (uint8_t i = 0; i < ARRAY_SIZE(error_names); i++) {
      if (error_flags & (1 << i)) {
        u8g2.print(error_names[i]);
        u8g2.print(" ");
      }
    }

    // Plot a power bar in the bottom of the screen
    const int bar_x = 0;
    const int bar_width = u8g2.getDisplayWidth();
    const int bar_height = 5;
    const int bar_y = u8g2.getDisplayHeight() - bar_height; // 32 - 5 = 27

    // Calculate the width of the filled part of the bar
    int filled_width = (power * bar_width);

    // Draw the bar outline
    u8g2.drawFrame(bar_x, bar_y, bar_width, bar_height);

    // Draw the filled part of the bar
    u8g2.drawBox(bar_x, bar_y, filled_width, bar_height);


  } while (u8g2.nextPage());
}


static void clear_error_flag(uint8_t flag) { error_flags &= ~flag; }

void loop() {
  // Print current heater state
  static enum state_e previous_heater_state;
  if (state != previous_heater_state) {
    Serial.print("Heater is ");
    Serial.println(state == STATE_HEATING               ? "HEAT"
                   : state == STATE_READING_TEMPERATURE ? "TEMP"
                                                        : "IDLE");
    previous_heater_state = state;
  }

  // Calculate frequency in Hz
  static unsigned long previous_ms = 0;
  static uint32_t previous_frequency_count = 0;
  unsigned long current_ms = millis();
  unsigned long count = frequency_count - previous_frequency_count;
  unsigned long interval = current_ms - previous_ms;
  if (interval > 1000.0) {
    float frequency = count / (interval / 1000.0);
    previous_ms = current_ms;
    previous_frequency_count = frequency_count;

    if (frequency > 40 && frequency < 70) {
      clear_error_flag(ERROR_BAD_FREQUENCY);
    } else {
      set_error_flag(ERROR_BAD_FREQUENCY);
    }

    // Print the frequency
    // Serial.print("Intterupt count: ");
    // Serial.print(frequency_count);
    // Serial.print("Interrupt frequency: ");
    // Serial.print(frequency);
    // Serial.println(" Hz");
  }

  // Only read the temperature if heater is off
#define ADC_FILTERING 1

  // Require TEMP_DELAY_MS to settle the voltage
  if ((state == STATE_READING_TEMPERATURE && current_ms >= (state_time + TEMP_DELAY_MS)) || (state == STATE_IDLE && current_ms >= (state_time + 5000))) {
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

    // Print results
    Serial.print("Measured Voltage: ");
    Serial.print(voltage_measured, 6);
    Serial.print(" V with gain: ");
    Serial.print(thermocouple_voltage * 1e3, 6);  // Convert back to mV for
    Serial.println(" mV");

    // Apply polynomial correction to the reported temperature
    actual_temperature = correct_temperature(reported_temperature);
    Serial.print("Temperature: ");
    Serial.print(reported_temperature, 2);
    Serial.print(" °C corrected: ");
    Serial.print(actual_temperature, 2);
    Serial.println(" °C");


    float setpoint = target_temp();
    Serial.print("Temperature: ");
    Serial.print(actual_temperature, 2);
    Serial.print(" °C --> ");
    Serial.print(setpoint, 2);
    Serial.println(" °C");

    if (actual_temperature < 20 || actual_temperature > 400) {
      Serial.println("Thermocouple fault(s) detected!");
      set_error_flag(ERROR_NO_READING);
    } else {
      clear_error_flag(ERROR_NO_READING);

      // Set power level based on temperature using a PID controller
      // PID controller
      float error = setpoint - actual_temperature;
      integral += error;
      integral = constrain(integral, 0,
                           integral_limit); // Windup protection
      float derivative = error - previous_error;
      float output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;

      // Print all pid values
      Serial.print("P: ");
      Serial.print(Kp * error);
      Serial.print(" I: ");
      Serial.print(Ki * integral);
      Serial.print(" D: ");
      Serial.print(Kd * derivative);
      Serial.print(" O: ");
      Serial.print(output);

      // Set power percentage based on PID output (0.0 - 1.0)
      power = constrain(output, 0.0, 100.0) / 100.0;
      // Print the power percentage
      Serial.print(" Power: ");
      Serial.println(power);
    }
    state = STATE_IDLE;
  }

  render_lcd();

  delay(1);
}
