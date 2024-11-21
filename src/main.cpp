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
#define PIN_ENCODER_A 9
#define PIN_ENCODER_B 10
#define PIN_ENCODER_SW 4

#define PIN_HEATER_ON HIGH
#define PIN_HEATER_OFF LOW

#define DEBOUNCE_DELAY 100 // Define key debounce delay in milliseconds

const float GAIN =
    201.3; // Amplification gain from the LTC2063 R1 = 200.5 kΩ, R2 = 1.001 kΩ
// const float THERMOCOUPLE_SENSITIVITY =
//     41.0; // Type K thermocouple sensitivity in µV/°C

const float THERMOCOUPLE_SENSITIVITY =
    42.85; // Type K thermocouple sensitivity in µV/°C

// Time to wait after switching of heater before reading the temperature
#define TEMP_DELAY_MS 20

// Samples to read for the ADC
#define ADC_FILTERING 3

#define STANDBY_TIMEOUT (5 * 60000) // 5 minutes

// initialize the Thermocouple
Adafruit_ADS1015 ads;

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(/* rotation =*/U8G2_R0,
                                            /* clock=*/LCD_SCL,
                                            /* data=*/LCD_SDA,
                                            /* reset=*/U8X8_PIN_NONE);

static uint32_t frequency_count = 0;
static float power = 0.0; // Number between 0.0 and 1.0

// PID constants
static const float Kp = 1.0; // In practice means 1% power increase per degree,
                             // so 100 degrees difference = 100% power
static const float Ki =
    0.2; // Integral gain 0.5% per second, so 100 seconds can adjust 50% power
static const float Kd = 0.2;

// PID variables
static float previous_error = 0;
static float integral = 0;
static volatile float heater_setpoint = 350.0; // Desired temperature in Celsius
static const float integral_limit = 100.0; // Maximum integral value in power %
static uint8_t error_flags = 0;
static uint32_t last_standby_time = 0;
static bool last_in_stand = false;

void hmiTask(void *pvParameters);
void lcdTask(void *pvParameters);

#define ERROR_NO_READING 1    // Thermocouple fault
#define ERROR_BAD_FREQUENCY 2 // Frequency out of range
#define ERROR_STAND_BY 4 // Iron have not been used for a while and shut down
const char *error_names[] = {
    "NO_READING",
    "BAD_FREQUENCY",
    "STAND_BY",
};

// Calibrate the temperature sensor
// Calibration points is measured at 150°C and 350°C
float calibrate_temperature(float reported_temperature) {

  float measserd_temperature_150 = 175;
  float measserd_temperature_350 = 430;

  // Calculate the slope and intercept
  float slope =
      (measserd_temperature_350 - measserd_temperature_150) / (350.0 - 150.0);
  float intercept = measserd_temperature_150 - slope * 150.0;

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
    100; // The amount of half cycles we will max do before taking a temperature
         // meassurement 200 half cycles = 1 full second at 50Hz
static float actual_temperature = 0;

static void goState(enum state_e new_state) {
  state = new_state;
  state_time = millis();
  state_count = 0;
}

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
      goState(STATE_READING_TEMPERATURE);
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
      if (error_flags == 0) {
        goState(STATE_HEATING);
      } else {
        goState(STATE_READING_TEMPERATURE);
      }
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

  // Initialize LCD
  u8g2.begin();

  Serial.print("Initializing ADC... ");
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(500);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    delay(1000);
  }
  Serial.println("Done");

  // Setup heater driver
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_ZERO_CROSS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), handlePin10Interrupt,
                  CHANGE);

  // Pin detect is to see if iron is in the stand. active low, pullup needed

  // Setup rotary encoder
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

  xTaskCreate(hmiTask,    // Task function
              "HMI Task", // Task name
              1000,       // Stack size
              NULL,       // Task parameters
              1,          // Task priority
              NULL        // Task handle
  );
  xTaskCreate(lcdTask,    // Task function
              "LCD Task", // Task name
              2000,       // Stack size
              NULL,       // Task parameters
              1,          // Task priority
              NULL        // Task handle
  );

  Serial.println("Setup done");
}

static void setTempSetpoint(float setpoint) {
  Serial.print("Setting setpoint to ");
  Serial.println(setpoint);

  heater_setpoint = setpoint;
}

static void setErrorFlag(uint8_t flag) {
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

static void clearErrorFlag(uint8_t flag) { error_flags &= ~flag; }

static float calcTargetTemp(bool in_stand) {
  if (in_stand) {
    return 150.0;
  }
  return heater_setpoint;
}

// Uses a 100k NTC thermistor with a 100k pulldown resistor
static float getAmbientTempCelcius() {
  int32_t reading = ads.readADC_SingleEnded(3);
  float voltage = ads.computeVolts(reading);

  // Calculate the resistance of the thermistor
  float resistance = 100000.0 * (3.289 / voltage - 1.0);

  // Calculate the temperature using the Steinhart-Hart equation
  float steinhart = resistance / 100000.0; // (R/Ro)
  steinhart = log(steinhart);              // ln(R/Ro)
  steinhart /= 3950.0;                     // 1/B * ln(R/Ro)

  // Calculate the temperature in Kelvin from the resistance
  float temp_kelvin =
      1.0 / (steinhart + 1.0 / 298.15); // 1/(1/B * ln(R/Ro) + 1/To)

  // Convert Kelvin to Celsius
  return temp_kelvin - 273.15;
}

// To not do complicated debouncing,
// I poll the encoder every 10ms and check if the state has changed
// in a separate task
void hmiTask(void *pvParameters) {
  while (1) {
    {
      static uint8_t lastState = 0;
      uint8_t currentState =
          (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);

      if (lastState == 0b11 && currentState == 0b10) {
        Serial.println("Clockwise rotation");
        // Clockwise rotation
        setTempSetpoint(heater_setpoint + 10);
      } else if (lastState == 0b11 && currentState == 0b01) {
        Serial.println("Counter-clockwise rotation");
        // Counter-clockwise rotation
        setTempSetpoint(heater_setpoint - 10);
      }
      lastState = currentState;
    }
    {
      static volatile bool lastState = HIGH;
      bool currentState = digitalRead(PIN_ENCODER_SW);
      if (lastState == HIGH && currentState == LOW) {
        Serial.println("Button pressed");
        // Reset the setpoint to 350 degrees
        setTempSetpoint(350);
      }
      lastState = currentState;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// LCD is using soft i2c and is really really slow,
// so i just update the screen every 10ms in a separate task
// LCD is 128x32 pixels
void lcdTask(void *pvParameters) {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Hello, World!");
  u8g2.sendBuffer();

  while (1) {

    u8g2.firstPage();
    do {
      if (error_flags) {
        u8g2.setFont(u8g2_font_ncenR08_tr);
        u8g2.setCursor(0, 10);
      } else {
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.setCursor(0, 22);
      }
      u8g2.print(actual_temperature, 2);
      if (last_in_stand) {
        u8g2.print(" // ");
      } else {
        u8g2.print(" -> ");
      }
      u8g2.print(heater_setpoint, 0);
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
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {

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
      clearErrorFlag(ERROR_BAD_FREQUENCY);
    } else {
      setErrorFlag(ERROR_BAD_FREQUENCY);
    }

    // Print the frequency
    // Serial.print("Intterupt count: ");
    // Serial.print(frequency_count);
    // Serial.print("Interrupt frequency: ");
    // Serial.print(frequency);
    // Serial.println(" Hz");
  }

  static float ambient_temp = 0;
  static uint32_t last_ambient_time = 0;
  if (last_ambient_time == 0 || (current_ms - last_ambient_time) > 60000) {
    last_ambient_time = current_ms;
    ambient_temp = getAmbientTempCelcius();
    Serial.print("Ambient temperature: ");
    Serial.print(ambient_temp);
    Serial.println(" °C");
  }

  // Calculate standby time
  // Only read the temperature if heater is off
  pinMode(PIN_DETECT, INPUT_PULLUP);
  vTaskDelay(1); // Sleep for a tick
  bool in_stand = digitalRead(PIN_DETECT) == LOW;
  pinMode(PIN_DETECT, INPUT);

  // Detect if the iron is placed in the stand has changed
  if (in_stand != last_in_stand) {
    last_standby_time = millis();
    last_in_stand = in_stand;
    clearErrorFlag(ERROR_STAND_BY);
  }

  // If the iron have been out of the stand or in the stand for more than 1
  // minute
  if (millis() - last_standby_time > STANDBY_TIMEOUT) {
    setErrorFlag(ERROR_STAND_BY);
  }

  float setpoint = calcTargetTemp(in_stand);

  // Require TEMP_DELAY_MS to settle the voltage
  if ((state == STATE_READING_TEMPERATURE &&
       current_ms >= (state_time + TEMP_DELAY_MS)) ||
      (state == STATE_IDLE && current_ms >= (state_time + 5000))) {
    // unsigned long start_ms = millis();
    int32_t adc_sum = 0;
    for (int i = 0; i < ADC_FILTERING; i++) {
      adc_sum += ads.readADC_SingleEnded(0);
    }
    float voltage_measured_mv =
        ads.computeVolts((float)adc_sum / (float)ADC_FILTERING) * 1000.0;

    // Calculate thermocouple voltage before amplification
    float thermocouple_voltage_mv = voltage_measured_mv / GAIN;

    // Print results
    Serial.print("Measured Voltage: ");
    Serial.print(voltage_measured_mv, 6);
    Serial.print(" mV before gain: ");
    Serial.print(thermocouple_voltage_mv, 6); // Convert back to mV for
    Serial.println(" mV");

    // Convert thermocouple voltage to temperature in °C
    float thermocouple_temperature =
        thermocouple_voltage_mv * 1e3 / THERMOCOUPLE_SENSITIVITY; // µV/°C to °C

    // Apply polynomial correction to the reported temperature
    actual_temperature =
        calibrate_temperature(ambient_temp + thermocouple_temperature);
    Serial.print("Temperature: ");
    Serial.print(thermocouple_temperature, 2);
    Serial.print(" °C corrected: ");
    Serial.print(ambient_temp + thermocouple_temperature, 2);
    Serial.print(" °C calibrated: ");
    Serial.print(actual_temperature, 2);
    Serial.println(" °C");

    Serial.print("Temperature: ");
    Serial.print(actual_temperature, 2);
    Serial.print(" °C --> ");
    Serial.print(setpoint, 2);
    Serial.println(" °C");

    if (actual_temperature < 5 || actual_temperature > 430) {
      Serial.println("Thermocouple fault(s) detected!");
      setErrorFlag(ERROR_NO_READING);
    } else {
      clearErrorFlag(ERROR_NO_READING);

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

  // Just so display looks right.
  if (error_flags != 0) {
    power = 0.0;
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);
}
