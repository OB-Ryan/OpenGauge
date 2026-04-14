#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <LiquidCrystal_I2C.h>

/*
OBDII Reference: https://en.wikipedia.org/wiki/OBD-II_PIDs

    MODES
01  Show current data
02  Show freeze frame data
03  Show stored Diag trouble codes
04  Clear diag trouble codes and stored values
05  Test results, o2 sensor minitoring non-CAN
06  Test results, other component/system monitoring CAN
07  Show pending diag trouble codes
08  Control operation of on-board component/system
09  Request vehicle info
0A  Permanent Diag trouble codes

    STANDARD PIDs OF INTEREST
04  Calculated engine load (%). Formula: (A * 100) / 255
05  Engine coolant temperature (C). Formula: (A - 40)
10  Fuel pressure (kPa). Formula: A * 3
12  Engine speed (RPM). Formula: ((256A + B) / 4)
13  Vehicle speed (km/h). Formula: (A)
15  Intake air temperature (C). Formula: (A - 40)
92  Engine oil temperature (C). Formula: (A - 40)
*/

// PIDs
#define LOAD        4
#define COOLANT_TMP 5
#define FUEL_PRESS  10
#define RPM         12
#define SPEED       13
#define AIR_TMP     15
#define OIL_TMP     92

// Pin definitions and constants
#define CAN_TX		5               // CAN TX GPIO
#define CAN_RX		4               // CAN RX GPIO
#define I2C_ADDR  0x27            // I2C LCD addr
#define LCD_LINES 2               // 16x2 LCD screen size
#define LCD_LEN   16
#define BUTTON    23              // Push-button on pin 23
#define OBDII_RESPONSE_ID_MIN      0x7E8
#define OBDII_RESPONSE_ID_MAX      0x7EF
#define OBDII_REQUEST_IDENTIFIER   0x7DF

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);
CanFrame rxFrame;
char buf[LCD_LEN + 1];  // Buffer to hold strings to print on LCD
int mode = 0;           // Current sensor mode
int modes[] = {LOAD, COOLANT_TMP, FUEL_PRESS, RPM, SPEED, AIR_TMP, OIL_TMP};
int mode_index = 0;
int num_modes = sizeof(modes) / sizeof(modes[0]);

// Sensor reading vars
int latest_load = -1;
int latest_coolant_temp = -1;
float latest_fuel_pressure = -1;
int latest_rpm = -1;
float latest_speed = -1;
int latest_air_temp = -1;
int latest_oil_temp = -1;

// State management
enum State {IDLE, WAITING};
State state = IDLE;
unsigned long request_sent_time = 0;
const unsigned long timeout = 1000;              // 1 second CAN request timeout
unsigned long last_obd_request_time = 0;
const unsigned long obd_request_interval = 1200; // 1.2 second between OBD requests (allow margin of error)

unsigned long last_invalid_frame_log_time = 0;
bool can_online = false;
int consecutive_tx_failures = 0;
const int failure_threshold = 10;
unsigned long last_recovery_attempt_time = 0;
const unsigned long recovery_interval = 3000;


bool is_obd_response_identifier(uint32_t identifier) {
  return identifier >= OBDII_RESPONSE_ID_MIN && identifier <= OBDII_RESPONSE_ID_MAX;
}

bool is_valid_obd_response(const CanFrame &frame, uint8_t expected_pid, uint8_t min_dlc) {
  return !frame.extd &&
    is_obd_response_identifier(frame.identifier) &&
    frame.data_length_code >= min_dlc &&
    frame.data[1] == 0x41 &&
    frame.data[2] == expected_pid;
}

void log_frame_brief(const CanFrame &frame) {
  Serial.printf("RX id=0x%03lX dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
    frame.identifier,
    frame.data_length_code,
    frame.data[0],
    frame.data[1],
    frame.data[2],
    frame.data[3],
    frame.data[4],
    frame.data[5],
    frame.data[6],
    frame.data[7]
  );
}

bool begin_can() {
  bool success = ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10);
  if (success) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }
  return success;
}

bool send_obd_frame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
  obdFrame.identifier = OBDII_REQUEST_IDENTIFIER;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
  
  // Padding of 0xAA to avoid bit surfing
  obdFrame.data[3] = 0xAA;
  obdFrame.data[4] = 0xAA;
  obdFrame.data[5] = 0xAA;
  obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;

  // Accepts both pointers and references 
  bool tx_ok = ESP32Can.writeFrame(obdFrame, 25);
  if (!tx_ok) {
    Serial.printf("TX failed for PID 0x%02X\n", obdId);
    return false;
  } else {
    Serial.printf("TX PID 0x%02X on 0x%03X\n", obdId, OBDII_REQUEST_IDENTIFIER);
    return true;
  }
}

void init_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("OBD2 CAN Ready");
}

void lcd_clear_line(int line) {
  if (line != 0 && line != 1) {
    Serial.println("ERROR    |    lcd_clear_line: line not valid for 16x2 LCD");
    return;
  }

  lcd.setCursor(0, line);
  for (int i = 0; i < LCD_LEN; i++) {
    lcd.print(" ");
  }
}

void lcd_write(int line, const char* message) {
  if (line != 0 && line != 1) {
    Serial.println("ERROR    |    lcd_write: line not valid for 16x2 LCD");
    return;
  }

  if (strlen(message) > LCD_LEN) {
    Serial.println("ERROR    |    lcd_write: message too long for 16x2 LCD");
    return;
  }

  lcd_clear_line(line);
  lcd.setCursor(0, line);
  lcd.print(message);
}

bool process_load_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, LOAD, 4)) {
    latest_load = (rxFrame.data[3] * 100) / 255;
    Serial.printf("Load: %d%%\n", latest_load);
    snprintf(buf, sizeof(buf), " %d%% ", latest_load);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_coolant_temp_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, COOLANT_TMP, 4)) {
    latest_coolant_temp = rxFrame.data[3] - 40;
    Serial.printf("Coolant temp: %d%c\n", latest_coolant_temp, (char)223);
    snprintf(buf, sizeof(buf), " %d%cC ", latest_coolant_temp, (char)223);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_fuel_pressure_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, FUEL_PRESS, 4)) {
    latest_fuel_pressure = (rxFrame.data[3] * 3) * 0.1450377; // Convert kPa to PSI
    Serial.printf("Fuel pressure: %.2f PSI\n", latest_fuel_pressure);
    snprintf(buf, sizeof(buf), "    %.2f PSI", latest_fuel_pressure);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_rpm_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, RPM, 5)) {
    latest_rpm = (((256 * rxFrame.data[3]) + rxFrame.data[4]) / 4);
    Serial.printf("RPM: %3d\n", latest_rpm);
    snprintf(buf, sizeof(buf), "    %d ", latest_rpm);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_speed_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, SPEED, 4)) {
    latest_speed = (rxFrame.data[3] / 1.609344);  // Convert kph to mph
    Serial.printf("Speed: %.2f mph\n", latest_speed);
    snprintf(buf, sizeof(buf), "    %.2f mph", latest_speed);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_air_temp_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, AIR_TMP, 4)) {
    latest_air_temp = rxFrame.data[3] - 40;
    Serial.printf("Intake air temp: %d%c\n", latest_air_temp, (char)223);
    snprintf(buf, sizeof(buf), "    %d%cC ", latest_air_temp, (char)223);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

bool process_oil_temp_response(const CanFrame &rxFrame) {
  if (is_valid_obd_response(rxFrame, OIL_TMP, 4)) {
    latest_oil_temp = rxFrame.data[3] - 40;
    Serial.printf("Oil temp: %d%c\n", latest_oil_temp, (char)223);
    snprintf(buf, sizeof(buf), "    %d%cC ", latest_oil_temp, (char)223);
    lcd_write(1, buf);
    return true;
  }

  return false;
}

const char* mode_title(int current_mode) {
  switch (current_mode)
  {
  case LOAD:
    return "Engine Load";
  case COOLANT_TMP:
    return "Coolant Temp";
  case RPM:
    return "Engine RPM";
  case SPEED:
    return "Car Speed";
  case FUEL_PRESS:
    return "Fuel Pressure";
  case AIR_TMP:
    return "Intake Air Temp";
  case OIL_TMP:
    return "Oil Temp";
  default:
    return "Unknown Mode";
  }
}

void write_mode() {
  lcd_write(0, mode_title(mode));
}

bool process_response(const CanFrame &rxFrame) {
  switch (mode)
  {
  case LOAD:
    return process_load_response(rxFrame);
  case COOLANT_TMP:
    return process_coolant_temp_response(rxFrame);
  case RPM:
    return process_rpm_response(rxFrame);
  case SPEED:
    return process_speed_response(rxFrame);
  case FUEL_PRESS:
    return process_fuel_pressure_response(rxFrame);
  case AIR_TMP:
    return process_air_temp_response(rxFrame);
  case OIL_TMP:
    return process_oil_temp_response(rxFrame);
  default:
    return false;
  }
}

void button_update() {
  if (digitalRead(BUTTON) == LOW) {
    mode_index++;
    if (mode_index >= num_modes) {
      mode_index = 0;
    }
    mode = modes[mode_index];
    write_mode();
    delay(200); // debounce delay
  }
}

void setup() {
  Serial.begin(115200);

  // Set pins
	ESP32Can.setPins(CAN_TX, CAN_RX);
  pinMode(BUTTON, INPUT_PULLUP);
	
  // Set TX and RX queue sizes. Larger RX queue incase of busy bus
  ESP32Can.setRxQueueSize(64);
	ESP32Can.setTxQueueSize(5);

  can_online = begin_can();
  init_lcd();
  if (!can_online) {
    lcd_write(1, "No CAN bus");
  }

  // Set the mode. Default to first mode in list
  mode = modes[0];
  write_mode();
}

void loop() {
  // Polling button - cycle mode on press
  button_update();

  unsigned long now = millis();

  // Attempt recovery of CAN bus if offline
  if (!can_online) {
    if (now - last_recovery_attempt_time >= recovery_interval) {
      last_recovery_attempt_time = now;
      Serial.println("Attempting CAN recovery...");
      if (begin_can()) {
        can_online = true;
        consecutive_tx_failures = 0;
        state = IDLE;
        lcd_write(1, "CAN recovered");
      } else {
        lcd_write(1, "No CAN bus");
      }
    }
    return;
  }

  // Send request if idle. No more than 1 per second
  if (state == IDLE) {
    if (now - last_obd_request_time >= obd_request_interval) {
      last_obd_request_time = now;
      if (send_obd_frame(mode)) {
        consecutive_tx_failures = 0;
        request_sent_time = now;
        state = WAITING;
      } else {
        consecutive_tx_failures++;
        // Leave state IDLE so we can retry on next interval without waiting for response timeout.
        snprintf(buf, sizeof(buf), "TX failed %d", mode);
        lcd_write(1, buf);
        if (consecutive_tx_failures >= failure_threshold) {
          Serial.println("CAN marked offline after repeated TX failures");
          can_online = false;
          state = IDLE;
          lcd_write(1, "No CAN bus");
        }
      }
    }
  } else if (state == WAITING) {
    // Empty all queued frames and only leave WAITING when we find a matching OBD response.
    bool matched_response = false;
    while (ESP32Can.readFrame(rxFrame, 0)) {
      if (process_response(rxFrame)) {
        matched_response = true;
        break;
      }

      if (now - last_invalid_frame_log_time >= 250) {
        // Throttle logs so monitor stays readable
        log_frame_brief(rxFrame);
        last_invalid_frame_log_time = now;
      }
    }

    if (matched_response) {
      state = IDLE;
    } else if (now - request_sent_time > timeout) {
      // Timeout for CAN response. Set to idle to try again
      snprintf(buf, sizeof(buf), "No Resp PID %2d", mode);
      lcd_write(1, buf);
      Serial.println("Timeout waiting for matching OBD response");
      state = IDLE;
    }
  }
}
