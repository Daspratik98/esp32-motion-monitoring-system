#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <MPU6050_tockn.h>
#include <Adafruit_NeoPixel.h>

// ======================== CONFIGURATION ========================

const char* ssid = "Pixel_2117";
const char* password = "Gj05lh@3467";

const char* GOOGLE_SCRIPT_URL =
    "https://script.google.com/macros/s/AKfycbz9Z-aAJifnZ4eQxuKt7BT4tE6Qk9MCAONts4fXTmr1R6IK8vva0Mt_qDTQazYHTCVJTw/exec";

const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;
const uint8_t NEOPIXEL_PIN = 26;
const uint8_t MOTOR_PIN = 25;
const uint8_t BUTTON_PIN = 27;
const uint8_t PIXEL_COUNT = 1;

const float STABLE_THRESHOLD = 5.0f;
const float MOTION_THRESHOLD = 25.0f;
const float SHAKE_THRESHOLD = 120.0f;

const float FILTER_OLD_WEIGHT = 0.8f;
const float FILTER_NEW_WEIGHT = 0.2f;

const unsigned long RED_FLASH_INTERVAL = 200;
const unsigned long SERIAL_INTERVAL = 100;
const unsigned long BUTTON_DEBOUNCE_TIME = 50;
const unsigned long STARTUP_STEP_TIME = 250;
const unsigned long STARTUP_MOTOR_TIME = 250;

const unsigned long UPLOAD_INTERVAL = 5000;
const unsigned long WIFI_RETRY_INTERVAL = 5000;
const uint16_t HTTP_TIMEOUT = 5000;
const uint8_t UPLOAD_QUEUE_LENGTH = 10;

const uint8_t LED_BRIGHTNESS = 64;
const uint8_t COLOR_LEVEL = 255;

// ===============================================================

enum ActivityState {
  STABLE,
  MOTION,
  SHAKE
};

enum UploadEvent {
  HEARTBEAT,
  STABLE_EVENT,
  MOTION_STARTED,
  SHAKE_STARTED
};

struct UploadRecord {
  float activity;
  ActivityState state;
  UploadEvent event;
};

MPU6050 mpu(Wire);
Adafruit_NeoPixel pixel(
    PIXEL_COUNT,
    NEOPIXEL_PIN,
    NEO_GRB + NEO_KHZ800);

QueueHandle_t uploadQueue = nullptr;
portMUX_TYPE activityMux = portMUX_INITIALIZER_UNLOCKED;

bool monitoringEnabled = false;
bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;

bool lastWiFiConnected = false;

unsigned long lastButtonChangeTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastWiFiRetryTime = 0;

float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float activity = 0.0f;

ActivityState currentState = STABLE;

volatile int lastHttpResponse = 0;

uint32_t lastLEDColor = 0;
bool ledInitialized = false;

void initializeHardware();
void startupAnimation();
void readButton();
void readIMU();
void calculateActivity();
void updateOutputs();
void printStatus();
void setLED(uint8_t red, uint8_t green, uint8_t blue);
void startWiFi();
void maintainWiFi();
void queueStateUpload(ActivityState state, float value);
void sendToGoogleSheets(const UploadRecord& record);
void uploadTask(void* parameter);
const char* getStateName(ActivityState state);
const char* getEventName(UploadEvent event);

// Initializes the existing hardware.
void initializeHardware() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  pixel.begin();
  pixel.setBrightness(LED_BRIGHTNESS);
  pixel.clear();
  pixel.show();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpu.begin();

  Serial.println("[IMU] Calibrating gyro");
  mpu.calcGyroOffsets(true);
  Serial.println("[IMU] Calibration complete");
}

// Runs the existing startup animation.
void startupAnimation() {
  setLED(COLOR_LEVEL, 0, 0);
  delay(STARTUP_STEP_TIME);

  setLED(0, COLOR_LEVEL, 0);
  delay(STARTUP_STEP_TIME);

  setLED(0, 0, COLOR_LEVEL);
  delay(STARTUP_STEP_TIME);

  setLED(0, 0, 0);
  delay(STARTUP_STEP_TIME);

  digitalWrite(MOTOR_PIN, HIGH);
  delay(STARTUP_MOTOR_TIME);
  digitalWrite(MOTOR_PIN, LOW);
}

// Debounces the button and toggles monitoring.
void readButton() {
  const bool rawButtonState = digitalRead(BUTTON_PIN);
  const unsigned long now = millis();

  if (rawButtonState != lastRawButtonState) {
    lastRawButtonState = rawButtonState;
    lastButtonChangeTime = now;
  }

  if ((now - lastButtonChangeTime) >= BUTTON_DEBOUNCE_TIME &&
      rawButtonState != stableButtonState) {
    stableButtonState = rawButtonState;

    if (stableButtonState == LOW) {
      monitoringEnabled = !monitoringEnabled;
      activity = 0.0f;
      currentState = STABLE;
      digitalWrite(MOTOR_PIN, LOW);

      Serial.print("[BUTTON] Monitoring ");
      Serial.println(monitoringEnabled ? "ON" : "OFF");
    }
  }
}

// Refreshes the MPU6050 and reads its gyroscope.
void readIMU() {
  mpu.update();

  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
}

// Runs the existing motion classification.
void calculateActivity() {
  float newActivity =
      fabsf(gyroX) +
      fabsf(gyroY) +
      fabsf(gyroZ);

  if (newActivity < STABLE_THRESHOLD) {
    newActivity = 0.0f;
  }

  activity =
      (activity * FILTER_OLD_WEIGHT) +
      (newActivity * FILTER_NEW_WEIGHT);

  if (!monitoringEnabled) {
    currentState = STABLE;
    return;
  }

  const ActivityState previousState = currentState;

  if (activity >= SHAKE_THRESHOLD) {
    currentState = SHAKE;
  } else if (activity >= MOTION_THRESHOLD) {
    currentState = MOTION;
  } else {
    currentState = STABLE;
  }

  if (currentState != previousState) {
    queueStateUpload(currentState, activity);
  }
}

// Runs the existing LED and motor behavior.
void updateOutputs() {
  if (!monitoringEnabled) {
    digitalWrite(MOTOR_PIN, LOW);
    setLED(0, COLOR_LEVEL, 0);
    return;
  }

  switch (currentState) {
    case STABLE:
      digitalWrite(MOTOR_PIN, LOW);
      setLED(0, 0, COLOR_LEVEL);
      break;

    case MOTION:
      digitalWrite(MOTOR_PIN, LOW);
      setLED(COLOR_LEVEL, COLOR_LEVEL, 0);
      break;

    case SHAKE: {
      digitalWrite(MOTOR_PIN, HIGH);

      const bool redOn =
          ((millis() / RED_FLASH_INTERVAL) % 2UL) == 0;

      if (redOn) {
        setLED(COLOR_LEVEL, 0, 0);
      } else {
        setLED(0, 0, 0);
      }
      break;
    }
  }
}

// Prints the existing motion status.
void printStatus() {
  const unsigned long now = millis();

  if ((now - lastSerialTime) < SERIAL_INTERVAL) {
    return;
  }

  lastSerialTime = now;

  Serial.print("GyroX: ");
  Serial.print(gyroX, 2);
  Serial.print(" | GyroY: ");
  Serial.print(gyroY, 2);
  Serial.print(" | GyroZ: ");
  Serial.print(gyroZ, 2);
  Serial.print(" | Activity: ");
  Serial.print(activity, 2);
  Serial.print(" | State: ");

  if (monitoringEnabled) {
    Serial.println(getStateName(currentState));
  } else {
    Serial.println("MONITORING OFF");
  }
}

// Writes a NeoPixel color only when it changes.
void setLED(uint8_t red, uint8_t green, uint8_t blue) {
  const uint32_t color = pixel.Color(red, green, blue);

  if (!ledInitialized || color != lastLEDColor) {
    ledInitialized = true;
    lastLEDColor = color;
    pixel.setPixelColor(0, color);
    pixel.show();
  }
}

// Starts the initial non-blocking WiFi connection.
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  lastWiFiRetryTime = millis();
  lastWiFiConnected = false;
}

// Monitors WiFi and retries without blocking the main loop.
void maintainWiFi() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const unsigned long now = millis();

  if (connected && !lastWiFiConnected) {
    Serial.println("WiFi Connected");
  } else if (!connected && lastWiFiConnected) {
    Serial.println("WiFi Lost");
  }

  lastWiFiConnected = connected;

  if (!connected &&
      (now - lastWiFiRetryTime) >= WIFI_RETRY_INTERVAL) {
    lastWiFiRetryTime = now;
    WiFi.begin(ssid, password);
  }
}

// Queues an immediate state-change upload.
void queueStateUpload(ActivityState state, float value) {
  if (!monitoringEnabled || uploadQueue == nullptr) {
    return;
  }

  UploadRecord record;
  record.activity = value;
  record.state = state;

  switch (state) {
    case STABLE:
      record.event = STABLE_EVENT;
      break;

    case MOTION:
      record.event = MOTION_STARTED;
      break;

    case SHAKE:
      record.event = SHAKE_STARTED;
      break;
  }

  xQueueSend(uploadQueue, &record, 0);
}

// Sends one record to Google Sheets using HTTPS GET.
void sendToGoogleSheets(const UploadRecord& record) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  String url;
  url.reserve(320);
  url = GOOGLE_SCRIPT_URL;
  url += "?activity=";
  url += String(record.activity, 2);
  url += "&state=";
  url += getStateName(record.state);

  if (record.event != HEARTBEAT) {
    url += "&event=";
    url += getEventName(record.event);
  }

  url.replace(" ", "%20");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) {
    lastHttpResponse = -1;
    Serial.println("Upload Failed");
    return;
  }

  const int responseCode = http.GET();
  lastHttpResponse = responseCode;

  Serial.print("HTTP Response Code: ");
  Serial.println(responseCode);

  if (responseCode >= 200 && responseCode < 300) {
    Serial.println("Upload Success");
  } else {
    Serial.println("Upload Failed");
  }

  http.end();
}

// Handles all potentially blocking network requests off the main loop.
void uploadTask(void* parameter) {
  (void)parameter;

  unsigned long lastHeartbeatTime = millis();
  UploadRecord queuedRecord;

  for (;;) {
    if (xQueueReceive(
            uploadQueue,
            &queuedRecord,
            pdMS_TO_TICKS(50)) == pdTRUE) {
      if (monitoringEnabled &&
          WiFi.status() == WL_CONNECTED) {
        sendToGoogleSheets(queuedRecord);
      }
    }

    const unsigned long now = millis();

    if (monitoringEnabled &&
        (now - lastHeartbeatTime) >= UPLOAD_INTERVAL) {
      lastHeartbeatTime = now;

      UploadRecord heartbeat;

      portENTER_CRITICAL(&activityMux);
      heartbeat.activity = activity;
      heartbeat.state = currentState;
      portEXIT_CRITICAL(&activityMux);

      heartbeat.event = HEARTBEAT;

      if (WiFi.status() == WL_CONNECTED) {
        sendToGoogleSheets(heartbeat);
      }
    }

    if (!monitoringEnabled) {
      lastHeartbeatTime = now;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Returns the Google Sheets state name.
const char* getStateName(ActivityState state) {
  switch (state) {
    case STABLE:
      return "Stable";

    case MOTION:
      return "Motion";

    case SHAKE:
      return "Shake";

    default:
      return "Stable";
  }
}

// Returns the Google Sheets event name.
const char* getEventName(UploadEvent event) {
  switch (event) {
    case STABLE_EVENT:
      return "Stable";

    case MOTION_STARTED:
      return "Motion Started";

    case SHAKE_STARTED:
      return "Shake Started";

    case HEARTBEAT:
    default:
      return "";
  }
}

// Initializes the complete device.
void setup() {
  Serial.begin(115200);

  initializeHardware();
  startupAnimation();

  uploadQueue =
      xQueueCreate(UPLOAD_QUEUE_LENGTH, sizeof(UploadRecord));

  startWiFi();

  xTaskCreatePinnedToCore(
      uploadTask,
      "GoogleSheets",
      8192,
      nullptr,
      1,
      nullptr,
      0);

  monitoringEnabled = false;
  currentState = STABLE;
  updateOutputs();

  Serial.println("[BOOT] COMPLETE");
}

// Runs all responsive device services.
void loop() {
  maintainWiFi();
  readButton();
  readIMU();
  calculateActivity();
  updateOutputs();
  printStatus();
}