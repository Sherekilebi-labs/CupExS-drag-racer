#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

const char *CAR_NAME = "Car N4 ";

const uint8_t MOTOR_FORWARD_PIN = 19;
const uint8_t MOTOR_REVERSE_PIN = 21;

const uint32_t MOTOR_RUN_MS = 60000;
const uint32_t PWM_FREQUENCY = 20000;
const uint8_t PWM_RESOLUTION_BITS = 10;
const uint16_t PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
const uint8_t PWM_FORWARD_CHANNEL = 0;
const uint8_t PWM_REVERSE_CHANNEL = 1;

WebServer server(80);

bool motorRunning = false;
uint32_t motorStopAtMs = 0;

void writeForwardPwm(uint16_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_FORWARD_PIN, duty);
#else
  ledcWrite(PWM_FORWARD_CHANNEL, duty);
#endif
}

void writeReversePwm(uint16_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_REVERSE_PIN, duty);
#else
  ledcWrite(PWM_REVERSE_CHANNEL, duty);
#endif
}

void stopMotor() {
  motorRunning = false;
  motorStopAtMs = 0;
  writeForwardPwm(0);
  writeReversePwm(0);
}

void startMotor() {
  motorRunning = true;
  motorStopAtMs = millis() + MOTOR_RUN_MS;
  writeReversePwm(0);
  writeForwardPwm(PWM_MAX_DUTY);
}

String htmlEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length());

  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    switch (c) {
    case '&':
      escaped += F("&amp;");
      break;
    case '<':
      escaped += F("&lt;");
      break;
    case '>':
      escaped += F("&gt;");
      break;
    case '"':
      escaped += F("&quot;");
      break;
    case '\'':
      escaped += F("&#39;");
      break;
    default:
      escaped += c;
      break;
    }
  }

  return escaped;
}

String buildIndexHtml() {
  String carName = htmlEscape(CAR_NAME);
  String page = F(R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)rawliteral");
  page += carName;
  page += F(R"rawliteral(</title>
  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background:
        radial-gradient(circle at 18% 16%, rgba(255, 120, 34, 0.18), transparent 32%),
        radial-gradient(circle at 82% 8%, rgba(154, 88, 255, 0.24), transparent 32%),
        linear-gradient(145deg, #0b0711 0%, #181020 48%, #261421 100%);
      color: #f7f1e4;
      font-family: Arial, Helvetica, sans-serif;
      padding: 24px;
    }

    main {
      width: min(420px, 100%);
      display: grid;
      gap: 28px;
      justify-items: center;
      text-align: center;
    }

    h1 {
      margin: 0;
      font-size: clamp(40px, 14vw, 72px);
      line-height: 0.95;
      letter-spacing: 0;
      color: #f7f1e4;
      text-shadow: 0 0 24px rgba(255, 138, 42, 0.34), 0 0 36px rgba(164, 92, 255, 0.22);
    }

    button {
      width: min(280px, 100%);
      min-height: 96px;
      border: 2px solid rgba(255, 255, 255, 0.38);
      border-radius: 999px;
      background: #d5162a;
      color: #ffffff;
      font-size: 30px;
      font-weight: 800;
      text-transform: uppercase;
      cursor: pointer;
      touch-action: manipulation;
      box-shadow: 0 0 32px rgba(213, 22, 42, 0.56), 0 0 44px rgba(164, 92, 255, 0.18);
      -webkit-tap-highlight-color: transparent;
    }

    button:active {
      transform: translateY(1px) scale(0.99);
    }
  </style>
</head>
<body>
  <main>
    <h1>)rawliteral");
  page += carName;
  page += F(R"rawliteral(</h1>
    <button id="startButton" type="button">Start</button>
    <button id="stopButton" type="button">Stop</button>
  </main>

  <script>
    const startButton = document.getElementById('startButton');
    const stopButton = document.getElementById('stopButton');

    startButton.addEventListener('click', async () => {
      startButton.disabled = true;
      try {
        const response = await fetch('/start', {method: 'POST'});
        if (!response.ok) {
          throw new Error(await response.text());
        }
      } catch (error) {
        alert(error.message);
      } finally {
        startButton.disabled = false;
      }
    });

    stopButton.addEventListener('click', async () => {
      stopButton.disabled = true;
      try {
        const response = await fetch('/stop', {method: 'POST'});
        if (!response.ok) {
          throw new Error(await response.text());
        }
      } catch (error) {
        alert(error.message);
      } finally {
        stopButton.disabled = false;
      }
    });
  </script>
</body>
</html>
)rawliteral");

  return page;
}

void handleIndex() {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), buildIndexHtml());
}

void handleStart() {
  startMotor();
  server.send(200, F("text/plain"), F("started"));
}

void handleStop() {
  stopMotor();
  server.send(200, F("text/plain"), F("stopped"));
}

void setupPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_FORWARD_PIN, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcAttach(MOTOR_REVERSE_PIN, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
#else
  ledcSetup(PWM_FORWARD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(PWM_REVERSE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_FORWARD_PIN, PWM_FORWARD_CHANNEL);
  ledcAttachPin(MOTOR_REVERSE_PIN, PWM_REVERSE_CHANNEL);
#endif
  stopMotor();
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_REVERSE_PIN, OUTPUT);
  setupPwm();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(CAR_NAME);

  server.on(F("/"), HTTP_GET, handleIndex);
  server.on(F("/start"), HTTP_POST, handleStart);
  server.on(F("/stop"), HTTP_POST, handleStop);
  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("Not found"));
  });
  server.begin();

  Serial.println();
  Serial.print(F("Wi-Fi AP: "));
  Serial.println(CAR_NAME);
  Serial.print(F("Open http://"));
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  if (motorRunning && static_cast<int32_t>(millis() - motorStopAtMs) >= 0) {
    stopMotor();
  }
}
