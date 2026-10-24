#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

const char *CAR_NAME = "Car N1";

const uint8_t MOTOR_FORWARD_PIN = 19;
const uint8_t MOTOR_REVERSE_PIN = 21;

const uint32_t PWM_FREQUENCY = 20000;
const uint8_t PWM_RESOLUTION_BITS = 10;
const uint16_t PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
const uint8_t PWM_FORWARD_CHANNEL = 0;
const uint8_t PWM_REVERSE_CHANNEL = 1;

enum class MotorDirection {
  Stop,
  Forward,
  Backward
};

WebServer server(80);

MotorDirection currentDirection = MotorDirection::Stop;
uint8_t currentDutyPercent = 0;
bool dragRaceActive = false;
uint32_t dragRaceEndMs = 0;

String directionToString(MotorDirection direction) {
  switch (direction) {
  case MotorDirection::Forward:
    return "forward";
  case MotorDirection::Backward:
    return "backward";
  case MotorDirection::Stop:
  default:
    return "stop";
  }
}

uint16_t dutyPercentToPwm(uint8_t dutyPercent) {
  dutyPercent = constrain(dutyPercent, 0, 100);
  return static_cast<uint16_t>((static_cast<uint32_t>(dutyPercent) * PWM_MAX_DUTY) / 100);
}

void writeForwardPwmPercent(uint8_t dutyPercent) {
  uint16_t duty = dutyPercentToPwm(dutyPercent);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_FORWARD_PIN, duty);
#else
  ledcWrite(PWM_FORWARD_CHANNEL, duty);
#endif
}

void writeReversePwmPercent(uint8_t dutyPercent) {
  uint16_t duty = dutyPercentToPwm(dutyPercent);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_REVERSE_PIN, duty);
#else
  ledcWrite(PWM_REVERSE_CHANNEL, duty);
#endif
}

void applyMotor(MotorDirection direction, uint8_t dutyPercent) {
  currentDirection = direction;
  currentDutyPercent = constrain(dutyPercent, 0, 100);

  if (direction == MotorDirection::Stop || currentDutyPercent == 0) {
    writeForwardPwmPercent(0);
    writeReversePwmPercent(0);
    return;
  }

  if (direction == MotorDirection::Forward) {
    writeReversePwmPercent(0);
    writeForwardPwmPercent(currentDutyPercent);
  } else {
    writeForwardPwmPercent(0);
    writeReversePwmPercent(currentDutyPercent);
  }
}

void cancelDragRace() {
  dragRaceActive = false;
  dragRaceEndMs = 0;
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

String buildStatusJson() {
  String json = F("{\"name\":\"");
  json += htmlEscape(CAR_NAME);
  json += F("\",\"direction\":\"");
  json += directionToString(currentDirection);
  json += F("\",\"duty\":");
  json += currentDutyPercent;
  json += F(",\"dragActive\":");
  json += dragRaceActive ? F("true") : F("false");
  json += F(",\"dragRemainingMs\":");

  if (dragRaceActive) {
    uint32_t now = millis();
    json += static_cast<uint32_t>(dragRaceEndMs - now);
  } else {
    json += 0;
  }

  json += '}';
  return json;
}

void sendStatus() {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), buildStatusJson());
}

void handleDirection() {
  if (!server.hasArg(F("value"))) {
    server.send(400, F("text/plain"), F("Missing value"));
    return;
  }

  String value = server.arg(F("value"));
  cancelDragRace();

  if (value == F("stop")) {
    applyMotor(MotorDirection::Stop, 0);
  } else if (value == F("forward")) {
    applyMotor(MotorDirection::Forward, currentDutyPercent);
  } else if (value == F("backward")) {
    applyMotor(MotorDirection::Backward, currentDutyPercent);
  } else {
    server.send(400, F("text/plain"), F("Invalid direction"));
    return;
  }

  sendStatus();
}

void handleDuty() {
  if (!server.hasArg(F("value"))) {
    server.send(400, F("text/plain"), F("Missing value"));
    return;
  }

  int requestedDuty = server.arg(F("value")).toInt();
  requestedDuty = constrain(requestedDuty, 0, 100);
  currentDutyPercent = static_cast<uint8_t>(requestedDuty);

  if (!dragRaceActive) {
    applyMotor(currentDirection, currentDutyPercent);
  }

  sendStatus();
}

void handleDragRace() {
  if (!server.hasArg(F("seconds"))) {
    server.send(400, F("text/plain"), F("Missing seconds"));
    return;
  }

  float requestedSeconds = server.arg(F("seconds")).toFloat();
  requestedSeconds = constrain(requestedSeconds, 0.1f, 60.0f);

  dragRaceActive = true;
  dragRaceEndMs = millis() + static_cast<uint32_t>(requestedSeconds * 1000.0f);
  applyMotor(MotorDirection::Forward, 100);
  sendStatus();
}

void handleStop() {
  cancelDragRace();
  applyMotor(MotorDirection::Stop, 0);
  sendStatus();
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
  page += F(R"rawliteral( Motor Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bone: #f7f1e4;
      --bone-dim: #d9d0be;
      --ink: #15101d;
      --panel: rgba(247, 241, 228, 0.09);
      --panel-strong: rgba(247, 241, 228, 0.15);
      --orange: #ff8a2a;
      --purple: #a45cff;
      --danger: #ff4f6d;
      --shadow-orange: rgba(255, 138, 42, 0.42);
      --shadow-purple: rgba(164, 92, 255, 0.45);
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background:
        radial-gradient(circle at 18% 18%, rgba(255, 138, 42, 0.18), transparent 34%),
        radial-gradient(circle at 85% 5%, rgba(164, 92, 255, 0.24), transparent 32%),
        linear-gradient(145deg, #0b0711 0%, #16101f 46%, #241421 100%);
      color: var(--bone);
      font-family: Arial, Helvetica, sans-serif;
      padding: 22px;
    }

    main {
      width: min(540px, 100%);
      border: 1px solid rgba(247, 241, 228, 0.16);
      border-radius: 24px;
      background: linear-gradient(155deg, rgba(247, 241, 228, 0.12), rgba(247, 241, 228, 0.05));
      box-shadow: 0 0 42px rgba(164, 92, 255, 0.16), 0 0 54px rgba(255, 138, 42, 0.10);
      padding: 22px;
      backdrop-filter: blur(12px);
    }

    header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 18px;
    }

    h1 {
      margin: 0;
      font-size: clamp(28px, 8vw, 44px);
      line-height: 0.95;
      letter-spacing: 0;
      color: var(--bone);
      text-shadow: 0 0 18px rgba(255, 138, 42, 0.25);
    }

    .chip {
      border: 1px solid rgba(255, 138, 42, 0.45);
      border-radius: 999px;
      color: var(--bone);
      padding: 8px 12px;
      font-size: 12px;
      text-transform: uppercase;
      box-shadow: 0 0 18px rgba(255, 138, 42, 0.12);
      white-space: nowrap;
    }

    .panel {
      border: 1px solid rgba(247, 241, 228, 0.14);
      border-radius: 18px;
      background: var(--panel);
      padding: 16px;
      margin-top: 14px;
    }

    .drag-grid {
      display: grid;
      grid-template-columns: 1fr minmax(98px, 0.42fr);
      gap: 12px;
      align-items: end;
    }

    label {
      display: block;
      color: var(--bone-dim);
      font-size: 13px;
      margin-bottom: 8px;
    }

    input[type="number"] {
      width: 100%;
      min-height: 48px;
      border: 1px solid rgba(247, 241, 228, 0.20);
      border-radius: 14px;
      background: rgba(10, 7, 16, 0.55);
      color: var(--bone);
      font-size: 18px;
      padding: 10px 12px;
      outline: none;
      box-shadow: inset 0 0 18px rgba(164, 92, 255, 0.08);
    }

    input[type="number"]:focus {
      border-color: var(--purple);
      box-shadow: 0 0 18px rgba(164, 92, 255, 0.28), inset 0 0 18px rgba(164, 92, 255, 0.10);
    }

    button {
      min-height: 52px;
      border: 1px solid rgba(247, 241, 228, 0.16);
      border-radius: 16px;
      background: rgba(247, 241, 228, 0.10);
      color: var(--bone);
      font-size: 16px;
      font-weight: 700;
      cursor: pointer;
      touch-action: manipulation;
      transition: transform 120ms ease, border-color 120ms ease, box-shadow 120ms ease, background 120ms ease;
      -webkit-tap-highlight-color: transparent;
    }

    button:active {
      transform: translateY(1px) scale(0.99);
    }

    button.primary {
      border-color: rgba(255, 138, 42, 0.72);
      background: linear-gradient(135deg, rgba(255, 138, 42, 0.92), rgba(164, 92, 255, 0.78));
      box-shadow: 0 0 26px var(--shadow-orange), 0 0 34px rgba(164, 92, 255, 0.18);
      color: #fff9ed;
    }

    button.mode.active {
      border-color: var(--orange);
      box-shadow: 0 0 20px var(--shadow-orange);
      background: rgba(255, 138, 42, 0.22);
    }

    button.stop.active {
      border-color: var(--danger);
      box-shadow: 0 0 18px rgba(255, 79, 109, 0.34);
      background: rgba(255, 79, 109, 0.18);
    }

    .modes {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }

    .slider-row {
      display: grid;
      grid-template-columns: 1fr 76px;
      gap: 14px;
      align-items: center;
    }

    input[type="range"] {
      width: 100%;
      accent-color: var(--orange);
    }

    .percent {
      min-height: 48px;
      border-radius: 14px;
      display: grid;
      place-items: center;
      border: 1px solid rgba(164, 92, 255, 0.40);
      background: rgba(164, 92, 255, 0.12);
      color: var(--bone);
      font-size: 20px;
      font-weight: 800;
      box-shadow: 0 0 18px rgba(164, 92, 255, 0.18);
    }

    .status {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 14px;
      color: var(--bone-dim);
      font-size: 13px;
    }

    .status div {
      border-radius: 14px;
      background: rgba(10, 7, 16, 0.38);
      padding: 12px;
    }

    .status strong {
      display: block;
      margin-top: 4px;
      color: var(--bone);
      font-size: 18px;
    }

    @media (max-width: 430px) {
      body {
        padding: 14px;
      }

      main {
        border-radius: 20px;
        padding: 16px;
      }

      header,
      .drag-grid,
      .status {
        grid-template-columns: 1fr;
      }

      header {
        display: grid;
      }

      .modes {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>)rawliteral");
  page += carName;
  page += F(R"rawliteral(</h1>
      <div class="chip">Open Wi-Fi</div>
    </header>

    <section class="panel">
      <div class="drag-grid">
        <button class="primary" id="dragButton" type="button">Drag Race</button>
        <div>
          <label for="dragSeconds">Seconds</label>
          <input id="dragSeconds" type="number" min="0.1" max="60" step="0.1" value="3">
        </div>
      </div>
    </section>

    <section class="panel">
      <label>Direction</label>
      <div class="modes">
        <button class="mode stop active" data-direction="stop" type="button">Stop</button>
        <button class="mode" data-direction="forward" type="button">Forward</button>
        <button class="mode" data-direction="backward" type="button">Backward</button>
      </div>
    </section>

    <section class="panel">
      <label for="duty">PWM Duty Cycle</label>
      <div class="slider-row">
        <input id="duty" type="range" min="0" max="100" step="1" value="0">
        <div class="percent"><span id="dutyText">0</span>%</div>
      </div>
    </section>

    <section class="status">
      <div>Direction<strong id="directionText">Stop</strong></div>
      <div>Drag Race<strong id="dragText">Ready</strong></div>
    </section>
  </main>

  <script>
    const duty = document.getElementById('duty');
    const dutyText = document.getElementById('dutyText');
    const dragSeconds = document.getElementById('dragSeconds');
    const dragButton = document.getElementById('dragButton');
    const directionText = document.getElementById('directionText');
    const dragText = document.getElementById('dragText');
    const modeButtons = Array.from(document.querySelectorAll('.mode'));
    let dutyTimer = 0;

    function titleCase(value) {
      return value.charAt(0).toUpperCase() + value.slice(1);
    }

    function setBusy(button, busy) {
      button.disabled = busy;
      button.style.opacity = busy ? '0.7' : '1';
    }

    async function post(path, data) {
      const body = new URLSearchParams(data);
      const response = await fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body
      });

      if (!response.ok) {
        throw new Error(await response.text());
      }

      return response.json();
    }

    function renderStatus(status) {
      duty.value = status.duty;
      dutyText.textContent = status.duty;
      directionText.textContent = titleCase(status.direction);
      dragText.textContent = status.dragActive
        ? (Math.ceil(status.dragRemainingMs / 100) / 10).toFixed(1) + 's'
        : 'Ready';

      modeButtons.forEach(button => {
        button.classList.toggle('active', button.dataset.direction === status.direction);
      });
    }

    modeButtons.forEach(button => {
      button.addEventListener('click', async () => {
        setBusy(button, true);
        try {
          renderStatus(await post('/direction', {value: button.dataset.direction}));
        } catch (error) {
          alert(error.message);
        } finally {
          setBusy(button, false);
        }
      });
    });

    duty.addEventListener('input', () => {
      dutyText.textContent = duty.value;
      clearTimeout(dutyTimer);
      dutyTimer = setTimeout(async () => {
        try {
          renderStatus(await post('/duty', {value: duty.value}));
        } catch (error) {
          alert(error.message);
        }
      }, 120);
    });

    duty.addEventListener('change', async () => {
      clearTimeout(dutyTimer);
      try {
        renderStatus(await post('/duty', {value: duty.value}));
      } catch (error) {
        alert(error.message);
      }
    });

    dragButton.addEventListener('click', async () => {
      setBusy(dragButton, true);
      try {
        renderStatus(await post('/drag', {seconds: dragSeconds.value}));
      } catch (error) {
        alert(error.message);
      } finally {
        setBusy(dragButton, false);
      }
    });

    async function refresh() {
      try {
        const response = await fetch('/status', {cache: 'no-store'});
        renderStatus(await response.json());
      } catch (error) {
      }
    }

    refresh();
    setInterval(refresh, 500);
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

void handleNotFound() {
  server.send(404, F("text/plain"), F("Not found"));
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
  writeForwardPwmPercent(0);
  writeReversePwmPercent(0);
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_REVERSE_PIN, OUTPUT);
  setupPwm();
  applyMotor(MotorDirection::Stop, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(CAR_NAME);

  server.on(F("/"), HTTP_GET, handleIndex);
  server.on(F("/status"), HTTP_GET, sendStatus);
  server.on(F("/direction"), HTTP_POST, handleDirection);
  server.on(F("/duty"), HTTP_POST, handleDuty);
  server.on(F("/drag"), HTTP_POST, handleDragRace);
  server.on(F("/stop"), HTTP_POST, handleStop);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println();
  Serial.print(F("Wi-Fi AP: "));
  Serial.println(CAR_NAME);
  Serial.print(F("Open http://"));
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  if (dragRaceActive && static_cast<int32_t>(millis() - dragRaceEndMs) >= 0) {
    cancelDragRace();
    applyMotor(MotorDirection::Stop, 0);
  }
}
