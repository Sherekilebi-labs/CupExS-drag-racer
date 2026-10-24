#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>

const char *CONTROLLER_WIFI_NAME = "drag controller";
const char *CONTROLLER_WIFI_PASSWORD = "CHANGE_ME_123";

const uint8_t LANE_1_RECEIVER_PIN = 13;
const uint8_t LANE_2_RECEIVER_PIN = 21;
const uint8_t LANE_1_LED_PIN = 4;
const uint8_t LANE_2_LED_PIN = 33;

const uint32_t IR_CARRIER_HZ = 36000;
const uint8_t IR_PWM_RESOLUTION_BITS = 10;
const uint16_t IR_PWM_DUTY = 512;
const uint8_t LANE_1_LED_CHANNEL = 0;
const uint8_t LANE_2_LED_CHANNEL = 1;

// TSOP34836 needs bursts, not continuous carrier. 18 carrier cycles at 36 kHz
// is 500 us; 14 cycles gap is 389 us, keeping each lane refreshed under 2 ms.
const uint32_t IR_BURST_US = 500;
const uint32_t IR_SAMPLE_US = 430;
const uint32_t IR_GAP_US = 390;
const uint8_t BEAM_MISSED_SAMPLES = 2;

enum class IrPhase {
  Lane1Burst,
  Lane1Gap,
  Lane2Burst,
  Lane2Gap
};

enum class RaceState {
  Idle,
  Countdown,
  Racing,
  Complete
};

struct LaneState {
  const uint8_t receiverPin;
  volatile uint32_t lastSeenUs;
  uint32_t finishUs;
  bool finished;
  bool armed;
  bool sampledThisBurst;
  bool beamPresent;
  uint8_t missedSamples;
};

WebServer server(80);

LaneState lane1 = {LANE_1_RECEIVER_PIN, 0, 0, false, false, false, false, BEAM_MISSED_SAMPLES};
LaneState lane2 = {LANE_2_RECEIVER_PIN, 0, 0, false, false, false, false, BEAM_MISSED_SAMPLES};

IrPhase irPhase = IrPhase::Lane1Burst;
uint32_t irPhaseStartedUs = 0;

RaceState raceState = RaceState::Idle;
uint32_t countdownStartedMs = 0;
uint32_t greenAtMs = 0;
uint32_t raceStartedUs = 0;

void writeLaneLed(uint8_t channel, uint8_t pin, uint16_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void setLane1Led(bool enabled) {
  writeLaneLed(LANE_1_LED_CHANNEL, LANE_1_LED_PIN, enabled ? IR_PWM_DUTY : 0);
}

void setLane2Led(bool enabled) {
  writeLaneLed(LANE_2_LED_CHANNEL, LANE_2_LED_PIN, enabled ? IR_PWM_DUTY : 0);
}

void setIrPhase(IrPhase phase) {
  irPhase = phase;
  irPhaseStartedUs = micros();
  lane1.sampledThisBurst = false;
  lane2.sampledThisBurst = false;

  setLane1Led(phase == IrPhase::Lane1Burst);
  setLane2Led(phase == IrPhase::Lane2Burst);
}

bool receiverSeesIr(const LaneState &lane) {
  return digitalRead(lane.receiverPin) == LOW;
}

void sampleLane(LaneState &lane) {
  if (receiverSeesIr(lane)) {
    lane.lastSeenUs = micros();
    lane.beamPresent = true;
    lane.missedSamples = 0;
  } else if (lane.missedSamples < BEAM_MISSED_SAMPLES) {
    lane.missedSamples++;
    if (lane.missedSamples >= BEAM_MISSED_SAMPLES) {
      lane.beamPresent = false;
    }
  } else {
    lane.beamPresent = false;
  }
}

bool beamPresent(const LaneState &lane, uint32_t nowUs) {
  (void)nowUs;
  return lane.beamPresent;
}

void updateIrBeams() {
  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - irPhaseStartedUs;

  switch (irPhase) {
  case IrPhase::Lane1Burst:
    if (!lane1.sampledThisBurst && elapsedUs >= IR_SAMPLE_US) {
      sampleLane(lane1);
      lane1.sampledThisBurst = true;
    }
    if (elapsedUs >= IR_BURST_US) {
      sampleLane(lane1);
      setIrPhase(IrPhase::Lane1Gap);
    }
    break;

  case IrPhase::Lane1Gap:
    if (elapsedUs >= IR_GAP_US) {
      setIrPhase(IrPhase::Lane2Burst);
    }
    break;

  case IrPhase::Lane2Burst:
    if (!lane2.sampledThisBurst && elapsedUs >= IR_SAMPLE_US) {
      sampleLane(lane2);
      lane2.sampledThisBurst = true;
    }
    if (elapsedUs >= IR_BURST_US) {
      sampleLane(lane2);
      setIrPhase(IrPhase::Lane2Gap);
    }
    break;

  case IrPhase::Lane2Gap:
    if (elapsedUs >= IR_GAP_US) {
      setIrPhase(IrPhase::Lane1Burst);
    }
    break;
  }
}

void resetRaceLanes() {
  lane1.finished = false;
  lane2.finished = false;
  lane1.finishUs = 0;
  lane2.finishUs = 0;
  lane1.armed = false;
  lane2.armed = false;
}

void beginCountdown() {
  resetRaceLanes();
  raceState = RaceState::Countdown;
  countdownStartedMs = millis();
  greenAtMs = countdownStartedMs + random(3000, 5001);
}

void setGreen() {
  uint32_t nowUs = micros();
  raceStartedUs = nowUs;
  lane1.armed = beamPresent(lane1, nowUs);
  lane2.armed = beamPresent(lane2, nowUs);
  raceState = RaceState::Racing;
}

void finishLaneIfBroken(LaneState &lane, uint32_t nowUs) {
  if (!lane.armed && beamPresent(lane, nowUs)) {
    lane.armed = true;
  }

  if (!lane.finished && lane.armed && !beamPresent(lane, nowUs)) {
    lane.finished = true;
    lane.finishUs = nowUs;
  }
}

void updateRace() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  if (raceState == RaceState::Countdown && static_cast<int32_t>(nowMs - greenAtMs) >= 0) {
    setGreen();
  }

  if (raceState == RaceState::Racing) {
    finishLaneIfBroken(lane1, nowUs);
    finishLaneIfBroken(lane2, nowUs);

    if (lane1.finished && lane2.finished) {
      raceState = RaceState::Complete;
    }
  }
}

String raceStateText() {
  switch (raceState) {
  case RaceState::Countdown:
    return F("countdown");
  case RaceState::Racing:
    return F("racing");
  case RaceState::Complete:
    return F("complete");
  case RaceState::Idle:
  default:
    return F("idle");
  }
}

String laneTimeJson(const LaneState &lane) {
  if (!lane.finished || raceStartedUs == 0) {
    return F("null");
  }

  float seconds = static_cast<float>(lane.finishUs - raceStartedUs) / 1000000.0f;
  return String(seconds, 3);
}

String buildStatusJson() {
  uint32_t nowUs = micros();
  String json = F("{\"state\":\"");
  json += raceStateText();
  json += F("\",\"countdownRemainingMs\":");

  if (raceState == RaceState::Countdown) {
    json += static_cast<uint32_t>(greenAtMs - millis());
  } else {
    json += 0;
  }

  json += F(",\"lane1Time\":");
  json += laneTimeJson(lane1);
  json += F(",\"lane2Time\":");
  json += laneTimeJson(lane2);
  json += F(",\"lane1Beam\":");
  json += beamPresent(lane1, nowUs) ? F("true") : F("false");
  json += F(",\"lane2Beam\":");
  json += beamPresent(lane2, nowUs) ? F("true") : F("false");
  json += '}';
  return json;
}

void sendStatus() {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), buildStatusJson());
}

String buildIndexHtml() {
  return F(R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drag Controller</title>
  <style>
    :root {
      color-scheme: dark;
      --bone: #f7f1e4;
      --muted: #d7ceb9;
      --track: #111018;
      --panel: rgba(247, 241, 228, 0.09);
      --orange: #ff8a2a;
      --purple: #a45cff;
      --red: #ff2d46;
      --amber: #ffbf2f;
      --green: #38ff8b;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background:
        linear-gradient(90deg, rgba(247, 241, 228, 0.04) 1px, transparent 1px),
        radial-gradient(circle at 18% 14%, rgba(255, 138, 42, 0.18), transparent 30%),
        radial-gradient(circle at 86% 6%, rgba(164, 92, 255, 0.22), transparent 28%),
        linear-gradient(145deg, #09070d 0%, #17101f 48%, #241421 100%);
      background-size: 42px 42px, auto, auto, auto;
      color: var(--bone);
      font-family: Arial, Helvetica, sans-serif;
      padding: 18px;
    }

    main {
      width: min(920px, 100%);
      margin: 0 auto;
      display: grid;
      gap: 16px;
    }

    .topbar {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 14px;
      align-items: center;
      padding: 18px;
      border: 1px solid rgba(247, 241, 228, 0.14);
      border-radius: 18px;
      background: rgba(247, 241, 228, 0.07);
      box-shadow: 0 0 36px rgba(164, 92, 255, 0.14), 0 0 42px rgba(255, 138, 42, 0.08);
    }

    h1 {
      margin: 0;
      font-size: clamp(24px, 6vw, 42px);
      letter-spacing: 0;
      text-transform: uppercase;
      text-shadow: 0 0 18px rgba(255, 138, 42, 0.25);
    }

    button {
      min-height: 56px;
      border: 1px solid rgba(255, 255, 255, 0.28);
      border-radius: 999px;
      background: linear-gradient(135deg, var(--red), #b70d2a);
      color: white;
      font-size: 18px;
      font-weight: 800;
      text-transform: uppercase;
      padding: 0 30px;
      cursor: pointer;
      touch-action: manipulation;
      box-shadow: 0 0 28px rgba(255, 45, 70, 0.42);
      -webkit-tap-highlight-color: transparent;
    }

    button:active {
      transform: translateY(1px) scale(0.99);
    }

    .tree {
      display: grid;
      grid-template-columns: repeat(5, minmax(46px, 1fr));
      gap: 12px;
      padding: 16px;
      border: 1px solid rgba(247, 241, 228, 0.13);
      border-radius: 18px;
      background: linear-gradient(180deg, rgba(247, 241, 228, 0.10), rgba(247, 241, 228, 0.045));
    }

    .bulb {
      aspect-ratio: 1;
      border-radius: 999px;
      background: #25232b;
      border: 2px solid rgba(247, 241, 228, 0.12);
      box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.58);
    }

    .bulb.red.on {
      background: var(--red);
      box-shadow: 0 0 26px rgba(255, 45, 70, 0.64), inset 0 0 12px rgba(255, 255, 255, 0.18);
    }

    .bulb.amber.on {
      background: var(--amber);
      box-shadow: 0 0 26px rgba(255, 191, 47, 0.64), inset 0 0 12px rgba(255, 255, 255, 0.18);
    }

    .bulb.green.on {
      background: var(--green);
      box-shadow: 0 0 30px rgba(56, 255, 139, 0.72), inset 0 0 12px rgba(255, 255, 255, 0.18);
    }

    .lane-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
    }

    .lane {
      position: relative;
      overflow: hidden;
      min-height: 210px;
      border: 1px solid rgba(247, 241, 228, 0.14);
      border-radius: 18px;
      background:
        linear-gradient(90deg, transparent 48%, rgba(247, 241, 228, 0.22) 49%, rgba(247, 241, 228, 0.22) 51%, transparent 52%),
        repeating-linear-gradient(0deg, rgba(247, 241, 228, 0.08) 0 2px, transparent 2px 22px),
        linear-gradient(180deg, #17151d, #0d0c12);
      padding: 18px;
      box-shadow: inset 0 0 40px rgba(0, 0, 0, 0.34);
    }

    .lane::after {
      content: "";
      position: absolute;
      left: 0;
      right: 0;
      bottom: 38px;
      height: 10px;
      background: repeating-linear-gradient(90deg, var(--bone) 0 20px, transparent 20px 40px);
      opacity: 0.82;
    }

    .lane h2 {
      margin: 0;
      font-size: 20px;
      letter-spacing: 0;
      text-transform: uppercase;
      color: var(--muted);
    }

    .time {
      margin-top: 34px;
      font-size: clamp(44px, 10vw, 74px);
      font-weight: 900;
      letter-spacing: 0;
      text-shadow: 0 0 22px rgba(164, 92, 255, 0.24);
    }

    .beam {
      position: absolute;
      right: 16px;
      top: 16px;
      border: 1px solid rgba(247, 241, 228, 0.18);
      border-radius: 999px;
      padding: 7px 10px;
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      background: rgba(0, 0, 0, 0.25);
    }

    .beam.ok {
      color: var(--green);
      box-shadow: 0 0 16px rgba(56, 255, 139, 0.22);
    }

    .beam.break {
      color: var(--red);
      box-shadow: 0 0 16px rgba(255, 45, 70, 0.24);
    }

    .state {
      color: var(--muted);
      text-transform: uppercase;
      font-size: 13px;
      padding-left: 2px;
    }

    @media (max-width: 700px) {
      .topbar,
      .lane-grid {
        grid-template-columns: 1fr;
      }

      button {
        width: 100%;
      }
    }
  </style>
</head>
<body>
  <main>
    <section class="topbar">
      <div>
        <h1>Drag Controller</h1>
        <div class="state" id="stateText">Idle</div>
      </div>
      <button id="startButton" type="button">Start</button>
    </section>

    <section class="tree">
      <div class="bulb red" id="red1"></div>
      <div class="bulb amber" id="amber1"></div>
      <div class="bulb amber" id="amber2"></div>
      <div class="bulb amber" id="amber3"></div>
      <div class="bulb green" id="green"></div>
    </section>

    <section class="lane-grid">
      <article class="lane">
        <div class="beam" id="lane1Beam">Beam</div>
        <h2>Lane 1</h2>
        <div class="time" id="lane1Time">--.---</div>
      </article>
      <article class="lane">
        <div class="beam" id="lane2Beam">Beam</div>
        <h2>Lane 2</h2>
        <div class="time" id="lane2Time">--.---</div>
      </article>
    </section>
  </main>

  <script>
    const startButton = document.getElementById('startButton');
    const stateText = document.getElementById('stateText');
    const lane1Time = document.getElementById('lane1Time');
    const lane2Time = document.getElementById('lane2Time');
    const lane1Beam = document.getElementById('lane1Beam');
    const lane2Beam = document.getElementById('lane2Beam');
    const bulbs = {
      red1: document.getElementById('red1'),
      amber1: document.getElementById('amber1'),
      amber2: document.getElementById('amber2'),
      amber3: document.getElementById('amber3'),
      green: document.getElementById('green')
    };

    function clearBulbs() {
      Object.values(bulbs).forEach(bulb => bulb.classList.remove('on'));
    }

    function renderTree(status) {
      clearBulbs();

      if (status.state === 'countdown') {
        const elapsed = Math.max(0, 5000 - status.countdownRemainingMs);
        bulbs.red1.classList.add('on');
        if (elapsed > 700) bulbs.amber1.classList.add('on');
        if (elapsed > 1400) bulbs.amber2.classList.add('on');
        if (elapsed > 2100) bulbs.amber3.classList.add('on');
      } else if (status.state === 'racing' || status.state === 'complete') {
        bulbs.green.classList.add('on');
      }
    }

    function renderBeam(element, present) {
      element.textContent = present ? 'Beam OK' : 'Broken';
      element.classList.toggle('ok', present);
      element.classList.toggle('break', !present);
    }

    function renderStatus(status) {
      stateText.textContent = status.state;
      lane1Time.textContent = status.lane1Time === null ? '--.---' : status.lane1Time.toFixed(3);
      lane2Time.textContent = status.lane2Time === null ? '--.---' : status.lane2Time.toFixed(3);
      renderBeam(lane1Beam, status.lane1Beam);
      renderBeam(lane2Beam, status.lane2Beam);
      renderTree(status);
    }

    startButton.addEventListener('click', async () => {
      startButton.disabled = true;
      try {
        const response = await fetch('/start', {method: 'POST'});
        if (!response.ok) throw new Error(await response.text());
        renderStatus(await response.json());
      } catch (error) {
        alert(error.message);
      } finally {
        startButton.disabled = false;
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
    setInterval(refresh, 50);
  </script>
</body>
</html>
)rawliteral");
}

void handleIndex() {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), buildIndexHtml());
}

void handleStart() {
  beginCountdown();
  sendStatus();
}

void setupIrPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LANE_1_LED_PIN, IR_CARRIER_HZ, IR_PWM_RESOLUTION_BITS);
  ledcAttach(LANE_2_LED_PIN, IR_CARRIER_HZ, IR_PWM_RESOLUTION_BITS);
#else
  ledcSetup(LANE_1_LED_CHANNEL, IR_CARRIER_HZ, IR_PWM_RESOLUTION_BITS);
  ledcSetup(LANE_2_LED_CHANNEL, IR_CARRIER_HZ, IR_PWM_RESOLUTION_BITS);
  ledcAttachPin(LANE_1_LED_PIN, LANE_1_LED_CHANNEL);
  ledcAttachPin(LANE_2_LED_PIN, LANE_2_LED_CHANNEL);
#endif
  setLane1Led(false);
  setLane2Led(false);
}

void setup() {
  Serial.begin(115200);

  pinMode(LANE_1_RECEIVER_PIN, INPUT_PULLUP);
  pinMode(LANE_2_RECEIVER_PIN, INPUT_PULLUP);
  pinMode(LANE_1_LED_PIN, OUTPUT);
  pinMode(LANE_2_LED_PIN, OUTPUT);

  setupIrPwm();
  randomSeed(esp_random());
  lane1.lastSeenUs = micros();
  lane2.lastSeenUs = micros();
  setIrPhase(IrPhase::Lane1Burst);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONTROLLER_WIFI_NAME, CONTROLLER_WIFI_PASSWORD);

  server.on(F("/"), HTTP_GET, handleIndex);
  server.on(F("/status"), HTTP_GET, sendStatus);
  server.on(F("/start"), HTTP_POST, handleStart);
  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("Not found"));
  });
  server.begin();

  Serial.println();
  Serial.print(F("Wi-Fi AP: "));
  Serial.println(CONTROLLER_WIFI_NAME);
  Serial.print(F("Open http://"));
  Serial.println(WiFi.softAPIP());
}

void loop() {
  updateIrBeams();
  updateRace();
  server.handleClient();
}
