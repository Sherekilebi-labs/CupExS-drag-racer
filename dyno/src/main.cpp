#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

// Existing motor pin mapping. Swap these two if your H-bridge follows the wiring note exactly.
constexpr int MOTOR_FORWARD_PIN = 19;
constexpr int MOTOR_BACKWARD_PIN = 21;
constexpr int CURRENT_SENSOR_PIN = 35;

// MT6701 magnetic encoder module wiring:
// VDD -> ESP32 3V3, GND -> common GND, SDA -> GPIO21, SCL -> GPIO5.
// GPIO22 is already used by the motor driver, so do not use the default ESP32 SCL pin.
constexpr int MT6701_SDA_PIN = 21;
constexpr int MT6701_SCL_PIN = 5;
constexpr uint8_t MT6701_I2C_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_HIGH_REG = 0x03;
constexpr uint16_t MT6701_CPR = 16384;
constexpr uint16_t MT6701_QUARTER_CPR = MT6701_CPR / 4;
constexpr int8_t ENCODER_DIRECTION_SIGN = -1;  // Mirrors the MT6701 direction so forward is positive.
constexpr uint32_t MT6701_SAMPLE_INTERVAL_US = 500;  // Supports quarter tracking at 10000 rpm.
constexpr uint32_t MT6701_I2C_CLOCK_HZ = 800000;

// PWM settings. 20 kHz keeps the motor quieter than low-frequency PWM.
constexpr int PWM_FORWARD_CHANNEL = 0;
constexpr int PWM_BACKWARD_CHANNEL = 1;
constexpr int PWM_FREQUENCY_HZ = 20000;
constexpr int PWM_RESOLUTION_BITS = 10;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;

// ACS712 30A module: about 66 mV/A at the sensor output, centered near VCC / 2.
constexpr float ACS712_SENSITIVITY_V_PER_A = 0.066f;
constexpr float DIVIDER_TOP_OHMS = 1000.0f;     // Sensor OUT to GPIO35.
constexpr float DIVIDER_BOTTOM_OHMS = 2000.0f;  // GPIO35 to GND.
constexpr float DIVIDER_SCALE =
    (DIVIDER_TOP_OHMS + DIVIDER_BOTTOM_OHMS) / DIVIDER_BOTTOM_OHMS;

// Control loop timing. Lower values react faster but make ADC noise more visible.
constexpr uint32_t CONTROL_INTERVAL_MS = 10;
constexpr uint32_t DRAG_MODE_DURATION_MS = 10000;
constexpr uint32_t DYNO_SAMPLE_INTERVAL_MS = 10;  // 100 measurements per second.
constexpr int DYNO_AVERAGE_COUNT = 10;
constexpr int DYNO_BUFFER_SIZE = 600;
constexpr int DYNO_STATUS_SAMPLE_COUNT = 20;
constexpr int CURRENT_SAMPLE_COUNT = 12;

// WiFi access point. Connect to this AP and open http://192.168.4.1/.
const char *AP_SSID = "CupExS-Motor";
const char *AP_PASSWORD = "CHANGE_ME_123";

WebServer server(80);

int motorDirection = 0;  // -1 backward, 0 stop, 1 forward.
float requestedPwmPercent = 0.0f;
float appliedPwmPercent = 0.0f;

bool torqueLimiterEnabled = true;
float currentLimitA = 5.0f;

// PID values are in PWM-percent-per-second units for this velocity-style loop.
float pidKp = 80.0f;
float pidKi = 35.0f;
float pidKd = 4.0f;
float pidIntegral = 0.0f;
float pidLastError = 0.0f;
float closedLoopPwmPercent = 0.0f;

// Exponential moving average alpha: 1.0 = no filtering, lower = smoother/slower.
float currentFilterAlpha = 0.25f;
float currentZeroSensorVoltage = 2.5f;
float rawCurrentA = 0.0f;
float filteredCurrentA = 0.0f;
bool currentFilterPrimed = false;

bool dragModeActive = false;
uint32_t dragModeStartedAt = 0;
uint32_t lastControlAt = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool encoderOnline = false;
volatile bool encoderQuarterTransitionFlag = false;
volatile uint16_t encoderAngleRaw = 0;
volatile uint8_t encoderQuarter = 0;
volatile int8_t encoderLastQuarterDelta = 0;
volatile int64_t encoderQuarterTransitions = 0;
volatile int32_t encoderRevolutions = 0;
volatile float encoderRpm = 0.0f;

bool dynoActive = false;
bool dynoCompleted = false;
uint32_t dynoStartedAt = 0;
uint32_t dynoLastSampleAt = 0;
uint32_t dynoSampleId = 0;
float dynoMomentInertia = 0.001f;  // kg*m^2. Set this from the web UI for your flywheel/system.
float dynoLastRpm = 0.0f;
float dynoLastAverageOmega = 0.0f;
float dynoTorqueNm = 0.0f;
float dynoPowerW = 0.0f;
float dynoMaxPowerW = 0.0f;
float dynoMaxPowerRpm = 0.0f;

struct SimpleKalmanFilter {
  float estimate;
  float errorEstimate;
  float processNoise;
  float measurementNoise;
  bool initialized;
};

SimpleKalmanFilter dynoRpmFilter = {0.0f, 1.0f, 8.0f, 80.0f, false};
SimpleKalmanFilter dynoAccelerationFilter = {0.0f, 1.0f, 60.0f, 600.0f, false};

float dynoRpmAverageSamples[DYNO_AVERAGE_COUNT] = {};
float dynoAccelerationAverageSamples[DYNO_AVERAGE_COUNT] = {};
int dynoRpmAverageIndex = 0;
int dynoRpmAverageCount = 0;
int dynoAccelerationAverageIndex = 0;
int dynoAccelerationAverageCount = 0;

struct DynoSample {
  uint32_t id;
  float rpm;
  float torqueNm;
  float powerW;
};

DynoSample dynoSamples[DYNO_BUFFER_SIZE];
int dynoSampleWriteIndex = 0;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CupExS Motor Control</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fb;
      --panel: #ffffff;
      --ink: #17202a;
      --muted: #607080;
      --line: #d8e0ea;
      --accent: #0f766e;
      --danger: #b42318;
      --blue: #2563eb;
      --dyno: #111827;
      --dyno-line: #22c55e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: var(--bg);
      color: var(--ink);
    }
    main {
      width: min(980px, calc(100% - 24px));
      margin: 18px auto 28px;
    }
    h1 {
      margin: 0 0 16px;
      font-size: 28px;
      letter-spacing: 0;
    }
    h2 {
      margin: 0 0 14px;
      font-size: 18px;
      letter-spacing: 0;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 14px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.04);
    }
    .row {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 12px;
      margin: 10px 0;
    }
    .direction {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      width: min(100%, 480px);
    }
    button {
      min-height: 42px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #eef3f8;
      color: var(--ink);
      font-size: 15px;
      cursor: pointer;
    }
    button.active {
      background: var(--accent);
      border-color: var(--accent);
      color: #ffffff;
    }
    button.stop.active {
      background: var(--danger);
      border-color: var(--danger);
    }
    button.primary {
      background: var(--blue);
      border-color: var(--blue);
      color: #ffffff;
      min-width: 160px;
    }
    button:disabled {
      opacity: 0.55;
      cursor: not-allowed;
    }
    label {
      color: var(--muted);
      font-size: 14px;
    }
    input[type="range"] {
      width: min(100%, 520px);
    }
    input[type="number"] {
      width: 110px;
      min-height: 36px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 6px 8px;
      font-size: 15px;
    }
    input[type="checkbox"] {
      width: 18px;
      height: 18px;
    }
    .value {
      min-width: 72px;
      font-weight: 700;
      font-size: 18px;
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      min-height: 78px;
      background: #fbfdff;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 8px;
    }
    .metric strong {
      font-size: 22px;
    }
    canvas {
      display: block;
      width: 100%;
      height: 260px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #ffffff;
    }
    .dyno-panel {
      background: var(--dyno);
      color: #f8fafc;
      border-color: #243041;
    }
    .dyno-panel h2,
    .dyno-panel label {
      color: #f8fafc;
    }
    .dyno-panel .metric {
      background: #182230;
      border-color: #334155;
    }
    .dyno-panel .metric span {
      color: #cbd5e1;
    }
    .dyno-panel canvas {
      background: #0b1120;
      border-color: #334155;
    }
    .config-grid {
      display: grid;
      grid-template-columns: repeat(5, minmax(110px, 1fr));
      gap: 12px;
      align-items: end;
    }
    .field {
      display: grid;
      gap: 6px;
    }
    .slider-field {
      min-width: min(100%, 220px);
    }
    .slider-field label {
      display: flex;
      justify-content: space-between;
      gap: 10px;
    }
    .slider-field input[type="range"] {
      width: 100%;
    }
    @media (max-width: 720px) {
      .metrics, .config-grid, .direction {
        grid-template-columns: 1fr;
      }
      h1 {
        font-size: 24px;
      }
    }
  </style>
</head>
<body>
  <main>
    <h1>CupExS Motor Control</h1>

    <section>
      <h2>Motor Control</h2>
      <div class="direction">
        <button id="backwardBtn" type="button">Backward</button>
        <button id="stopBtn" class="stop active" type="button">Stop</button>
        <button id="forwardBtn" type="button">Forward</button>
      </div>
      <div class="row">
        <label for="pwmSlider">PWM duty cycle</label>
        <input id="pwmSlider" type="range" min="0" max="100" value="0">
        <div id="pwmLabel" class="value">0%</div>
      </div>
    </section>

    <section>
      <h2>Motor Current</h2>
      <div class="metrics">
        <div class="metric"><span>Current</span><strong id="currentNow">0.00 A</strong></div>
        <div class="metric"><span>Applied PWM</span><strong id="appliedPwm">0%</strong></div>
        <div class="metric"><span>Current Limit</span><strong id="limitNow">5.00 A</strong></div>
      </div>
      <div class="row">
        <canvas id="currentChart" width="900" height="260"></canvas>
      </div>
    </section>

    <section>
      <h2>Torque Limiter</h2>
      <div class="config-grid">
        <div class="field">
          <label for="limitInput">Max current (A)</label>
          <input id="limitInput" type="number" min="0" max="30" step="0.1" value="5.0">
        </div>
        <div class="field">
          <label for="kpInput">PID Kp</label>
          <input id="kpInput" type="number" min="0" step="1" value="80">
        </div>
        <div class="field">
          <label for="kiInput">PID Ki</label>
          <input id="kiInput" type="number" min="0" step="1" value="35">
        </div>
        <div class="field">
          <label for="kdInput">PID Kd</label>
          <input id="kdInput" type="number" min="0" step="0.5" value="4">
        </div>
        <div class="field">
          <label for="filterInput">Filter alpha</label>
          <input id="filterInput" type="number" min="0.02" max="1" step="0.01" value="0.25">
        </div>
      </div>
      <div class="row">
        <input id="limiterEnabled" type="checkbox" checked>
        <label for="limiterEnabled">Enable torque limiter</label>
        <button id="saveConfigBtn" class="primary" type="button">Save Settings</button>
      </div>
    </section>

    <section>
      <h2>Drag Mode</h2>
      <button id="dragBtn" class="primary" type="button">Start 10s Drag Mode</button>
    </section>

    <section class="dyno-panel">
      <h2>Dyno Tuning</h2>
      <div class="metrics">
        <div class="metric"><span>RPM</span><strong id="rpmNow">0</strong></div>
        <div class="metric"><span>Angle</span><strong id="angleNow">0.00 deg</strong></div>
        <div class="metric"><span>Torque</span><strong id="torqueNow">0.000 Nm</strong></div>
        <div class="metric"><span>Power</span><strong id="powerNow">0.0 W</strong></div>
        <div class="metric"><span>Revolutions</span><strong id="revsNow">0</strong></div>
        <div class="metric"><span>Peak Power</span><strong id="peakPowerNow">0.0 W @ 0 rpm</strong></div>
      </div>
      <div class="row">
        <div class="field">
          <label for="inertiaInput">Moment of inertia (kg*m^2)</label>
          <input id="inertiaInput" type="number" min="0.000001" step="0.0001" value="0.001">
        </div>
        <button id="dynoBtn" class="primary" type="button">Start Dyno Test</button>
      </div>
      <div class="config-grid">
        <div class="field slider-field">
          <label for="rpmProcessInput">RPM process <span id="rpmProcessLabel">8.0</span></label>
          <input id="rpmProcessInput" type="range" min="0.1" max="100" step="0.1" value="8">
        </div>
        <div class="field slider-field">
          <label for="rpmMeasurementInput">RPM measurement <span id="rpmMeasurementLabel">80</span></label>
          <input id="rpmMeasurementInput" type="range" min="1" max="1000" step="1" value="80">
        </div>
        <div class="field slider-field">
          <label for="accelProcessInput">Accel process <span id="accelProcessLabel">60</span></label>
          <input id="accelProcessInput" type="range" min="1" max="1000" step="1" value="60">
        </div>
        <div class="field slider-field">
          <label for="accelMeasurementInput">Accel measurement <span id="accelMeasurementLabel">600</span></label>
          <input id="accelMeasurementInput" type="range" min="10" max="5000" step="10" value="600">
        </div>
      </div>
      <div class="row">
        <label>Torque vs RPM</label>
        <canvas id="dynoChart" width="900" height="300"></canvas>
      </div>
      <div class="row">
        <label>Power vs RPM</label>
        <canvas id="dynoPowerChart" width="900" height="300"></canvas>
      </div>
    </section>
  </main>

  <script>
    const pwmSlider = document.getElementById('pwmSlider');
    const pwmLabel = document.getElementById('pwmLabel');
    const currentNow = document.getElementById('currentNow');
    const appliedPwm = document.getElementById('appliedPwm');
    const limitNow = document.getElementById('limitNow');
    const dragBtn = document.getElementById('dragBtn');
    const chart = document.getElementById('currentChart');
    const ctx = chart.getContext('2d');
    const dynoBtn = document.getElementById('dynoBtn');
    const dynoChart = document.getElementById('dynoChart');
    const dynoCtx = dynoChart.getContext('2d');
    const dynoPowerChart = document.getElementById('dynoPowerChart');
    const dynoPowerCtx = dynoPowerChart.getContext('2d');
    let direction = 0;
    let samples = [];
    let dynoSamples = [];
    let lastDynoSampleId = 0;
    let controlTimer = 0;
    const filterSliders = [
      ['rpmProcessInput', 'rpmProcessLabel', 1],
      ['rpmMeasurementInput', 'rpmMeasurementLabel', 0],
      ['accelProcessInput', 'accelProcessLabel', 0],
      ['accelMeasurementInput', 'accelMeasurementLabel', 0]
    ];

    function postForm(path, data) {
      const body = new URLSearchParams(data);
      return fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body
      });
    }

    function setDirection(nextDirection) {
      direction = nextDirection;
      sendControl();
      updateDirectionButtons();
    }

    function updateDirectionButtons() {
      document.getElementById('backwardBtn').classList.toggle('active', direction === -1);
      document.getElementById('stopBtn').classList.toggle('active', direction === 0);
      document.getElementById('forwardBtn').classList.toggle('active', direction === 1);
    }

    function sendControl() {
      clearTimeout(controlTimer);
      controlTimer = setTimeout(() => {
        postForm('/api/control', {
          pwm: pwmSlider.value,
          direction
        }).catch(() => {});
      }, 70);
    }

    function saveConfig() {
      postForm('/api/config', {
        limit: document.getElementById('limitInput').value,
        enabled: document.getElementById('limiterEnabled').checked ? 1 : 0,
        kp: document.getElementById('kpInput').value,
        ki: document.getElementById('kiInput').value,
        kd: document.getElementById('kdInput').value,
        alpha: document.getElementById('filterInput').value,
        rpmProcess: document.getElementById('rpmProcessInput').value,
        rpmMeasurement: document.getElementById('rpmMeasurementInput').value,
        accelProcess: document.getElementById('accelProcessInput').value,
        accelMeasurement: document.getElementById('accelMeasurementInput').value
      }).catch(() => {});
    }

    function updateFilterSliderLabels() {
      filterSliders.forEach(([inputId, labelId, digits]) => {
        const input = document.getElementById(inputId);
        document.getElementById(labelId).textContent = Number(input.value).toFixed(digits);
      });
    }

    function syncSliderValue(inputId, value) {
      const input = document.getElementById(inputId);
      if (document.activeElement !== input) {
        input.value = value;
      }
    }

    function startDragMode() {
      postForm('/api/drag', {}).catch(() => {});
    }

    function startDynoTest() {
      if (!dynoBtn.dataset.active || dynoBtn.dataset.active === '0') {
        dynoSamples = [];
        lastDynoSampleId = 0;
        drawDynoChart();
        drawDynoPowerChart();
      }
      postForm('/api/dyno-toggle', {
        inertia: document.getElementById('inertiaInput').value
      }).catch(() => {});
    }

    function drawChart() {
      const w = chart.width;
      const h = chart.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, w, h);
      ctx.strokeStyle = '#d8e0ea';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = 24 + i * ((h - 44) / 4);
        ctx.beginPath();
        ctx.moveTo(44, y);
        ctx.lineTo(w - 12, y);
        ctx.stroke();
      }
      const maxCurrent = Math.max(1, ...samples.map(p => p.current), Number(document.getElementById('limitInput').value || 1));
      ctx.fillStyle = '#607080';
      ctx.font = '12px Arial';
      ctx.fillText(maxCurrent.toFixed(1) + ' A', 6, 28);
      ctx.fillText('0 A', 18, h - 18);
      ctx.strokeStyle = '#0f766e';
      ctx.lineWidth = 2;
      ctx.beginPath();
      samples.forEach((p, index) => {
        const x = 44 + index * ((w - 58) / Math.max(1, samples.length - 1));
        const y = h - 20 - (p.current / maxCurrent) * (h - 48);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    function drawDynoChart() {
      const w = dynoChart.width;
      const h = dynoChart.height;
      dynoCtx.clearRect(0, 0, w, h);
      dynoCtx.fillStyle = '#0b1120';
      dynoCtx.fillRect(0, 0, w, h);
      dynoCtx.strokeStyle = '#334155';
      dynoCtx.lineWidth = 1;
      for (let i = 0; i <= 5; i++) {
        const y = 22 + i * ((h - 46) / 5);
        dynoCtx.beginPath();
        dynoCtx.moveTo(52, y);
        dynoCtx.lineTo(w - 12, y);
        dynoCtx.stroke();
      }
      const maxRpm = Math.max(1000, ...dynoSamples.map(p => p.rpm));
      const maxTorque = Math.max(0.01, ...dynoSamples.map(p => p.torque));
      dynoCtx.fillStyle = '#cbd5e1';
      dynoCtx.font = '12px Arial';
      dynoCtx.fillText(maxTorque.toFixed(2) + ' Nm', 6, 26);
      dynoCtx.fillText('0 Nm', 18, h - 16);
      dynoCtx.fillText(maxRpm.toFixed(0) + ' rpm', w - 96, h - 8);
      dynoCtx.strokeStyle = '#22c55e';
      dynoCtx.lineWidth = 2;
      dynoCtx.beginPath();
      dynoSamples.forEach((p, index) => {
        const x = 52 + (p.rpm / maxRpm) * (w - 70);
        const y = h - 24 - (p.torque / maxTorque) * (h - 52);
        if (index === 0) dynoCtx.moveTo(x, y);
        else dynoCtx.lineTo(x, y);
      });
      dynoCtx.stroke();
    }

    function drawDynoPowerChart() {
      const w = dynoPowerChart.width;
      const h = dynoPowerChart.height;
      dynoPowerCtx.clearRect(0, 0, w, h);
      dynoPowerCtx.fillStyle = '#0b1120';
      dynoPowerCtx.fillRect(0, 0, w, h);
      dynoPowerCtx.strokeStyle = '#334155';
      dynoPowerCtx.lineWidth = 1;
      for (let i = 0; i <= 5; i++) {
        const y = 22 + i * ((h - 46) / 5);
        dynoPowerCtx.beginPath();
        dynoPowerCtx.moveTo(52, y);
        dynoPowerCtx.lineTo(w - 12, y);
        dynoPowerCtx.stroke();
      }
      const maxRpm = Math.max(1000, ...dynoSamples.map(p => p.rpm));
      const maxPower = Math.max(1, ...dynoSamples.map(p => p.power));
      dynoPowerCtx.fillStyle = '#cbd5e1';
      dynoPowerCtx.font = '12px Arial';
      dynoPowerCtx.fillText(maxPower.toFixed(0) + ' W', 12, 26);
      dynoPowerCtx.fillText('0 W', 24, h - 16);
      dynoPowerCtx.fillText(maxRpm.toFixed(0) + ' rpm', w - 96, h - 8);
      dynoPowerCtx.strokeStyle = '#f97316';
      dynoPowerCtx.lineWidth = 2;
      dynoPowerCtx.beginPath();
      dynoSamples.forEach((p, index) => {
        const x = 52 + (p.rpm / maxRpm) * (w - 70);
        const y = h - 24 - (p.power / maxPower) * (h - 52);
        if (index === 0) dynoPowerCtx.moveTo(x, y);
        else dynoPowerCtx.lineTo(x, y);
      });
      dynoPowerCtx.stroke();
    }

    function applyStatus(status) {
      currentNow.textContent = status.current.toFixed(2) + ' A';
      appliedPwm.textContent = status.appliedPwm.toFixed(0) + '%';
      limitNow.textContent = status.currentLimit.toFixed(2) + ' A';
      dragBtn.disabled = status.dragActive;
      dragBtn.textContent = status.dragActive ? 'Drag Mode Running' : 'Start 10s Drag Mode';
      document.getElementById('rpmNow').textContent = status.rpm.toFixed(0);
      document.getElementById('angleNow').textContent = status.angleDeg.toFixed(2) + ' deg';
      document.getElementById('torqueNow').textContent = status.dynoTorque.toFixed(3) + ' Nm';
      document.getElementById('powerNow').textContent = status.dynoPowerW.toFixed(1) + ' W';
      document.getElementById('revsNow').textContent = status.revolutions;
      document.getElementById('peakPowerNow').textContent = status.dynoMaxPowerW.toFixed(1) + ' W @ ' + status.dynoMaxPowerRpm.toFixed(0) + ' rpm';
      syncSliderValue('rpmProcessInput', status.rpmProcessNoise);
      syncSliderValue('rpmMeasurementInput', status.rpmMeasurementNoise);
      syncSliderValue('accelProcessInput', status.accelProcessNoise);
      syncSliderValue('accelMeasurementInput', status.accelMeasurementNoise);
      updateFilterSliderLabels();
      dynoBtn.dataset.active = status.dynoActive ? '1' : '0';
      dynoBtn.textContent = status.dynoActive ? 'Stop Dyno Test' : 'Start Dyno Test';

      if (!status.dragActive && !status.dynoActive) {
        direction = status.direction;
        updateDirectionButtons();
      }

      if (status.dynoSamples && status.dynoSamples.length) {
        status.dynoSamples.forEach(sample => {
          if (sample.id > lastDynoSampleId) {
            lastDynoSampleId = sample.id;
            dynoSamples.push({ rpm: sample.rpm, torque: sample.torque, power: sample.power });
          }
        });
        drawDynoChart();
        drawDynoPowerChart();
      }

      samples.push({ time: Date.now(), current: status.current });
      const cutoff = Date.now() - 60000;
      samples = samples.filter(p => p.time >= cutoff);
      drawChart();
    }

    function pollStatus() {
      fetch('/api/status')
        .then(r => r.json())
        .then(applyStatus)
        .catch(() => {});
    }

    document.getElementById('backwardBtn').addEventListener('click', () => setDirection(-1));
    document.getElementById('stopBtn').addEventListener('click', () => setDirection(0));
    document.getElementById('forwardBtn').addEventListener('click', () => setDirection(1));
    document.getElementById('saveConfigBtn').addEventListener('click', saveConfig);
    dragBtn.addEventListener('click', startDragMode);
    dynoBtn.addEventListener('click', startDynoTest);
    filterSliders.forEach(([inputId]) => {
      document.getElementById(inputId).addEventListener('input', updateFilterSliderLabels);
      document.getElementById(inputId).addEventListener('change', saveConfig);
    });
    pwmSlider.addEventListener('input', () => {
      pwmLabel.textContent = pwmSlider.value + '%';
      sendControl();
    });

    updateDirectionButtons();
    updateFilterSliderLabels();
    drawDynoChart();
    drawDynoPowerChart();
    pollStatus();
    setInterval(pollStatus, 100);
  </script>
</body>
</html>
)rawliteral";

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

bool readMT6701AngleRaw(uint16_t &angleRaw) {
  Wire.beginTransmission(MT6701_I2C_ADDRESS);
  Wire.write(MT6701_ANGLE_HIGH_REG);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MT6701_I2C_ADDRESS, static_cast<uint8_t>(2)) != 2) {
    return false;
  }

  const uint8_t highByte = Wire.read();
  const uint8_t lowByte = Wire.read();
  angleRaw = ((static_cast<uint16_t>(highByte) << 6) | (lowByte >> 2)) & 0x3FFF;
  return true;
}

int unwrapAngleDelta(uint16_t currentRaw, uint16_t previousRaw) {
  int delta = static_cast<int>(currentRaw) - static_cast<int>(previousRaw);
  if (delta > MT6701_CPR / 2) {
    delta -= MT6701_CPR;
  } else if (delta < -MT6701_CPR / 2) {
    delta += MT6701_CPR;
  }
  return delta;
}

void encoderSampleTask(void *parameter) {
  (void)parameter;

  bool havePrevious = false;
  uint16_t previousRaw = 0;
  uint8_t previousQuarter = 0;
  uint32_t previousSampleUs = micros();
  int64_t localQuarterTransitions = 0;
  float localRpm = 0.0f;

  for (;;) {
    const uint32_t sampleStartedUs = micros();
    uint16_t raw = 0;
    const bool readOk = readMT6701AngleRaw(raw);

    if (readOk) {
      const uint32_t nowUs = micros();
      const uint8_t quarter = raw / MT6701_QUARTER_CPR;
      int8_t quarterDelta = 0;
      bool quarterChanged = false;

      if (havePrevious) {
        const int angleDelta = unwrapAngleDelta(raw, previousRaw) * ENCODER_DIRECTION_SIGN;
        const uint32_t dtUs = nowUs - previousSampleUs;

        if (dtUs > 0) {
          const float instantRpm =
              (angleDelta / static_cast<float>(MT6701_CPR)) * (60000000.0f / dtUs);
          localRpm = 0.30f * instantRpm + 0.70f * localRpm;
        }

        if (quarter != previousQuarter) {
          const uint8_t diff = (quarter + 4 - previousQuarter) % 4;
          if (diff == 1) {
            quarterDelta = ENCODER_DIRECTION_SIGN;
          } else if (diff == 3) {
            quarterDelta = -ENCODER_DIRECTION_SIGN;
          } else {
            quarterDelta = angleDelta >= 0 ? 2 : -2;
          }
          localQuarterTransitions += quarterDelta;
          quarterChanged = true;
        }
      } else {
        havePrevious = true;
        previousQuarter = quarter;
      }

      portENTER_CRITICAL(&encoderMux);
      encoderOnline = true;
      encoderAngleRaw = raw;
      encoderQuarter = quarter;
      encoderLastQuarterDelta = quarterDelta;
      encoderQuarterTransitionFlag = quarterChanged;
      encoderQuarterTransitions = localQuarterTransitions;
      encoderRevolutions = static_cast<int32_t>(localQuarterTransitions / 4);
      encoderRpm = localRpm;
      portEXIT_CRITICAL(&encoderMux);

      previousRaw = raw;
      previousQuarter = quarter;
      previousSampleUs = nowUs;
    } else {
      portENTER_CRITICAL(&encoderMux);
      encoderOnline = false;
      encoderRpm = 0.0f;
      portEXIT_CRITICAL(&encoderMux);
    }

    const uint32_t elapsedUs = micros() - sampleStartedUs;
    if (elapsedUs < MT6701_SAMPLE_INTERVAL_US) {
      delayMicroseconds(MT6701_SAMPLE_INTERVAL_US - elapsedUs);
    } else {
      taskYIELD();
    }
  }
}

void getEncoderSnapshot(bool &online, uint16_t &angleRaw, float &rpm, int32_t &revolutions,
                        int64_t &quarterTransitions, uint8_t &quarter, bool &quarterFlag) {
  portENTER_CRITICAL(&encoderMux);
  online = encoderOnline;
  angleRaw = encoderAngleRaw;
  rpm = encoderRpm;
  revolutions = encoderRevolutions;
  quarterTransitions = encoderQuarterTransitions;
  quarter = encoderQuarter;
  quarterFlag = encoderQuarterTransitionFlag;
  encoderQuarterTransitionFlag = false;
  portEXIT_CRITICAL(&encoderMux);
}

float rpmToRadPerSecond(float rpm) {
  return rpm * TWO_PI / 60.0f;
}

float angleRawToDegrees(uint16_t angleRaw) {
  return (angleRaw * 360.0f) / static_cast<float>(MT6701_CPR);
}

void resetKalmanFilter(SimpleKalmanFilter &filter) {
  filter.estimate = 0.0f;
  filter.errorEstimate = 1.0f;
  filter.initialized = false;
}

void setFilterNoise(SimpleKalmanFilter &filter, float processNoise, float measurementNoise) {
  filter.processNoise = processNoise;
  filter.measurementNoise = measurementNoise;
  resetKalmanFilter(filter);
}

// A compact 1D Kalman-style filter. Raise processNoise for faster response;
// raise measurementNoise for stronger smoothing of noisy RPM/acceleration values.
float updateKalmanFilter(SimpleKalmanFilter &filter, float measurement) {
  if (!filter.initialized) {
    filter.estimate = measurement;
    filter.initialized = true;
    return filter.estimate;
  }

  filter.errorEstimate += filter.processNoise;
  const float gain = filter.errorEstimate / (filter.errorEstimate + filter.measurementNoise);
  filter.estimate += gain * (measurement - filter.estimate);
  filter.errorEstimate *= (1.0f - gain);
  return filter.estimate;
}

float updateMovingAverage(float samples[], int sampleCount, int &index, int &count, float value) {
  samples[index] = value;
  index = (index + 1) % sampleCount;
  if (count < sampleCount) {
    count++;
  }

  float total = 0.0f;
  for (int i = 0; i < count; i++) {
    total += samples[i];
  }
  return total / count;
}

void appendDynoSamplesJson(String &json) {
  const uint32_t firstId =
      dynoSampleId > DYNO_STATUS_SAMPLE_COUNT ? dynoSampleId - DYNO_STATUS_SAMPLE_COUNT + 1 : 1;
  bool firstSample = true;

  json += ",\"dynoSamples\":[";
  for (uint32_t id = firstId; id <= dynoSampleId; id++) {
    for (int i = 0; i < DYNO_BUFFER_SIZE; i++) {
      if (dynoSamples[i].id == id) {
        if (!firstSample) {
          json += ",";
        }
        json += "{\"id\":" + String(dynoSamples[i].id);
        json += ",\"rpm\":" + String(dynoSamples[i].rpm, 1);
        json += ",\"torque\":" + String(dynoSamples[i].torqueNm, 5);
        json += ",\"power\":" + String(dynoSamples[i].powerW, 2);
        json += "}";
        firstSample = false;
        break;
      }
    }
  }
  json += "]";
}

void resetPid(float startPwmPercent) {
  pidIntegral = 0.0f;
  pidLastError = 0.0f;
  closedLoopPwmPercent = clampFloat(startPwmPercent, 0.0f, 100.0f);
}

void writeMotorPwm(int direction, float pwmPercent) {
  const float safePwm = clampFloat(pwmPercent, 0.0f, 100.0f);
  const int duty = roundf((safePwm / 100.0f) * PWM_MAX_DUTY);

  if (direction > 0 && duty > 0) {
    ledcWrite(PWM_FORWARD_CHANNEL, duty);
    ledcWrite(PWM_BACKWARD_CHANNEL, 0);
  } else if (direction < 0 && duty > 0) {
    ledcWrite(PWM_FORWARD_CHANNEL, 0);
    ledcWrite(PWM_BACKWARD_CHANNEL, duty);
  } else {
    ledcWrite(PWM_FORWARD_CHANNEL, 0);
    ledcWrite(PWM_BACKWARD_CHANNEL, 0);
  }
}

float readAdcVoltage() {
  uint32_t millivoltsTotal = 0;
  for (int i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    millivoltsTotal += analogReadMilliVolts(CURRENT_SENSOR_PIN);
    delayMicroseconds(250);
  }

  return (millivoltsTotal / static_cast<float>(CURRENT_SAMPLE_COUNT)) / 1000.0f;
}

// Converts GPIO35 voltage back through the divider, then ACS712 volts to amps.
float readMotorCurrentA() {
  const float gpioVoltage = readAdcVoltage();
  const float sensorVoltage = gpioVoltage * DIVIDER_SCALE;
  return (sensorVoltage - currentZeroSensorVoltage) / ACS712_SENSITIVITY_V_PER_A;
}

void updateCurrentReading() {
  rawCurrentA = readMotorCurrentA();
  const float currentMagnitudeA = fabsf(rawCurrentA);

  if (!currentFilterPrimed) {
    filteredCurrentA = currentMagnitudeA;
    currentFilterPrimed = true;
    return;
  }

  filteredCurrentA =
      currentFilterAlpha * currentMagnitudeA + (1.0f - currentFilterAlpha) * filteredCurrentA;
}

float calibrateCurrentZeroVoltage() {
  float voltageTotal = 0.0f;
  for (int i = 0; i < 120; i++) {
    voltageTotal += readAdcVoltage() * DIVIDER_SCALE;
    delay(4);
  }
  return voltageTotal / 120.0f;
}

// Velocity-style PID: positive current error raises PWM, negative error lowers it.
float updateCurrentLimitedPwm(float maxRequestedPwmPercent, float dtSeconds) {
  const float safeLimitA = clampFloat(currentLimitA, 0.0f, 30.0f);
  if (safeLimitA <= 0.0f || maxRequestedPwmPercent <= 0.0f) {
    resetPid(0.0f);
    return 0.0f;
  }

  const float error = safeLimitA - filteredCurrentA;
  pidIntegral += error * dtSeconds;
  pidIntegral = clampFloat(pidIntegral, -20.0f, 20.0f);

  const float derivative = (error - pidLastError) / dtSeconds;
  const float correctionPerSecond = pidKp * error + pidKi * pidIntegral + pidKd * derivative;

  closedLoopPwmPercent += correctionPerSecond * dtSeconds;
  closedLoopPwmPercent = clampFloat(closedLoopPwmPercent, 0.0f, maxRequestedPwmPercent);
  pidLastError = error;

  return closedLoopPwmPercent;
}

void endDragMode() {
  dragModeActive = false;
  requestedPwmPercent = 0.0f;
  motorDirection = 0;
  appliedPwmPercent = 0.0f;
  resetPid(0.0f);
  writeMotorPwm(0, 0.0f);
}

void clearDynoSamples() {
  dynoSampleWriteIndex = 0;
  for (int i = 0; i < DYNO_BUFFER_SIZE; i++) {
    dynoSamples[i] = {0, 0.0f, 0.0f, 0.0f};
  }
}

void resetDynoFilters() {
  for (int i = 0; i < DYNO_AVERAGE_COUNT; i++) {
    dynoRpmAverageSamples[i] = 0.0f;
    dynoAccelerationAverageSamples[i] = 0.0f;
  }
  dynoRpmAverageIndex = 0;
  dynoRpmAverageCount = 0;
  dynoAccelerationAverageIndex = 0;
  dynoAccelerationAverageCount = 0;
  resetKalmanFilter(dynoRpmFilter);
  resetKalmanFilter(dynoAccelerationFilter);
}

void recordDynoSample(float rpm, float torqueNm, float powerW) {
  dynoSamples[dynoSampleWriteIndex] = {dynoSampleId, rpm, torqueNm, powerW};
  dynoSampleWriteIndex = (dynoSampleWriteIndex + 1) % DYNO_BUFFER_SIZE;
}

void endDynoTest() {
  dynoActive = false;
  dynoCompleted = true;
  requestedPwmPercent = 0.0f;
  motorDirection = 0;
  appliedPwmPercent = 0.0f;
  resetPid(0.0f);
  writeMotorPwm(0, 0.0f);
}

void startDynoTest(float inertiaKgM2) {
  dynoMomentInertia = clampFloat(inertiaKgM2, 0.000001f, 100.0f);
  dynoActive = true;
  dynoCompleted = false;
  dynoStartedAt = millis();
  dynoLastSampleAt = dynoStartedAt;
  dynoSampleId = 0;
  dynoLastRpm = 0.0f;
  dynoLastAverageOmega = 0.0f;
  dynoTorqueNm = 0.0f;
  dynoPowerW = 0.0f;
  dynoMaxPowerW = 0.0f;
  dynoMaxPowerRpm = 0.0f;
  clearDynoSamples();
  resetDynoFilters();

  dragModeActive = false;
  motorDirection = 1;
  requestedPwmPercent = 100.0f;
  appliedPwmPercent = 100.0f;
  resetPid(100.0f);
  writeMotorPwm(motorDirection, appliedPwmPercent);
}

void updateDynoTest(uint32_t now) {
  if (!dynoActive || now - dynoLastSampleAt < DYNO_SAMPLE_INTERVAL_MS) {
    return;
  }

  bool encoderIsOnline = false;
  uint16_t angleRaw = 0;
  float measuredRpm = 0.0f;
  int32_t revolutions = 0;
  int64_t quarterTransitions = 0;
  uint8_t quarter = 0;
  bool quarterFlag = false;
  getEncoderSnapshot(encoderIsOnline, angleRaw, measuredRpm, revolutions, quarterTransitions, quarter,
                     quarterFlag);

  const uint32_t elapsedMs = now - dynoLastSampleAt;
  const float dtSeconds = elapsedMs / 1000.0f;
  const float rpmMagnitude = fabsf(measuredRpm);
  const float averagedRpm = updateMovingAverage(dynoRpmAverageSamples, DYNO_AVERAGE_COUNT,
                                                dynoRpmAverageIndex, dynoRpmAverageCount,
                                                rpmMagnitude);
  const float filteredRpm = updateKalmanFilter(dynoRpmFilter, averagedRpm);
  const float filteredOmega = rpmToRadPerSecond(filteredRpm);
  const float averageOmega = rpmToRadPerSecond(averagedRpm);
  const float rawAngularAcceleration = (averageOmega - dynoLastAverageOmega) / dtSeconds;
  const float averagedAngularAcceleration =
      updateMovingAverage(dynoAccelerationAverageSamples, DYNO_AVERAGE_COUNT,
                          dynoAccelerationAverageIndex, dynoAccelerationAverageCount,
                          rawAngularAcceleration);
  const float filteredAngularAcceleration =
      updateKalmanFilter(dynoAccelerationFilter, averagedAngularAcceleration);

  dynoTorqueNm = dynoMomentInertia * max(0.0f, filteredAngularAcceleration);
  dynoPowerW = filteredOmega * dynoTorqueNm;
  if (dynoPowerW > dynoMaxPowerW) {
    dynoMaxPowerW = dynoPowerW;
    dynoMaxPowerRpm = filteredRpm;
  }
  dynoSampleId++;
  recordDynoSample(filteredRpm, dynoTorqueNm, dynoPowerW);

  dynoLastRpm = filteredRpm;
  dynoLastAverageOmega = averageOmega;
  dynoLastSampleAt = now;
}

void updateMotorControl() {
  const uint32_t now = millis();
  const float dtSeconds = (now - lastControlAt) / 1000.0f;
  lastControlAt = now;

  updateCurrentReading();

  if (dynoActive) {
    motorDirection = 1;
    requestedPwmPercent = 100.0f;
    appliedPwmPercent = 100.0f;
    writeMotorPwm(motorDirection, appliedPwmPercent);
    updateDynoTest(now);
    return;
  }

  if (dragModeActive && now - dragModeStartedAt >= DRAG_MODE_DURATION_MS) {
    endDragMode();
    return;
  }

  const bool motorRequested = motorDirection != 0 && requestedPwmPercent > 0.0f;
  if (!motorRequested) {
    appliedPwmPercent = 0.0f;
    resetPid(0.0f);
    writeMotorPwm(0, 0.0f);
    return;
  }

  if (torqueLimiterEnabled || dragModeActive) {
    appliedPwmPercent = updateCurrentLimitedPwm(requestedPwmPercent, dtSeconds);
  } else {
    appliedPwmPercent = requestedPwmPercent;
    resetPid(appliedPwmPercent);
  }

  writeMotorPwm(motorDirection, appliedPwmPercent);
}

String buildStatusJson() {
  bool encoderIsOnline = false;
  uint16_t angleRaw = 0;
  float measuredRpm = 0.0f;
  int32_t revolutions = 0;
  int64_t quarterTransitions = 0;
  uint8_t quarter = 0;
  bool quarterFlag = false;
  getEncoderSnapshot(encoderIsOnline, angleRaw, measuredRpm, revolutions, quarterTransitions, quarter,
                     quarterFlag);

  String json = "{";
  json += "\"current\":" + String(filteredCurrentA, 3);
  json += ",\"rawCurrent\":" + String(rawCurrentA, 3);
  json += ",\"requestedPwm\":" + String(requestedPwmPercent, 1);
  json += ",\"appliedPwm\":" + String(appliedPwmPercent, 1);
  json += ",\"direction\":" + String(motorDirection);
  json += ",\"currentLimit\":" + String(currentLimitA, 2);
  json += ",\"limiterEnabled\":" + String(torqueLimiterEnabled ? "true" : "false");
  json += ",\"dragActive\":" + String(dragModeActive ? "true" : "false");
  json += ",\"encoderOnline\":" + String(encoderIsOnline ? "true" : "false");
  json += ",\"angleDeg\":" + String(angleRawToDegrees(angleRaw), 2);
  json += ",\"rpm\":" + String(fabsf(measuredRpm), 1);
  json += ",\"signedRpm\":" + String(measuredRpm, 1);
  json += ",\"revolutions\":" + String(revolutions);
  json += ",\"quarter\":" + String(quarter);
  json += ",\"quarterTransitions\":" + String(static_cast<long long>(quarterTransitions));
  json += ",\"quarterChanged\":" + String(quarterFlag ? "true" : "false");
  json += ",\"dynoActive\":" + String(dynoActive ? "true" : "false");
  json += ",\"dynoCompleted\":" + String(dynoCompleted ? "true" : "false");
  json += ",\"dynoSampleId\":" + String(dynoSampleId);
  json += ",\"dynoRpm\":" + String(dynoLastRpm, 1);
  json += ",\"dynoTorque\":" + String(dynoTorqueNm, 5);
  json += ",\"dynoPowerW\":" + String(dynoPowerW, 2);
  json += ",\"dynoMaxPowerW\":" + String(dynoMaxPowerW, 2);
  json += ",\"dynoMaxPowerRpm\":" + String(dynoMaxPowerRpm, 1);
  json += ",\"dynoInertia\":" + String(dynoMomentInertia, 6);
  json += ",\"rpmProcessNoise\":" + String(dynoRpmFilter.processNoise, 2);
  json += ",\"rpmMeasurementNoise\":" + String(dynoRpmFilter.measurementNoise, 2);
  json += ",\"accelProcessNoise\":" + String(dynoAccelerationFilter.processNoise, 2);
  json += ",\"accelMeasurementNoise\":" + String(dynoAccelerationFilter.measurementNoise, 2);
  appendDynoSamplesJson(json);
  json += "}";
  return json;
}

void sendStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleControl() {
  const float nextPwm =
      server.hasArg("pwm") ? server.arg("pwm").toFloat() : requestedPwmPercent;
  const int nextDirection =
      server.hasArg("direction") ? server.arg("direction").toInt() : motorDirection;

  if (dynoActive && nextDirection == 0) {
    endDynoTest();
  } else if (dragModeActive && nextDirection == 0) {
    endDragMode();
  } else if (!dragModeActive && !dynoActive) {
    requestedPwmPercent = clampFloat(nextPwm, 0.0f, 100.0f);
    motorDirection = constrain(nextDirection, -1, 1);
    closedLoopPwmPercent = clampFloat(closedLoopPwmPercent, 0.0f, requestedPwmPercent);
  }

  sendStatus();
}

void handleConfig() {
  if (server.hasArg("limit")) {
    currentLimitA = clampFloat(server.arg("limit").toFloat(), 0.0f, 30.0f);
  }
  if (server.hasArg("enabled")) {
    torqueLimiterEnabled = server.arg("enabled").toInt() != 0;
  }
  if (server.hasArg("kp")) {
    pidKp = clampFloat(server.arg("kp").toFloat(), 0.0f, 500.0f);
  }
  if (server.hasArg("ki")) {
    pidKi = clampFloat(server.arg("ki").toFloat(), 0.0f, 500.0f);
  }
  if (server.hasArg("kd")) {
    pidKd = clampFloat(server.arg("kd").toFloat(), 0.0f, 100.0f);
  }
  if (server.hasArg("alpha")) {
    currentFilterAlpha = clampFloat(server.arg("alpha").toFloat(), 0.02f, 1.0f);
  }
  const float rpmProcess =
      server.hasArg("rpmProcess")
          ? clampFloat(server.arg("rpmProcess").toFloat(), 0.1f, 100.0f)
          : dynoRpmFilter.processNoise;
  const float rpmMeasurement =
      server.hasArg("rpmMeasurement")
          ? clampFloat(server.arg("rpmMeasurement").toFloat(), 1.0f, 1000.0f)
          : dynoRpmFilter.measurementNoise;
  const float accelProcess =
      server.hasArg("accelProcess")
          ? clampFloat(server.arg("accelProcess").toFloat(), 1.0f, 1000.0f)
          : dynoAccelerationFilter.processNoise;
  const float accelMeasurement =
      server.hasArg("accelMeasurement")
          ? clampFloat(server.arg("accelMeasurement").toFloat(), 10.0f, 5000.0f)
          : dynoAccelerationFilter.measurementNoise;
  setFilterNoise(dynoRpmFilter, rpmProcess, rpmMeasurement);
  setFilterNoise(dynoAccelerationFilter, accelProcess, accelMeasurement);

  resetPid(appliedPwmPercent);
  sendStatus();
}

void handleDragMode() {
  if (!dragModeActive && !dynoActive) {
    dragModeActive = true;
    dragModeStartedAt = millis();
    motorDirection = 1;
    requestedPwmPercent = 100.0f;
    appliedPwmPercent = 0.0f;
    resetPid(0.0f);
  }

  sendStatus();
}

void handleDynoTest() {
  if (dynoActive) {
    endDynoTest();
  } else {
    const float inertia =
        server.hasArg("inertia") ? server.arg("inertia").toFloat() : dynoMomentInertia;
    startDynoTest(inertia);
  }

  sendStatus();
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/control", HTTP_POST, handleControl);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/drag", HTTP_POST, handleDragMode);
  server.on("/api/dyno", HTTP_POST, handleDynoTest);
  server.on("/api/dyno-toggle", HTTP_POST, handleDynoTest);
  server.begin();
}

void setupMotorPwm() {
  ledcSetup(PWM_FORWARD_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(PWM_BACKWARD_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_FORWARD_PIN, PWM_FORWARD_CHANNEL);
  ledcAttachPin(MOTOR_BACKWARD_PIN, PWM_BACKWARD_CHANNEL);
  writeMotorPwm(0, 0.0f);
}

void setupEncoder() {
  Wire.begin(MT6701_SDA_PIN, MT6701_SCL_PIN);
  Wire.setClock(MT6701_I2C_CLOCK_HZ);
  xTaskCreatePinnedToCore(encoderSampleTask, "mt6701", 4096, nullptr, 2, nullptr, 0);
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_SENSOR_PIN, ADC_11db);

  setupMotorPwm();
  setupEncoder();
  currentZeroSensorVoltage = calibrateCurrentZeroVoltage();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  setupWebServer();

  lastControlAt = millis();

  Serial.println();
  Serial.println("CupExS motor controller ready");
  Serial.print("WiFi AP: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("ACS712 zero voltage: ");
  Serial.println(currentZeroSensorVoltage, 3);
  Serial.print("MT6701 SDA/SCL: ");
  Serial.print(MT6701_SDA_PIN);
  Serial.print("/");
  Serial.println(MT6701_SCL_PIN);
}

void loop() {
  server.handleClient();

  const uint32_t now = millis();
  if (now - lastControlAt >= CONTROL_INTERVAL_MS) {
    updateMotorControl();
  }
}
