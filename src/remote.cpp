#include "remote.h"

#include <BluetoothSerial.h>
#include <WebServer.h>
#include <WiFi.h>

#include "config.h"
#include "pid.h"

namespace {

BluetoothSerial bluetooth;
WebServer server(80);

// Sans nouvelle commande pendant ce délai, le gyropode se remet à l'arrêt.
constexpr uint32_t COMMAND_TIMEOUT_MS = 600;

const char kIndexHtml[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Gyropode</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 0; padding: 16px; background: #101418; color: #eee; }
  h1 { font-size: 20px; }
  .pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 320px; }
  button { padding: 22px 0; font-size: 20px; border: 0; border-radius: 10px; background: #2b6cb0; color: #fff; }
  button:active { background: #1a4a7a; }
  button.stop { background: #b02b2b; }
  .gains { margin-top: 24px; max-width: 320px; }
  .gains label { display: block; margin: 10px 0 2px; font-size: 14px; }
  .gains input { width: 100%; }
  #telemetry { margin-top: 20px; font-family: ui-monospace, monospace; font-size: 14px; }
</style>
</head>
<body>
<h1>Gyropode</h1>
<div class="pad">
  <span></span>
  <button id="fwd">&#9650;</button>
  <span></span>
  <button id="left">&#9664;</button>
  <button id="stop" class="stop">STOP</button>
  <button id="right">&#9654;</button>
  <span></span>
  <button id="back">&#9660;</button>
  <span></span>
</div>

<div class="gains">
  <label>Kp <span id="kpVal"></span></label>
  <input type="range" id="kp" min="0" max="60" step="0.5">
  <label>Ki <span id="kiVal"></span></label>
  <input type="range" id="ki" min="0" max="300" step="1">
  <label>Kd <span id="kdVal"></span></label>
  <input type="range" id="kd" min="0" max="5" step="0.05">
</div>

<div id="telemetry">angle : -- deg</div>

<script>
let throttle = 0, steering = 0, enabled = true;

function send() {
  fetch(`/api/control?throttle=${throttle}&steering=${steering}&enabled=${enabled ? 1 : 0}`);
}

function hold(id, apply) {
  const el = document.getElementById(id);
  const start = (e) => { e.preventDefault(); apply(); send(); };
  const end = (e) => { e.preventDefault(); throttle = 0; steering = 0; send(); };
  el.addEventListener('pointerdown', start);
  el.addEventListener('pointerup', end);
  el.addEventListener('pointerleave', end);
}

hold('fwd', () => { throttle = 1; steering = 0; });
hold('back', () => { throttle = -1; steering = 0; });
hold('left', () => { steering = -1; });
hold('right', () => { steering = 1; });

document.getElementById('stop').addEventListener('click', () => {
  enabled = !enabled;
  throttle = 0; steering = 0;
  send();
});

for (const name of ['kp', 'ki', 'kd']) {
  const slider = document.getElementById(name);
  slider.addEventListener('input', () => {
    document.getElementById(name + 'Val').textContent = slider.value;
    const kp = document.getElementById('kp').value;
    const ki = document.getElementById('ki').value;
    const kd = document.getElementById('kd').value;
    fetch(`/api/pid?kp=${kp}&ki=${ki}&kd=${kd}`);
  });
}

async function refresh() {
  try {
    const data = await (await fetch('/api/telemetry')).json();
    document.getElementById('telemetry').textContent =
      `angle : ${data.pitch.toFixed(1)} deg | sortie : ${data.output.toFixed(2)}` +
      (data.fallen ? ' | CHUTE' : '');
    for (const name of ['kp', 'ki', 'kd']) {
      const slider = document.getElementById(name);
      if (document.activeElement !== slider) {
        slider.value = data[name];
        document.getElementById(name + 'Val').textContent = data[name];
      }
    }
  } catch (e) { /* l'ESP32 est occupe, on reessaie au prochain tour */ }
}

setInterval(refresh, 300);
setInterval(send, 300);  // maintient la liaison vivante cote ESP32
refresh();
</script>
</body>
</html>)HTML";

}  // namespace

void Remote::begin() {
  bluetooth.begin(BLUETOOTH_DEVICE_NAME);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  registerWebRoutes();
  server.begin();
}

void Remote::poll() {
  handleBluetooth();
  server.handleClient();

  if (millis() - last_command_ms_ > COMMAND_TIMEOUT_MS) {
    command_.throttle = 0.0f;
    command_.steering = 0.0f;
  }
}

DriveCommand Remote::command() { return command_; }

void Remote::handleBluetooth() {
  while (bluetooth.available() > 0) {
    applyCharCommand(static_cast<char>(bluetooth.read()));
  }
}

void Remote::applyCharCommand(char c) {
  switch (c) {
    case 'z':
    case 'F':
      command_.throttle = 1.0f;
      break;
    case 's':
    case 'B':
      command_.throttle = -1.0f;
      break;
    case 'q':
    case 'L':
      command_.steering = -1.0f;
      break;
    case 'd':
    case 'R':
      command_.steering = 1.0f;
      break;
    case ' ':
    case 'S':
      command_.throttle = 0.0f;
      command_.steering = 0.0f;
      break;
    case 'x':
      command_.enabled = false;
      break;
    case 'e':
      command_.enabled = true;
      break;
    default:
      return;
  }
  last_command_ms_ = millis();
}

void Remote::registerWebRoutes() {
  server.on("/", HTTP_GET, [this]() { server.send_P(200, "text/html", kIndexHtml); });

  server.on("/api/control", HTTP_GET, [this]() {
    if (server.hasArg("throttle")) {
      command_.throttle = constrain(server.arg("throttle").toFloat(), -1.0f, 1.0f);
    }
    if (server.hasArg("steering")) {
      command_.steering = constrain(server.arg("steering").toFloat(), -1.0f, 1.0f);
    }
    if (server.hasArg("enabled")) {
      command_.enabled = server.arg("enabled").toInt() != 0;
    }
    last_command_ms_ = millis();
    server.send(200, "text/plain", "ok");
  });

  server.on("/api/pid", HTTP_GET, [this]() {
    const float kp = server.hasArg("kp") ? server.arg("kp").toFloat() : balance_pid_.kp();
    const float ki = server.hasArg("ki") ? server.arg("ki").toFloat() : balance_pid_.ki();
    const float kd = server.hasArg("kd") ? server.arg("kd").toFloat() : balance_pid_.kd();
    balance_pid_.setGains(kp, ki, kd);
    server.send(200, "text/plain", "ok");
  });

  server.on("/api/telemetry", HTTP_GET, [this]() {
    char body[192];
    snprintf(body, sizeof(body),
             "{\"pitch\":%.2f,\"output\":%.3f,\"fallen\":%s,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f}",
             telemetry_.pitch_deg, telemetry_.output, telemetry_.fallen ? "true" : "false",
             balance_pid_.kp(), balance_pid_.ki(), balance_pid_.kd());
    server.send(200, "application/json", body);
  });
}
