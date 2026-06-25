const state = {
  data: null,
  history: [],
  sampleCounterHistory: [],
  wrench: { values: {} },
  channelCardsReady: false,
  chartRefreshHz: 20,
  lastChartDrawMs: 0,
};

const channels = [
  { name: "fx", label: "Force X", unit: "N" },
  { name: "fy", label: "Force Y", unit: "N" },
  { name: "fz", label: "Force Z", unit: "N" },
  { name: "tx", label: "Torque X", unit: "Nm" },
  { name: "ty", label: "Torque Y", unit: "Nm" },
  { name: "tz", label: "Torque Z", unit: "Nm" },
];

function text(id, value) {
  document.getElementById(id).textContent = value;
}

function fixed(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "-";
  return Number(value).toFixed(4);
}

function statusLabel(ok) {
  return ok ? '<span class="ok">OK</span>' : '<span class="bad">ALERT</span>';
}

function applyFull(data) {
  state.data = data;
  state.history = data.history || state.history;
  state.sampleCounterHistory = data.sample_counter_history || state.sampleCounterHistory;
  state.wrench = data.wrench || state.wrench;
  state.chartRefreshHz = Number(data.config?.chart_refresh_rate_hz || 20);
  const alerts = data.alerts || [];
  text("overall", alerts.length ? "ALERT" : "OK");
  text("wrenchState", data.wrench && data.wrench.updating && data.wrench.valid ? "OK" : "ALERT");
  text("ethercatState", data.ethercat && data.ethercat.ok ? "OK" : "ALERT");
  text("temperature", data.metrics.temperature ?? "-");
  text("voltage", data.metrics.voltage ?? "-");

  const alertBox = document.getElementById("alerts");
  alertBox.classList.toggle("hidden", alerts.length === 0);
  alertBox.innerHTML = alerts.map((item) => `<div>${escapeHtml(item)}</div>`).join("");

  updateBiasResult(data.last_bias_result);
  ensureChannelCards();
  updateChannelValues();
  updateEthercat(data.ethercat || {});
  updateSampleCounterValues();

  document.getElementById("checks").innerHTML = Object.entries(data.checks || {}).map(([name, check]) => `
    <tr>
      <td>${escapeHtml(name)}</td>
      <td>${statusLabel(check.ok)}</td>
      <td>${escapeHtml(check.detail)}</td>
    </tr>
  `).join("");

  document.getElementById("interfaces").innerHTML = (data.interfaces || []).map((item) => `
    <tr>
      <td>${escapeHtml(item.name)}</td>
      <td>${item.available ? "yes" : "no"}</td>
      <td>${item.claimed ? "yes" : "no"}</td>
    </tr>
  `).join("");

  document.getElementById("diagnostics").innerHTML = (data.diagnostics || []).map((item) => `
    <tr>
      <td>${escapeHtml(item.name)}</td>
      <td>${item.level}</td>
      <td>${escapeHtml(item.message)}</td>
      <td>${escapeHtml(formatValues(item.values))}</td>
    </tr>
  `).join("");
}

function applyWrench(message) {
  state.wrench = message.wrench || state.wrench;
  state.history = message.history || state.history;
  updateChannelValues();
}

function updateBiasResult(result) {
  const element = document.getElementById("biasResult");
  if (!result) {
    element.textContent = "No bias action yet";
    return;
  }
  const status = result.success ? "OK" : "FAILED";
  element.textContent = `${result.action}: ${status} - ${result.message}`;
}

function ensureChannelCards() {
  if (state.channelCardsReady) return;
  document.getElementById("channelCharts").innerHTML = channels.map((channel) => `
    <article class="channel">
      <header>
        <div class="channel-name">${channel.label}</div>
        <div class="channel-value" id="value-${channel.name}">- ${channel.unit}</div>
      </header>
      <canvas id="chart-${channel.name}" width="420" height="160"></canvas>
    </article>
  `).join("");
  state.channelCardsReady = true;
}

function updateChannelValues() {
  ensureChannelCards();
  const values = state.wrench.values || {};
  channels.forEach((channel) => {
    const element = document.getElementById(`value-${channel.name}`);
    if (element) element.textContent = `${fixed(values[channel.name])} ${channel.unit}`;
  });
}

function updateEthercat(ethercat) {
  document.getElementById("ethercatChecks").innerHTML = (ethercat.items || []).map((item) => `
    <tr>
      <td>${escapeHtml(item.name)}</td>
      <td>${statusLabel(item.ok)}</td>
      <td>${escapeHtml(item.detail)}</td>
    </tr>
  `).join("");

  document.getElementById("ethercatValues").innerHTML = Object.entries(ethercat.values || {}).map(([key, value]) => `
    <tr>
      <td>${escapeHtml(key)}</td>
      <td>${escapeHtml(value ?? "-")}</td>
    </tr>
  `).join("");
}

function drawCharts(history) {
  channels.forEach((channel) => drawSingleChart(channel, history));
  drawCounterChart(
    "chart-counter-repeats",
    state.sampleCounterHistory,
    "actual_repeats_per_sec",
    "repeats/s"
  );
  drawCounterChart(
    "chart-counter-jumps",
    state.sampleCounterHistory,
    "actual_jump_events_per_sec",
    "jumps/s"
  );
}

function updateSampleCounterValues() {
  const history = state.sampleCounterHistory || [];
  const latest = history.length ? history[history.length - 1] : {};
  text("counter-repeat-value", `${fixed(latest.actual_repeats_per_sec)} /s`);
  text("counter-jump-value", `${fixed(latest.actual_jump_events_per_sec)} /s`);
  text(
    "counter-repeat-reference",
    `Expected repeats: ${fixed(latest.expected_repeats_per_sec)} /s`
  );
  text(
    "counter-jump-reference",
    `Expected jumps: ${fixed(latest.expected_jump_events_per_sec)} /s; expected/actual skipped: ${fixed(latest.expected_skipped_samples_per_sec)} / ${fixed(latest.actual_skipped_samples_per_sec)} /s`
  );
}

function drawCounterChart(canvasId, history, key, label) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  const width = rect.width;
  const height = rect.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#ddd";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(36, 16);
  ctx.lineTo(36, height - 24);
  ctx.lineTo(width - 10, height - 24);
  ctx.stroke();

  const values = history.map((sample) => Number(sample[key])).filter(Number.isFinite);
  if (!values.length) return;
  const max = Math.max(1, ...values);
  ctx.strokeStyle = "#111";
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  history.forEach((sample, index) => {
    const value = Number(sample[key]);
    if (!Number.isFinite(value)) return;
    const x = 36 + (index / Math.max(1, history.length - 1)) * (width - 48);
    const y = 16 + ((max - value) / max) * (height - 44);
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();
  ctx.fillStyle = "#111";
  ctx.font = "12px sans-serif";
  ctx.fillText(max.toFixed(1), 4, 20);
  ctx.fillText("0", 18, height - 26);
  ctx.fillText(label, 44, height - 7);
}

function drawSingleChart(channel, history) {
  const canvas = document.getElementById(`chart-${channel.name}`);
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);

  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  const width = rect.width;
  const height = rect.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#ddd";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(36, 16);
  ctx.lineTo(36, height - 24);
  ctx.lineTo(width - 10, height - 24);
  ctx.stroke();

  if (history.length < 2) return;

  const all = history.map((sample) => Number(sample[channel.name])).filter(Number.isFinite);
  if (!all.length) return;

  let min = Math.min(...all);
  let max = Math.max(...all);
  if (min === max) {
    min -= 1;
    max += 1;
  }

  ctx.strokeStyle = "#111";
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  history.forEach((sample, sampleIndex) => {
    const value = Number(sample[channel.name]);
    if (!Number.isFinite(value)) return;
    const x = 36 + (sampleIndex / (history.length - 1)) * (width - 48);
    const y = 16 + ((max - value) / (max - min)) * (height - 44);
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  });
  ctx.stroke();

  ctx.fillStyle = "#111";
  ctx.font = "12px sans-serif";
  ctx.fillText(max.toFixed(2), 4, 20);
  ctx.fillText(min.toFixed(2), 4, height - 26);
  ctx.fillText(channel.name, 44, height - 7);
}

function formatValues(values) {
  return Object.entries(values || {}).map(([key, value]) => `${key}: ${value}`).join("; ");
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (char) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#039;",
  }[char]));
}

async function bias(action) {
  const label = action === "set" ? "set_bias" : "clear_bias";
  if (!window.confirm(`Confirm ${label}?`)) return;
  const response = await fetch(`/api/bias/${action}`, { method: "POST" });
  const result = await response.json();
  if (!result.success) window.alert(result.message);
}

function connect() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${scheme}://${window.location.host}/ws`);
  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (!message.type) {
      applyFull(message);
    } else if (message.type === "full") {
      applyFull(message.state);
    } else if (message.type === "wrench") {
      applyWrench(message);
    }
  };
  ws.onclose = () => setTimeout(connect, 1500);
}

function chartLoop(timestampMs) {
  const minPeriodMs = 1000 / Math.max(1, state.chartRefreshHz || 20);
  if (timestampMs - state.lastChartDrawMs >= minPeriodMs) {
    state.lastChartDrawMs = timestampMs;
    drawCharts(state.history || []);
  }
  window.requestAnimationFrame(chartLoop);
}

document.getElementById("setBias").addEventListener("click", () => bias("set"));
document.getElementById("clearBias").addEventListener("click", () => bias("clear"));
window.addEventListener("resize", () => drawCharts(state.history || []));
connect();
window.requestAnimationFrame(chartLoop);
