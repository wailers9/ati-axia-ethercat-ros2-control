const state = {
  data: null,
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

function update(data) {
  state.data = data;
  const alerts = data.alerts || [];
  text("overall", alerts.length ? "ALERT" : "OK");
  text("wrenchState", data.wrench && data.wrench.updating && data.wrench.valid ? "OK" : "ALERT");
  text("temperature", data.metrics.temperature ?? "-");
  text("voltage", data.metrics.voltage ?? "-");

  const alertBox = document.getElementById("alerts");
  alertBox.classList.toggle("hidden", alerts.length === 0);
  alertBox.innerHTML = alerts.map((item) => `<div>${escapeHtml(item)}</div>`).join("");

  updateBiasResult(data.last_bias_result);
  updateChannelCards(data);

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

  drawCharts(data.history || []);
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

function updateChannelCards(data) {
  const values = data.wrench.values || {};
  document.getElementById("channelCharts").innerHTML = channels.map((channel) => `
    <article class="channel">
      <header>
        <div class="channel-name">${channel.label}</div>
        <div class="channel-value">${fixed(values[channel.name])} ${channel.unit}</div>
      </header>
      <canvas id="chart-${channel.name}" width="420" height="160"></canvas>
    </article>
  `).join("");
}

function drawCharts(history) {
  channels.forEach((channel) => drawSingleChart(channel, history));
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
  ws.onmessage = (event) => update(JSON.parse(event.data));
  ws.onclose = () => setTimeout(connect, 1500);
}

document.getElementById("setBias").addEventListener("click", () => bias("set"));
document.getElementById("clearBias").addEventListener("click", () => bias("clear"));
window.addEventListener("resize", () => state.data && drawCharts(state.data.history || []));
connect();
