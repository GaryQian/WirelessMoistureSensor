#pragma once

static const char WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Soil moisture</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7;
  --surface-1: #fcfcfb;
  --text-primary: #0b0b0b;
  --text-secondary: #52514e;
  --text-muted: #898781;
  --grid: #e1e0d9;
  --axis: #c3c2b7;
  --border: rgba(11,11,11,0.10);
  --series-1: #2a78d6;
  --series-2: #eb6834;
}
@media (prefers-color-scheme: dark) {
  :root {
    color-scheme: dark;
    --page: #0d0d0d;
    --surface-1: #1a1a19;
    --text-primary: #ffffff;
    --text-secondary: #c3c2b7;
    --text-muted: #898781;
    --grid: #2c2c2a;
    --axis: #383835;
    --border: rgba(255,255,255,0.10);
    --series-1: #3987e5;
    --series-2: #d95926;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--text-primary);
  font: 15px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 960px; margin: 0 auto; padding: 24px 16px 40px; }
h1 { font-size: 20px; font-weight: 600; margin: 0; }
.sub { color: var(--text-secondary); font-size: 13px; margin: 2px 0 20px; }
.card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
.hero { display: flex; flex-wrap: wrap; gap: 24px; align-items: flex-end; margin-bottom: 12px; }
.hero-value { font-size: 56px; font-weight: 600; line-height: 1; }
.hero-label { color: var(--text-secondary); font-size: 13px; margin-top: 6px; }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 12px; margin-bottom: 20px; }
.tile .label { color: var(--text-secondary); font-size: 13px; }
.tile .value { font-size: 22px; font-weight: 600; margin-top: 2px; }
.tile .note { color: var(--text-muted); font-size: 12px; margin-top: 2px; }
.filters { display: flex; gap: 4px; margin-bottom: 12px; }
.filters button { font: inherit; font-size: 13px; color: var(--text-secondary); background: transparent;
  border: 1px solid var(--border); border-radius: 8px; padding: 4px 12px; cursor: pointer; }
.filters button[aria-pressed="true"] { color: var(--text-primary); font-weight: 600; background: var(--surface-1); }
figure { margin: 0; }
figcaption { display: flex; flex-wrap: wrap; justify-content: space-between; gap: 8px; margin-bottom: 8px; }
.title { font-weight: 600; }
.legend { display: flex; gap: 16px; color: var(--text-secondary); font-size: 13px; align-items: center; }
.key-line { display: inline-block; width: 16px; height: 2px; border-radius: 1px; background: var(--series-1); vertical-align: middle; margin-right: 6px; }
.key-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: var(--series-2); vertical-align: middle; margin-right: 6px; }
.key-ring { display: inline-block; width: 8px; height: 8px; border-radius: 50%; border: 2px solid var(--series-2); vertical-align: middle; margin-right: 6px; }
#chart-wrap { position: relative; transition: opacity .2s; }
#chart-wrap.loading { opacity: .6; }
svg { display: block; width: 100%; height: auto; overflow: visible; }
svg:focus { outline: none; }
svg:focus-visible { outline: 2px solid var(--series-1); outline-offset: 4px; border-radius: 4px; }
.tick { fill: var(--text-muted); font-size: 11px; font-variant-numeric: tabular-nums; }
.end-label { fill: var(--text-primary); font-size: 12px; font-weight: 600; }
.ref-label { fill: var(--text-secondary); font-size: 11px; }
#tip { position: absolute; pointer-events: none; background: var(--surface-1); border: 1px solid var(--border);
  border-radius: 8px; padding: 6px 10px; font-size: 13px; box-shadow: 0 2px 8px rgba(0,0,0,.12); display: none; white-space: nowrap; }
#tip .v { font-weight: 600; color: var(--text-primary); }
#tip .k { color: var(--text-secondary); }
#tip .row { display: flex; align-items: center; gap: 6px; }
.empty { color: var(--text-muted); padding: 48px 0; text-align: center; }
details { margin-top: 12px; color: var(--text-secondary); font-size: 13px; }
summary { cursor: pointer; }
table { border-collapse: collapse; margin-top: 8px; font-variant-numeric: tabular-nums; }
th, td { text-align: left; padding: 3px 16px 3px 0; border-bottom: 1px solid var(--grid); }
th { color: var(--text-muted); font-weight: 500; }
footer { margin-top: 24px; color: var(--text-muted); font-size: 12px; }
footer a { color: var(--text-secondary); }
</style>
</head>
<body>
<main>
  <h1>Soil moisture</h1>
  <p class="sub" id="updated">Loading&hellip;</p>

  <section class="card hero" aria-live="polite">
    <div>
      <div class="hero-value" id="avg">&ndash;</div>
      <div class="hero-label" id="avg-label">Average of the fitted sensors</div>
    </div>
  </section>

  <section class="tiles" id="tiles"></section>

  <div class="filters" role="group" aria-label="Time range">
    <button type="button" data-days="2" aria-pressed="false">48 hours</button>
    <button type="button" data-days="7" aria-pressed="true">7 days</button>
    <button type="button" data-days="14" aria-pressed="false">14 days</button>
  </div>

  <figure class="card">
    <figcaption>
      <span class="title">Average moisture, hourly</span>
      <span class="legend">
        <span><span class="key-line"></span>Moisture</span>
        <span><span class="key-dot"></span>Watered</span>
        <span id="legend-failed" hidden><span class="key-ring"></span>Attempt stopped</span>
      </span>
    </figcaption>
    <div id="chart-wrap">
      <svg id="chart" role="img" tabindex="0" aria-label="Average soil moisture over time"></svg>
      <div id="tip"></div>
    </div>
    <details>
      <summary>Table view</summary>
      <table id="table"></table>
    </details>
  </figure>

  <footer>
    Data as JSON: <a href="/api/current">/api/current</a> &middot; <a href="/api/history">/api/history</a> (<code>?days=1-14</code>)
  </footer>
</main>
<script>
"use strict";
const NS = "http://www.w3.org/2000/svg";
const GAP_S = 2 * 3600;
let days = 7, current = null, history = null, points = [], focusIndex = -1;

const fmtTime = t => new Date(t * 1000).toLocaleString([], {weekday: "short", hour: "2-digit", minute: "2-digit"});
const fmtDay = t => new Date(t * 1000).toLocaleDateString([], {month: "short", day: "numeric"});
const fmtFull = t => new Date(t * 1000).toLocaleString([], {weekday: "short", month: "short", day: "numeric", hour: "2-digit", minute: "2-digit"});
const toS = iso => iso ? Date.parse(iso) / 1000 : null;

function ago(s) {
  if (s < 90) return "just now";
  if (s < 5400) return Math.round(s / 60) + " min ago";
  if (s < 172800) return Math.round(s / 3600) + " h ago";
  return Math.round(s / 86400) + " days ago";
}
function inS(s) {
  if (s <= 0) return "now";
  if (s < 5400) return "in " + Math.round(s / 60) + " min";
  if (s < 172800) return "in " + Math.round(s / 3600) + " h";
  return "in " + Math.round(s / 86400) + " days";
}
function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function text(parent, tag, cls, value) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  e.textContent = value;
  parent.appendChild(e);
  return e;
}

function tile(label, value, note) {
  const t = document.createElement("div");
  t.className = "card tile";
  text(t, "div", "label", label);
  text(t, "div", "value", value);
  if (note) text(t, "div", "note", note);
  return t;
}

function renderCurrent() {
  const c = current;
  const now = toS(c.time);
  document.getElementById("updated").textContent =
    (c.clockSet ? "Updated " + new Date().toLocaleTimeString([], {hour: "2-digit", minute: "2-digit", second: "2-digit"}) : "Receiver clock not set yet") +
    " · waters below " + c.thresholdPct + "%";
  const r = c.reading;
  document.getElementById("avg").textContent = r && r.avg !== null ? r.avg + "%" : "–";
  document.getElementById("avg-label").textContent = r
    ? "Average of the fitted sensors over " + r.windowS + " s, measured " + (r.time && now ? ago(now - toS(r.time)) : "recently")
    : "No reading since the receiver started";

  const tiles = document.getElementById("tiles");
  tiles.replaceChildren();
  if (r) for (const s of r.sensors) {
    if (!s.fitted) continue;
    tiles.appendChild(tile("Sensor " + s.pin, s.status === "ok" ? s.pct + "%" : "Fault", "raw " + s.raw));
  }
  const w = c.watering;
  let state = "Off", stateNote = "relay reads wet";
  if (w.active) { state = "Watering"; stateNote = Math.ceil(w.remainingS / 60) + " min left"; }
  else if (w.queued !== "none") { state = "Queued"; stateNote = w.queued + (w.nextSlot ? ", " + fmtTime(toS(w.nextSlot)) : ""); }
  tiles.appendChild(tile("Watering", state, stateNote));
  tiles.appendChild(tile("Last watered", w.lastWatered && now ? ago(now - toS(w.lastWatered)) : "Unknown",
    w.lastWatered ? fmtFull(toS(w.lastWatered)) : ""));
  tiles.appendChild(tile("Next allowed", w.dryAllowedInS > 0 ? inS(w.dryAllowedInS) : "When dry",
    "forced " + inS(w.forcedInS)));
}

function buildPoints() {
  const pts = history.hourly.map(h => ({t: toS(h.t) + 1800, v: h.avg, label: fmtFull(toS(h.t))}));
  const r = current && current.reading;
  if (r && r.avg !== null && r.time) {
    const t = toS(r.time);
    if (!pts.length || t > pts[pts.length - 1].t) pts.push({t, v: r.avg, label: fmtFull(t) + " (latest)"});
  }
  return pts;
}

function renderChart() {
  const svg = document.getElementById("chart");
  const W = Math.max(320, svg.parentNode.clientWidth), H = 300;
  const m = {l: 40, r: 44, t: 12, b: 28};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);
  svg.replaceChildren();
  const now = current ? toS(current.time) || Date.now() / 1000 : Date.now() / 1000;
  const t0 = now - days * 86400;
  const x = t => m.l + (t - t0) / (now - t0) * pw;
  const y = v => m.t + (1 - v / 100) * ph;

  for (const v of [0, 25, 50, 75, 100]) {
    el("line", {x1: m.l, x2: m.l + pw, y1: y(v), y2: y(v), stroke: v === 0 ? "var(--axis)" : "var(--grid)", "stroke-width": 1}, svg);
    el("text", {x: m.l - 8, y: y(v) + 4, "text-anchor": "end", class: "tick"}, svg).textContent = v + "%";
  }
  const d = new Date(t0 * 1000);
  if (days <= 2) {
    d.setMinutes(0, 0, 0);
    d.setHours(Math.ceil(d.getHours() / 6) * 6);
  } else {
    d.setHours(24, 0, 0, 0);
  }
  for (; d.getTime() / 1000 <= now; days <= 2 ? d.setHours(d.getHours() + 6) : d.setDate(d.getDate() + (days <= 7 ? 1 : 2))) {
    const t = d.getTime() / 1000;
    const label = days > 2 || d.getHours() === 0 ? fmtDay(t) : d.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
    el("text", {x: x(t), y: H - 8, "text-anchor": "middle", class: "tick"}, svg).textContent = label;
  }

  const thr = current ? current.thresholdPct : null;
  if (thr !== null) {
    el("line", {x1: m.l, x2: m.l + pw, y1: y(thr), y2: y(thr), stroke: "var(--text-secondary)", "stroke-width": 1}, svg);
    el("text", {x: m.l + 6, y: y(thr) - 6, class: "ref-label"}, svg).textContent = "Waters below " + thr + "%";
  }

  points = buildPoints().filter(p => p.t >= t0);
  const waterings = history.waterings.map(w => ({t: toS(w.t), completed: w.completed, reason: w.reason})).filter(w => w.t >= t0);
  document.getElementById("legend-failed").hidden = !waterings.some(w => !w.completed);

  if (!points.length) {
    const e = el("text", {x: m.l + pw / 2, y: m.t + ph / 2, "text-anchor": "middle", class: "tick"}, svg);
    e.textContent = "No readings in this range yet";
  }
  let seg = [];
  const segments = [];
  points.forEach((p, i) => {
    if (i && p.t - points[i - 1].t > GAP_S) { segments.push(seg); seg = []; }
    seg.push(p);
  });
  if (seg.length) segments.push(seg);
  for (const s of segments) {
    const d = s.map((p, i) => (i ? "L" : "M") + x(p.t).toFixed(1) + " " + y(p.v).toFixed(1)).join(" ");
    if (s.length > 1) {
      el("path", {d: d + " L" + x(s[s.length - 1].t).toFixed(1) + " " + y(0) + " L" + x(s[0].t).toFixed(1) + " " + y(0) + " Z",
        fill: "var(--series-1)", "fill-opacity": 0.1, stroke: "none"}, svg);
      el("path", {d, fill: "none", stroke: "var(--series-1)", "stroke-width": 2, "stroke-linejoin": "round", "stroke-linecap": "round"}, svg);
    } else {
      el("circle", {cx: x(s[0].t), cy: y(s[0].v), r: 2, fill: "var(--series-1)"}, svg);
    }
  }
  if (points.length) {
    const p = points[points.length - 1];
    el("circle", {cx: x(p.t), cy: y(p.v), r: 4, fill: "var(--series-1)", stroke: "var(--surface-1)", "stroke-width": 2}, svg);
    el("text", {x: x(p.t) + 8, y: y(p.v) + 4, class: "end-label"}, svg).textContent = p.v + "%";
  }
  for (const w of waterings) {
    el("circle", {cx: x(w.t), cy: y(0), r: 4, fill: w.completed ? "var(--series-2)" : "var(--surface-1)",
      stroke: w.completed ? "var(--surface-1)" : "var(--series-2)", "stroke-width": 2}, svg);
  }

  const cross = el("line", {y1: m.t, y2: m.t + ph, stroke: "var(--axis)", "stroke-width": 1, visibility: "hidden"}, svg);
  const dot = el("circle", {r: 4, fill: "var(--series-1)", stroke: "var(--surface-1)", "stroke-width": 2, visibility: "hidden"}, svg);
  const hit = el("rect", {x: m.l, y: m.t, width: pw, height: ph + 12, fill: "transparent"}, svg);
  const tip = document.getElementById("tip");

  function show(i) {
    if (i < 0 || i >= points.length) return hide();
    focusIndex = i;
    const p = points[i];
    cross.setAttribute("x1", x(p.t)); cross.setAttribute("x2", x(p.t)); cross.setAttribute("visibility", "visible");
    dot.setAttribute("cx", x(p.t)); dot.setAttribute("cy", y(p.v)); dot.setAttribute("visibility", "visible");
    tip.replaceChildren();
    const row = document.createElement("div");
    row.className = "row";
    const key = document.createElement("span"); key.className = "key-line"; row.appendChild(key);
    text(row, "span", "v", p.v + "%");
    text(row, "span", "k", "moisture");
    tip.appendChild(row);
    for (const w of waterings) {
      if (Math.abs(w.t - p.t) > 1800) continue;
      const wr = document.createElement("div");
      wr.className = "row";
      const k = document.createElement("span"); k.className = w.completed ? "key-dot" : "key-ring"; wr.appendChild(k);
      text(wr, "span", "v", w.completed ? "Watered" : "Attempt stopped");
      text(wr, "span", "k", w.reason);
      tip.appendChild(wr);
    }
    text(tip, "div", "k", p.label);
    tip.style.display = "block";
    const scale = svg.getBoundingClientRect().width / W;
    let left = x(p.t) * scale + 12;
    if (left + tip.offsetWidth > svg.parentNode.clientWidth) left = x(p.t) * scale - tip.offsetWidth - 12;
    tip.style.left = left + "px";
    tip.style.top = Math.max(0, y(p.v) * scale - tip.offsetHeight - 8) + "px";
  }
  function hide() {
    cross.setAttribute("visibility", "hidden"); dot.setAttribute("visibility", "hidden");
    tip.style.display = "none"; focusIndex = -1;
  }
  function nearest(px) {
    let best = -1, bd = Infinity;
    points.forEach((p, i) => { const d = Math.abs(x(p.t) - px); if (d < bd) { bd = d; best = i; } });
    return best;
  }
  hit.addEventListener("pointermove", e => {
    const rect = svg.getBoundingClientRect();
    show(nearest((e.clientX - rect.left) * W / rect.width));
  });
  hit.addEventListener("pointerleave", hide);
  svg.onkeydown = e => {
    if (!points.length) return;
    if (e.key === "ArrowLeft") { show(focusIndex <= 0 ? 0 : focusIndex - 1); e.preventDefault(); }
    else if (e.key === "ArrowRight") { show(focusIndex < 0 ? points.length - 1 : Math.min(points.length - 1, focusIndex + 1)); e.preventDefault(); }
    else if (e.key === "Escape") hide();
  };
  svg.onblur = hide;
  svg.setAttribute("aria-label", "Average soil moisture over the last " + days + " days" +
    (points.length ? ", latest " + points[points.length - 1].v + "%" : ", no readings"));
}

function renderTable() {
  const table = document.getElementById("table");
  table.replaceChildren();
  const head = table.createTHead().insertRow();
  for (const h of ["Hour", "Average moisture", "Watering"]) text(head, "th", "", h);
  const body = table.createTBody();
  const waterings = history.waterings.map(w => ({t: toS(w.t), completed: w.completed, reason: w.reason}));
  for (const h of history.hourly.slice().reverse()) {
    const t = toS(h.t);
    const row = body.insertRow();
    text(row, "td", "", fmtFull(t));
    text(row, "td", "", h.avg + "%");
    const ws = waterings.filter(w => w.t >= t && w.t < t + 3600);
    text(row, "td", "", ws.map(w => (w.completed ? "watered" : "stopped") + " (" + w.reason + ")").join(", "));
  }
}

async function loadCurrent() {
  try {
    const res = await fetch("/api/current", {cache: "no-store"});
    current = await res.json();
    renderCurrent();
    if (history) renderChart();
  } catch (e) {
    document.getElementById("updated").textContent = "Receiver not responding; retrying…";
  }
}
async function loadHistory() {
  const wrap = document.getElementById("chart-wrap");
  wrap.classList.add("loading");
  try {
    const res = await fetch("/api/history?days=" + days, {cache: "no-store"});
    history = await res.json();
    renderChart();
    renderTable();
  } catch (e) {}
  wrap.classList.remove("loading");
}

document.querySelectorAll(".filters button").forEach(b => b.addEventListener("click", () => {
  days = +b.dataset.days;
  document.querySelectorAll(".filters button").forEach(o => o.setAttribute("aria-pressed", o === b));
  loadHistory();
}));
new ResizeObserver(() => { if (history) renderChart(); }).observe(document.getElementById("chart-wrap"));

loadCurrent().then(loadHistory);
setInterval(loadCurrent, 15000);
setInterval(loadHistory, 300000);
</script>
</body>
</html>
)HTML";
