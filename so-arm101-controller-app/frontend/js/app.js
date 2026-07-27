/**
 * SO-ARM101 Web Controller — main entry.
 *
 * Layout: sidebar + pages (control, status, teach, record).
 * Live state: fed by a WebSocket at /api/ws. Commands go through REST.
 */

// Flip the boot flag *immediately* so the diagnostic overlay in index.html
// knows this module loaded. Anything that silently kills ES-module loading
// (import-map typo, missing vendored file, syntax error) will keep this
// false and the overlay will pop up.
window.__soarmBooted = true;

import {
  api, openWebSocket,
  PermissionDeniedError, getClientId, getClientName, setClientName,
} from "./api.js";
import { DigitalTwin } from "./twin.js";
import { loadKinematics, unwrapPosition, windowFraction } from "./kinematics.js";


// ── Elements ───────────────────────────────────────────────────────────────
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];

const JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"];
const JOINT_IDS = [1, 2, 3, 4, 5, 6];

// Per-team palette. team1..team7 get explicit, distinct hues so 7 tabs
// open side-by-side are unmistakable. Anything else falls through to a
// stable hash → hue so custom team names (e.g. "lab-A") still get color.
const TEAM_PALETTE = {
  team1: { color: "#58b6ff", color2: "#7c8cff" },   // blue
  team2: { color: "#3fd37a", color2: "#5cffa7" },   // green
  team3: { color: "#ffb447", color2: "#ffcf6b" },   // amber
  team4: { color: "#c074ff", color2: "#a058ff" },   // purple
  team5: { color: "#ff5a8a", color2: "#ff8aa8" },   // pink
  team6: { color: "#3fdde0", color2: "#62f1f4" },   // cyan
  team7: { color: "#ffe066", color2: "#ffd23f" },   // yellow
};

function teamColors(team) {
  if (TEAM_PALETTE[team]) return TEAM_PALETTE[team];
  // Stable hash → hue for custom team names.
  let h = 0;
  for (let i = 0; i < team.length; i++) h = ((h << 5) - h + team.charCodeAt(i)) | 0;
  const hue = Math.abs(h) % 360;
  return { color: `hsl(${hue},70%,62%)`, color2: `hsl(${(hue + 30) % 360},70%,62%)` };
}

function applyTeamLabel(team) {
  const badge = document.getElementById("team-badge");
  if (!team) {
    badge.classList.add("hidden");
    return;
  }
  const { color, color2 } = teamColors(team);
  document.documentElement.style.setProperty("--team-color", color);
  document.documentElement.style.setProperty("--team-color-2", color2);
  // Soft tint derived from the same hue, used for the badge background and
  // the brand-mark glow. Translucent so it sits over the dark theme cleanly.
  document.documentElement.style.setProperty("--team-tint",
    color.startsWith("#") ? `${color}26` : color.replace(")", " / 0.15)"));
  document.body.classList.add("has-team");
  badge.textContent = team.toUpperCase();
  badge.classList.remove("hidden");
  document.title = `${team.toUpperCase()} — SO-ARM101`;
  const sub = document.getElementById("brand-sub");
  if (sub) sub.textContent = team.toUpperCase();
}

// ── State ─────────────────────────────────────────────────────────────────
const state = {
  connected: false,
  torque: {},      // { id: bool }
  positions: {},   // { id: int }
  speed: 200,
  poses: {},
  recordings: [],
  recording: false,
  playing: false,
  recFrames: 0,
  selectedPose: null,
  twin: null,
  kin: null,
  limits: {},         // { joint_id: {min, max, kind} } from /api/info
  sendThrottle: null,
  pending: {},        // joint IDs queued for the next throttled send
  userDriving: {},    // { joint_id: timeoutId } — non-null means user is
                      // actively moving this joint via slider/button, so
                      // incoming WebSocket positions must NOT overwrite
                      // the slider or twin target for that joint.
  myClientId: null,
  isAdmin: false,
  permitted: true,    // mirrors my own row in the server-side registry
  clients: [],        // for the admin team-members panel
};

/**
 * Mark a joint as actively driven by the user for ~300 ms. Any subsequent
 * WebSocket update for that joint is ignored for its slider/twin target
 * (the real-robot telemetry keeps flowing into the Status page etc).
 * Without this the WS feedback drags the slider back to the lagging
 * physical position every 125 ms, fighting the user's input.
 */
const USER_DRIVE_HOLD_MS = 300;
function markUserDriving(id) {
  if (state.userDriving[id]) clearTimeout(state.userDriving[id]);
  state.userDriving[id] = setTimeout(() => {
    delete state.userDriving[id];
  }, USER_DRIVE_HOLD_MS);
}

// ── Utilities ─────────────────────────────────────────────────────────────
function toast(msg, kind = "ok") {
  const el = $("#toast");
  el.textContent = msg;
  el.className = `toast show ${kind}`;
  clearTimeout(toast._t);
  toast._t = setTimeout(() => (el.className = "toast"), 2400);
}

function posToDeg(pos) {
  if (pos === null || pos === undefined) return null;
  return Math.round(((pos - 2048) / 4095) * 360 * 10) / 10;
}

/**
 * Per-joint display. Rotation joints show the URDF joint angle; the gripper
 * shows 0 % (fully closed) .. 100 % (fully open). Unit is chosen by
 * `limit.kind` returned from /api/info.
 *
 * Both readouts run off the same 0..1 position within the joint's calibrated
 * window that the twin uses, so the number next to the slider and the pose on
 * screen can't disagree.
 */
function posToLabel(id, pos) {
  if (pos === null || pos === undefined) return "—";
  const lim = state.limits?.[id];
  const t = windowFraction(pos, lim);
  if (lim && lim.kind === "percent") {
    if (t === null) return "—";
    return `${Math.round(t * 100)}%`;
  }
  if (t !== null && lim.rad_min !== undefined && lim.rad_max !== undefined) {
    let rad = lim.rad_min + t * (lim.rad_max - lim.rad_min);
    if (lim.invert) rad = -rad;   // same last step as twin.js, or they disagree
    return `${Math.round((rad * 180 / Math.PI) * 10) / 10}°`;
  }
  // Uncalibrated: no window to place this count in, so fall back to the
  // servo's own centre-2048 reading.
  return `${posToDeg(pos)}°`;
}

function guessDefaultPort() {
  // Best-effort: Windows ≈ COM3, macOS ≈ /dev/tty.usbmodem*, Linux ≈ /dev/ttyACM0
  const ua = navigator.userAgent || "";
  if (/Win/.test(ua)) return "COM3";
  if (/Mac/.test(ua)) return "/dev/tty.usbmodem0";
  return "/dev/ttyACM0";
}

/**
 * Bind continuous hold-to-move motion to a push button.
 *
 * While held, applies a smooth velocity (units/sec from getVelocity()) to
 * a floating-point accumulator every animation frame. Whole-unit deltas
 * are forwarded to onStep(). Result is ~60 Hz updates with no perceptible
 * ticking, independent of frame rate.
 *
 * A single tap still works because the first frame fires right away.
 */
function bindHoldMotion(button, direction, onStep, getVelocity) {
  let active = false;
  let rafId = null;
  let lastT = 0;
  let acc = 0;

  const tick = (now) => {
    if (!active) return;
    const dt = Math.min((now - lastT) / 1000, 0.05);
    lastT = now;
    acc += dt * getVelocity() * direction;
    const delta = acc > 0 ? Math.floor(acc) : Math.ceil(acc);
    if (delta !== 0) {
      acc -= delta;
      onStep(delta);
    }
    rafId = requestAnimationFrame(tick);
  };

  const stop = () => {
    active = false;
    if (rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
  };

  const start = (ev) => {
    if (ev.button !== undefined && ev.button !== 0) return;
    ev.preventDefault();
    if (active) return;
    active = true;
    lastT = performance.now();
    acc = direction;   // fire one unit immediately so a quick tap still moves
    onStep(direction);
    try { button.setPointerCapture(ev.pointerId); } catch (_) {}
    rafId = requestAnimationFrame(tick);
  };

  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointercancel", stop);
  button.addEventListener("lostpointercapture", stop);
  window.addEventListener("blur", stop);
}

/** Map speed slider (50..1000) → motion velocity (units/sec).
 *
 *  Critical: the button velocity must NOT exceed the servo's configured
 *  goal_speed (which is state.speed in STS3215 units ≈ units/sec). If the
 *  UI slider runs ahead of the physical joint, the WebSocket feedback
 *  tries to drag it back, producing the "real arm can't follow gauge
 *  and twin desyncs" artifact. Matching 1:1 keeps intent aligned with
 *  what the hardware can actually achieve.
 */
function stepVelocity() {
  return state.speed;
}

// Throttled per-joint sender. Always forwards the *latest* queued position
// once the throttle window elapses — not the snapshot at scheduling time.
//
// 80 ms (~12 Hz) is the sweet spot for Feetech STS3215: faster than this and
// the servo's motion planner can't settle between commands, producing audible
// chatter and mechanical jitter. Slower than ~120 ms and button-hold feels
// laggy. The twin is still updated at 60 fps on the client so the *visual*
// motion stays silky smooth even though the bus traffic is much lighter.
const SEND_THROTTLE_MS = 80;

function scheduleSend(id, pos) {
  state.positions[id] = pos;
  if (!state.connected) return;
  state.pending[id] = true;
  if (state.sendThrottle) return;
  state.sendThrottle = setTimeout(async () => {
    state.sendThrottle = null;
    const pending = state.pending;
    state.pending = {};
    for (const jid of Object.keys(pending)) {
      try {
        await api.move(+jid, state.positions[+jid], state.speed);
      } catch (e) {
        if (e instanceof PermissionDeniedError) {
          // Permission revoked: stop hammering the bus with rejections;
          // the throttle queue would otherwise dispatch ~12 doomed
          // requests per second while the user holds the slider.
          state.pending = {};
          state.permitted = false;
          renderSelfPermission();
          toast("Your control was revoked by the instructor", "error");
          return;
        }
        toast(e.message, "error");
      }
    }
  }, SEND_THROTTLE_MS);
}


// ── Driver name card (every client) + Team-members admin panel ──────────
function setupSelfAndAdmin() {
  state.myClientId = getClientId();

  const nameInput = $("#ctrl-name");
  nameInput.value = getClientName();
  // Persist on every keystroke so the next request's x-client-name is
  // up to date — no explicit "save" button.
  nameInput.addEventListener("input", () => setClientName(nameInput.value));

  renderSelfPermission();
}

function renderSelfPermission() {
  const card = $("#ctrl-perm");
  const banner = $("#revoked-banner");
  card.querySelector(".txt").textContent = state.permitted ? "Permitted" : "Revoked";
  card.className = `ctrl-perm ${state.permitted ? "permitted" : "revoked"}`;
  banner.classList.toggle("show", !state.permitted);
}

function renderTeamPanel() {
  if (!state.isAdmin) return;
  $("#team-card").classList.remove("hidden");

  const list = $("#team-list");
  list.innerHTML = "";
  // Admin (= the main PC's local browser) is always at the top, marked
  // "you", and has no permit/revoke button so they can't lock themselves
  // out. Everyone else is sorted by recency.
  const ordered = [...state.clients].sort((a, b) => {
    if (a.cid === state.myClientId) return -1;
    if (b.cid === state.myClientId) return  1;
    return (b.last_seen || 0) - (a.last_seen || 0);
  });
  for (const c of ordered) {
    const isMe = c.cid === state.myClientId;
    const row = document.createElement("div");
    row.className = `team-row ${c.permitted ? "permitted" : "revoked"}${isMe ? " me" : ""}`;
    const sub = isMe
      ? "you (main PC)"
      : `${c.ip || "?"} · ${new Date((c.last_seen || 0) * 1000).toLocaleTimeString()}`;
    row.innerHTML = `
      <div class="who">${escapeHtml(c.name || "anonymous")}<span class="sub">${escapeHtml(sub)}</span></div>
      ${isMe ? "" : `<button class="perm-btn" data-cid="${escapeHtml(c.cid)}">${c.permitted ? "Permitted" : "Revoked"}</button>`}
    `;
    list.appendChild(row);
  }
  list.querySelectorAll(".perm-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const cid = btn.dataset.cid;
      const target = state.clients.find((x) => x.cid === cid);
      if (!target) return;
      try {
        await api.setPermission(cid, !target.permitted);
        // The WS broadcast will refresh state.clients shortly, but flip
        // locally too so the click feels instant.
        target.permitted = !target.permitted;
        renderTeamPanel();
      } catch (e) { toast(e.message, "error"); }
    });
  });
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    "&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;",
  }[c]));
}

/** Wrap any API write so a 423 (permission revoked) surfaces clearly. */
async function guarded(fn, ctx = "") {
  try { return await fn(); }
  catch (e) {
    if (e instanceof PermissionDeniedError) {
      state.permitted = false;
      renderSelfPermission();
      toast("Your control was revoked by the instructor", "error");
      return null;
    }
    toast(`${ctx ? ctx + ": " : ""}${e.message}`, "error");
    throw e;
  }
}


// ── Pages / nav ───────────────────────────────────────────────────────────
function setupNav() {
  $$(".nav-item").forEach((btn) => {
    btn.addEventListener("click", () => {
      const key = btn.dataset.page;
      $$(".nav-item").forEach((b) => b.classList.toggle("active",
                                                       b.dataset.page === key));
      $$(".page").forEach((p) => p.classList.toggle("hidden",
                                                    p.dataset.page !== key));
      $("#page-title").textContent = btn.textContent.trim();
    });
  });
}


// ── Connection ────────────────────────────────────────────────────────────
function setupConnection() {
  $("#port-input").value = guessDefaultPort();

  $("#connect-btn").addEventListener("click", async () => {
    setConnState("connecting");
    try {
      const port = $("#port-input").value.trim();
      const res = state.connected
        ? await api.disconnect()
        : await api.connect(port);
      if (res.connected === false) setConnState("offline");
      if (res.connected === true)  setConnState("online");
      toast(state.connected ? "Disconnected" : `Connected on ${port}`);
    } catch (e) {
      setConnState("offline");
      toast(e.message, "error");
    }
  });
}

function setConnState(kind) {
  state.connected = (kind === "online");
  const s = $("#conn-status");
  s.classList.remove("offline", "online", "connecting");
  s.classList.add(kind);
  s.querySelector(".txt").textContent =
    kind === "online" ? "Connected" :
    kind === "connecting" ? "Connecting…" : "Disconnected";
  $("#connect-btn").textContent = state.connected ? "Disconnect" : "Connect";
}


// ── Top bar actions ───────────────────────────────────────────────────────
function setupTopbar() {
  $("#center-btn").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (await guarded(() => api.center(state.speed), "center") !== null) {
      toast("Centering all joints");
    }
  });
  $("#lock-btn").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (await guarded(() => api.torque(true), "lock") !== null) {
      JOINT_IDS.forEach((id) => state.torque[id] = true);
      renderJointRows();
      toast("All joints locked");
    }
  });
  $("#unlock-btn").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (await guarded(() => api.torque(false), "unlock") !== null) {
      JOINT_IDS.forEach((id) => state.torque[id] = false);
      renderJointRows();
      toast("All joints unlocked");
    }
  });
}

function requireConnected() {
  if (!state.connected) { toast("Not connected", "error"); return false; }
  return true;
}


// ── Joint sliders ─────────────────────────────────────────────────────────
function buildJointRows() {
  const host = $("#joints");
  host.innerHTML = "";
  for (let i = 0; i < JOINT_IDS.length; i++) {
    const id = JOINT_IDS[i];
    const row = document.createElement("div");
    row.className = "joint-row";
    row.dataset.id = id;
    row.innerHTML = `
      <div class="joint-name">${JOINT_NAMES[i]}<span class="idx">ID ${id}</span></div>
      <button class="joint-step" data-step="-50">−</button>
      <input type="range" class="joint-slider" min="0" max="4095" value="2048" />
      <button class="joint-step" data-step="50">+</button>
      <div class="joint-value"><span class="raw">2048</span><span class="deg"> 0°</span></div>
      <button class="torque-btn free">Free</button>
    `;
    host.appendChild(row);

    const slider = row.querySelector(".joint-slider");
    const raw = row.querySelector(".raw");
    const deg = row.querySelector(".deg");
    const torque = row.querySelector(".torque-btn");

    const onSlide = () => {
      const lim = state.limits[id] || { min: 0, max: 4095 };
      const v = parseInt(slider.value, 10);
      markUserDriving(id);
      // Persist as the joint's intended position so WS handler knows what
      // to preserve while the user is driving.
      state.positions[id] = v;
      raw.textContent = v;
      deg.textContent = ` ${posToLabel(id, v)}`;
      const pct = ((v - lim.min) / (lim.max - lim.min || 1)) * 100;
      slider.style.setProperty("--pct", `${Math.max(0, Math.min(100, pct))}%`);
      scheduleSend(id, v);
      state.twin?.setPositions({ ...state.positions, [id]: v });
      updateHUD();
    };
    slider.addEventListener("input", onSlide);
    onSlide();

    row.querySelectorAll(".joint-step").forEach((b) => {
      const direction = parseInt(b.dataset.step, 10) > 0 ? 1 : -1;
      bindHoldMotion(b, direction, (delta) => {
        const lim = state.limits[id] || { min: 0, max: 4095 };
        slider.value = Math.max(lim.min, Math.min(lim.max,
          parseInt(slider.value, 10) + delta));
        slider.dispatchEvent(new Event("input"));
      }, stepVelocity);
    });

    torque.addEventListener("click", async () => {
      if (!requireConnected()) return;
      const next = !(state.torque[id] ?? false);
      await api.torque(next, id);
      state.torque[id] = next;
      renderJointRows();
    });
  }

  $("#speed").addEventListener("input", (e) => {
    state.speed = parseInt(e.target.value, 10);
    $("#speed-val").textContent = state.speed;
  });
}

function renderJointRows() {
  $$(".joint-row").forEach((row) => {
    const id = parseInt(row.dataset.id, 10);
    const locked = !!state.torque[id];
    row.classList.toggle("locked", locked);
    row.classList.toggle("free", !locked);
    const btn = row.querySelector(".torque-btn");
    btn.textContent = locked ? "Locked" : "Free";
    btn.classList.toggle("locked", locked);
    btn.classList.toggle("free", !locked);
  });
  updateLockPill();
}

// 상단 공통 메뉴의 lock/unlock 상태 배지 (관절 토크 상태 반영)
function updateLockPill() {
  const pill = document.getElementById("lock-pill");
  if (!pill) return;
  const states = JOINT_IDS.map((id) => !!state.torque[id]);
  const allLocked = states.every((s) => s);
  const allFree = states.every((s) => !s);
  pill.classList.toggle("locked", !allFree);   // 하나라도 잠겨 있으면 lock 색
  pill.classList.toggle("free", allFree);
  pill.textContent = allLocked ? "🔒 Lock" : allFree ? "Free" : "🔒 Mixed";
}

function updateJointRowPosition(id, pos) {
  const row = document.querySelector(`.joint-row[data-id="${id}"]`);
  if (!row) return;
  const lim = state.limits[id] || { min: 0, max: 4095 };
  const slider = row.querySelector(".joint-slider");
  const raw = row.querySelector(".raw");
  const deg = row.querySelector(".deg");
  slider.value = pos;
  const pct = ((pos - lim.min) / (lim.max - lim.min || 1)) * 100;
  slider.style.setProperty("--pct", `${Math.max(0, Math.min(100, pct))}%`);
  raw.textContent = pos;
  deg.textContent = ` ${posToLabel(id, pos)}`;
}

/**
 * Apply per-joint limits (fetched from /api/info) to slider min/max attrs.
 * Must run BEFORE the first status update so slider.value clamping works.
 */
function applyJointLimits() {
  for (const id of JOINT_IDS) {
    const lim = state.limits[id];
    if (!lim) continue;
    const row = document.querySelector(`.joint-row[data-id="${id}"]`);
    const slider = row?.querySelector(".joint-slider");
    if (!slider) continue;
    slider.min = lim.min;
    slider.max = lim.max;
    // If current value falls outside the new range, clamp and refresh.
    const v = parseInt(slider.value, 10);
    const c = Math.max(lim.min, Math.min(lim.max, v));
    if (c !== v) { slider.value = c; slider.dispatchEvent(new Event("input")); }
  }
}


// ── Digital twin ──────────────────────────────────────────────────────────
async function setupTwin() {
  const vp = $("#twin-viewport");
  const twin = new DigitalTwin(vp);
  state.twin = twin;
  // Exposed for in-browser debugging + our playwright probes. Zero cost.
  window.__soarm = { twin, state };

  const kin = await loadKinematics();
  state.kin = kin;
  // Pass joint calibration before loadRobot — the gripper mapping depends
  // on knowing (pos_min, pos_max) → (rad_min, rad_max).
  twin.setJointLimits(state.limits);
  await twin.loadRobot(kin);

  vp.addEventListener("twinReady", () => {
    $("#twin-loader").classList.add("done");
  });
  vp.addEventListener("twinFps", (ev) => {
    $("#hud-fps").textContent = `${ev.detail.hz} Hz`;
  });

  $("#twin-reset").addEventListener("click", () => twin.resetView());
  $("#twin-mesh").addEventListener("click", () => {
    twin.setMeshMode("solid");
    $("#twin-mesh").classList.add("active");
    $("#twin-wire").classList.remove("active");
  });
  $("#twin-wire").addEventListener("click", () => {
    twin.setMeshMode("wireframe");
    $("#twin-wire").classList.add("active");
    $("#twin-mesh").classList.remove("active");
  });

  // Fallback: hide loader after 6 s even if some meshes failed
  setTimeout(() => $("#twin-loader").classList.add("done"), 6000);
}

function updateHUD() {
  if (!state.twin) return;
  const [x, y, z] = state.twin.getEndEffectorPosition();
  $("#hud-eef").textContent =
    `${x.toFixed(3)} , ${y.toFixed(3)} , ${z.toFixed(3)} m`;
}


// ── Status page ───────────────────────────────────────────────────────────
function renderStatusTable(statusJoints) {
  const tbody = $("#status-table tbody");
  tbody.innerHTML = "";
  let online = 0;
  for (const id of JOINT_IDS) {
    const j = statusJoints?.[String(id)] ?? {
      id, name: JOINT_NAMES[id - 1], online: false,
    };
    if (j.online) online++;
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${id}</td>
      <td>${j.name}</td>
      <td class="${j.online ? "status-ok" : "status-off"}">${j.online ? "● ONLINE" : "● OFFLINE"}</td>
      <td class="mono">${j.position ?? "—"}</td>
      <td class="mono">${posToLabel(id, j.position)}</td>
      <td>${j.temperature != null ? j.temperature + "°C" : "—"}</td>
      <td>${j.voltage != null ? j.voltage + " V" : "—"}</td>
      <td class="mono">${j.load ?? "—"}</td>
      <td>${j.torque_enabled == null ? "—" : j.torque_enabled ? "ON" : "OFF"}</td>
    `;
    tbody.appendChild(tr);
  }
  $("#m-online").textContent = `${online} / 6`;
}

function setupStatusPage() {
  $("#status-refresh").addEventListener("click", async () => {
    try {
      const s = await api.status();
      renderStatusTable(s.joints);
      $("#m-port").textContent = s.port || "—";
      $("#m-conn").textContent = s.connected ? "Online" : "Offline";
    } catch (e) { toast(e.message, "error"); }
  });
}


// ── Teach & Poses ─────────────────────────────────────────────────────────
function setupTeachPage() {
  const grid = $("#pose-grid");
  grid.innerHTML = "";
  for (let i = 0; i < JOINT_IDS.length; i++) {
    const id = JOINT_IDS[i];
    const cell = document.createElement("div");
    cell.className = "pose-cell";
    cell.innerHTML = `
      <div class="lbl">${JOINT_NAMES[i]}</div>
      <div class="val"><span class="raw">—</span><span class="deg"> —°</span></div>
    `;
    cell.dataset.id = id;
    grid.appendChild(cell);
  }

  $("#pose-save").addEventListener("click", async () => {
    const name = $("#pose-name").value.trim();
    if (!name) return toast("Enter a pose name", "error");
    await api.poseSave(name);
    await refreshPoses();
    $("#pose-name").value = "";
    toast(`Saved pose "${name}"`);
  });
  $("#pose-go").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (!state.selectedPose) return toast("Select a pose first", "error");
    if (await guarded(() => api.poseGo(state.selectedPose, state.speed),
                      "go-to") !== null) {
      toast(`Going to "${state.selectedPose}"`);
    }
  });
  $("#pose-delete").addEventListener("click", async () => {
    if (!state.selectedPose) return;
    await api.poseDelete(state.selectedPose);
    toast(`Deleted "${state.selectedPose}"`);
    state.selectedPose = null;
    await refreshPoses();
  });
}

function updatePoseCells(positions) {
  for (const id of JOINT_IDS) {
    const pos = positions?.[String(id)];
    const cell = document.querySelector(`.pose-cell[data-id="${id}"]`);
    if (!cell) continue;
    cell.querySelector(".raw").textContent = pos ?? "—";
    cell.querySelector(".deg").textContent = pos != null ? ` ${posToLabel(id, pos)}` : " —";
  }
}

async function refreshPoses() {
  try { state.poses = await api.poses(); }
  catch { state.poses = {}; }
  const tbody = $("#pose-table tbody");
  tbody.innerHTML = "";
  for (const [name, data] of Object.entries(state.poses)) {
    const tr = document.createElement("tr");
    const pos = data.positions ?? {};
    tr.innerHTML = `
      <td><input type="radio" name="pose-sel"></td>
      <td>${name}</td>
      ${JOINT_IDS.map((id) => `<td class="mono">${pos[String(id)] ?? "—"}</td>`).join("")}
      <td>${data.timestamp ?? "—"}</td>
    `;
    tr.addEventListener("click", () => {
      state.selectedPose = name;
      $$("#pose-table tbody tr").forEach((r) => r.classList.toggle(
        "selected", r === tr));
      tr.querySelector("input").checked = true;
    });
    tbody.appendChild(tr);
  }
}


// ── Record & Play ─────────────────────────────────────────────────────────
// 모델: 손으로 움직인 걸 녹화 → 멈추면 파일로 저장 → 아래 목록에서 골라 바로 재생.
function setupRecordPage() {
  $("#rec-unlock").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (await guarded(() => api.torque(false), "unlock") !== null) {
      JOINT_IDS.forEach((id) => state.torque[id] = false);
      renderJointRows();
    }
  });

  // Record ↔ Stop. 멈추면 방금 녹화한 걸 곧바로 파일로 저장해 목록에 올린다.
  $("#rec-toggle").addEventListener("click", async () => {
    if (!requireConnected()) return;
    if (state.recording) {
      await api.recStop();
      const name = $("#rec-filename").value.trim() || defaultRecName();
      const res = await api.recSave(name);
      if (res.ok) { toast(`Saved: ${name}`); $("#rec-filename").value = ""; }
      else toast(res.error ?? "Save failed", "error");
      await refreshRecordingList();
    } else {
      const interval = parseInt($("#rec-interval").value, 10);
      await api.recStart(interval);
      toast("Recording… move the arm by hand");
    }
  });

  $("#play-stop").addEventListener("click", async () => {
    await api.playStop();
    toast("Playback stopped");
  });

  // 목록의 ▶ Play / 🗑 (이벤트 위임 — 행은 동적으로 생김)
  $("#rec-list-table tbody").addEventListener("click", async (e) => {
    const btn = e.target.closest("button[data-act]");
    if (!btn) return;
    const name = btn.dataset.name;
    if (btn.dataset.act === "play") {
      if (!requireConnected()) return;
      const loaded = await api.recOpen(name);            // 파일 → 버퍼
      if (!loaded.ok) return toast(loaded.error ?? "Open failed", "error");
      const res = await guarded(() => api.play(), "play");   // 기본 speed=1.0, loop=false
      if (res === null) return;
      if (!res.ok) return toast(res.error ?? "Cannot play", "error");
      toast(`Playing: ${name}`);
    } else if (btn.dataset.act === "del") {
      if (!confirm(`Delete recording "${name}"?`)) return;
      await api.recDelete(name);
      toast("Deleted");
      await refreshRecordingList();
    }
  });

  refreshRecordingList();
}

// 이름을 안 적었을 때 기본 파일명 (rec_20260709_143512)
function defaultRecName() {
  const d = new Date(), p = (n) => String(n).padStart(2, "0");
  return `rec_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}`
       + `_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

async function refreshRecordingList() {
  let items = [];
  try { items = (await api.recList()).items || []; } catch { /* ignore */ }
  const tbody = $("#rec-list-table tbody");
  tbody.innerHTML = "";
  const empty = $("#rec-empty");
  if (empty) empty.style.display = items.length ? "none" : "block";
  items.forEach((it) => {
    const name = typeof it === "string" ? it : it.name;      // 구버전(문자열) 방어
    const frames = typeof it === "string" ? "" : it.frames;
    const length = typeof it === "string" ? "" : `${it.seconds}s`;
    const tr = document.createElement("tr");
    // 이름은 서버에서 영숫자/-_ 공백만 남기고 sanitize 되므로 그대로 넣어도 안전.
    tr.innerHTML =
        `<td>${name}</td><td class="mono">${frames}</td><td class="mono">${length}</td>`
      + `<td style="text-align:right;white-space:nowrap">`
      + `<button class="btn btn-primary" style="padding:4px 10px" data-act="play" data-name="${name}">▶ Play</button> `
      + `<button class="btn btn-danger" style="padding:4px 10px" data-act="del" data-name="${name}">🗑</button></td>`;
    tbody.appendChild(tr);
  });
}


// ── WebSocket → live state ────────────────────────────────────────────────
function handleWsMessage(msg) {
  if (msg.type === "clients" && msg.data) {
    state.clients = msg.data;
    // Re-derive my own permission state from the broadcast — that's how
    // a non-admin learns the instructor revoked them in real time.
    const me = state.clients.find((c) => c.cid === state.myClientId);
    if (me) {
      state.permitted = !!me.permitted;
      renderSelfPermission();
    }
    renderTeamPanel();
    return;
  }
  if (msg.type === "status" && msg.data) {
    const s = msg.data;
    setConnState(s.connected ? "online" : "offline");
    $("#m-port").textContent = s.port || "—";
    $("#m-conn").textContent = s.connected ? "Online" : "Offline";

    // Positions — only update slider/twin target for joints the user is
    // NOT currently driving. Otherwise the lagging real-robot telemetry
    // would clobber the user's input, and the UI would look like it's
    // fighting itself (slider jumps back, twin flickers between target
    // and reality, servo never catches up).
    const effective = {};   // positions fed to the 3D twin
    for (const id of JOINT_IDS) {
      const wsPos = s.positions?.[String(id)];
      const row = document.querySelector(`.joint-row[data-id="${id}"]`);
      const slider = row?.querySelector(".joint-slider");

      if (state.userDriving[id]) {
        // User is driving this joint — keep their slider value, ignore WS.
        effective[String(id)] = state.positions[id];
      } else if (wsPos != null) {
        // Raw counts enter the UI here and nowhere else, so this is where we
        // lift them into the calibrated window. Everything downstream —
        // sliders, readouts, twin, the value we POST back — then speaks one
        // coordinate system; the server re-wraps before touching the bus.
        const p = unwrapPosition(wsPos, state.limits?.[id]);
        state.positions[id] = p;
        effective[String(id)] = p;
        if (slider && document.activeElement !== slider) {
          updateJointRowPosition(id, p);
        }
      } else if (state.positions[id] != null) {
        effective[String(id)] = state.positions[id];
      }

      // Torque state always follows WS (not user-owned).
      const j = s.joints?.[String(id)];
      if (j?.torque_enabled != null) state.torque[id] = !!j.torque_enabled;
    }
    renderJointRows();
    updatePoseCells(effective);
    state.twin?.setPositions(effective);
    updateHUD();

    if (s.joints) renderStatusTable(s.joints);

    // Recording state
    state.recording = !!s.recording;
    state.playing = !!s.playing;
    const recBtn = $("#rec-toggle");
    recBtn.textContent = state.recording ? "■ Stop" : "● Record";
    $("#rec-state").innerHTML = state.recording
      ? '<span class="dot" style="background: var(--danger); color: var(--danger)"></span>Recording'
      : state.playing
        ? '<span class="dot" style="background: var(--ok); color: var(--ok)"></span>Playing'
        : '<span class="dot"></span>Idle';
  } else if (msg.type === "recording" && msg.data) {
    $("#rec-count").textContent = `Frames: ${msg.data.frames}`;
  }
}


// ── Boot ──────────────────────────────────────────────────────────────────
async function boot() {
  // Order matters: wire every page + the critical control path BEFORE the
  // twin. That way even if three.js fails to load, Connect + sliders +
  // WebSocket still work — the 3D view is nice-to-have, not load-bearing.
  setupNav();
  setupConnection();
  setupSelfAndAdmin();
  setupTopbar();
  buildJointRows();
  setupStatusPage();
  setupTeachPage();
  setupRecordPage();

  // Load per-joint limits (eg gripper min/max + display kind + URDF rad
  // range) and apply to sliders + the twin. Non-fatal — sliders fall back
  // to the default 0..4095 range.
  try {
    const info = await api.info();
    if (info.joint_limits) {
      state.limits = Object.fromEntries(
        Object.entries(info.joint_limits).map(([k, v]) => [+k, v]));
    }
    applyTeamLabel(info.team || "");
    applyJointLimits();
    state.isAdmin = !!info.is_admin;
    if (state.isAdmin) {
      // Pull initial client list; thereafter the WS keeps it in sync.
      try {
        const r = await api.clients();
        state.clients = r.clients || [];
        renderTeamPanel();
      } catch (e) { console.warn("clients:", e); }
    }
  } catch (e) { console.warn("info:", e); }

  // Kick off twin load non-blocking. If three.js can't load we surface the
  // error but keep the rest of the app usable.
  setupTwin().catch((e) => {
    console.error("[twin] setup failed:", e);
    const loader = document.querySelector("#twin-loader");
    if (loader) {
      loader.textContent = "3D view unavailable — check browser console";
      loader.style.color = "var(--danger)";
    }
    toast("3D viewer failed to load (controls still work)", "error");
  });

  try { await refreshPoses(); } catch (e) { console.warn("poses:", e); }

  // Initial status snapshot
  try {
    const s = await api.status();
    handleWsMessage({ type: "status", data: s });
  } catch (e) { console.warn("status:", e); }

  openWebSocket(handleWsMessage,
    () => { $("#ws-pill").classList.add("live"); },
    () => { $("#ws-pill").classList.remove("live"); });

  // Exposed for probes/debugging — lets tests inject fake WS messages.
  window.__soarm_handleWs = handleWsMessage;
}

boot().catch((e) => {
  console.error("[boot] fatal:", e);
  toast(`Startup error: ${e.message}`, "error");
});
