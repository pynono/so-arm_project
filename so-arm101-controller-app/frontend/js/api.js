/**
 * Tiny fetch wrapper + WebSocket client for the SO-ARM101 controller.
 *
 * Uses relative URLs so this works wherever the frontend is served from.
 */

const BASE = "";   // same-origin, relative

// ── Client identity ───────────────────────────────────────────────────────
// Stable per-browser. Used by the soft control lock so the server knows
// who's driving and can refuse moves from non-holders. Persisted in
// localStorage so refreshing the page doesn't make you a "new" driver
// (which would let you steal the lock from yourself, confusing the UI).
export function getClientId() {
  let id = localStorage.getItem("soarm.cid");
  if (!id) {
    id = (crypto.randomUUID && crypto.randomUUID()) ||
         `c-${Math.random().toString(36).slice(2, 12)}-${Date.now()}`;
    localStorage.setItem("soarm.cid", id);
  }
  return id;
}
export function getClientName() {
  return localStorage.getItem("soarm.cname") || "";
}
export function setClientName(name) {
  localStorage.setItem("soarm.cname", (name || "").trim() || "anonymous");
}

/** Raised on 423 — the admin (main PC) has revoked this client's
 *  permission to drive. */
export class PermissionDeniedError extends Error {
  constructor(detail) {
    super(detail?.error || "your control was revoked by the instructor");
    this.name = "PermissionDeniedError";
    this.client = detail?.client || null;
  }
}

async function request(path, { method = "GET", body = null } = {}) {
  const headers = {
    "x-client-id": getClientId(),
    "x-client-name": getClientName() || "anonymous",
  };
  if (body) headers["content-type"] = "application/json";
  const res = await fetch(`${BASE}/api${path}`, {
    method,
    headers,
    body: body ? JSON.stringify(body) : null,
  });
  if (res.status === 423) {
    let detail = null;
    try { detail = (await res.json()).detail; } catch (_) {}
    throw new PermissionDeniedError(detail);
  }
  if (!res.ok) throw new Error(`${method} ${path}: ${res.status}`);
  const ct = res.headers.get("content-type") || "";
  return ct.includes("application/json") ? res.json() : res.text();
}

export const api = {
  info:        () => request("/info"),
  status:      () => request("/status"),
  connect:     (port) => request("/connect", { method: "POST", body: { port } }),
  disconnect:  () => request("/disconnect", { method: "POST" }),
  move:        (joint_id, position, speed = 200) =>
                 request("/move", { method: "POST",
                                    body: { joint_id, position, speed } }),
  moveAll:     (positions, speed = 200) =>
                 request("/move_all", { method: "POST",
                                        body: { positions, speed } }),
  center:      (speed = 200) => request("/center", { method: "POST",
                                                     body: { speed } }),
  torque:      (enable, joint_id = null) =>
                 request("/torque", { method: "POST",
                                      body: { enable, joint_id } }),

  // Poses
  poses:       () => request("/poses"),
  poseSave:    (name) => request("/poses/save", { method: "POST",
                                                  body: { name } }),
  poseGo:      (name, speed = 200) => request("/poses/go", { method: "POST",
                                                             body: { name, speed } }),
  poseDelete:  (name) => request(`/poses/${encodeURIComponent(name)}`,
                                 { method: "DELETE" }),

  // Recording
  recStart:    (interval_ms = 100) => request("/record/start", { method: "POST",
                                                                 body: { interval_ms } }),
  recStop:     () => request("/record/stop", { method: "POST" }),
  recClear:    () => request("/record/clear", { method: "POST" }),
  recGet:      () => request("/record"),
  recSave:     (name) => request("/record/save", { method: "POST",
                                                   body: { name } }),
  recOpen:     (name) => request("/record/open", { method: "POST",
                                                   body: { name } }),
  recList:     () => request("/record/list"),
  recDelete:   (name) => request(`/record/${encodeURIComponent(name)}`,
                                 { method: "DELETE" }),

  // Playback
  play:        (speed = 1.0, loop = false) => request("/playback/start",
                 { method: "POST", body: { speed, loop } }),
  playStop:    () => request("/playback/stop", { method: "POST" }),

  // Self / clients (the latter two are admin-only, gated by loopback)
  me:            () => request("/me"),
  clients:       () => request("/clients"),
  setPermission: (cid, permitted) =>
                    request(`/clients/${encodeURIComponent(cid)}/permission`,
                            { method: "POST", body: { permitted } }),
  forgetClient:  (cid) =>
                    request(`/clients/${encodeURIComponent(cid)}`,
                            { method: "DELETE" }),
};


/**
 * Open the status WebSocket. Auto-reconnects on disconnect.
 */
export function openWebSocket(onMessage, onOpen, onClose) {
  let ws;
  let retryMs = 500;
  let stopped = false;

  function connect() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${proto}//${location.host}/api/ws`;
    ws = new WebSocket(url);
    ws.addEventListener("open", () => {
      retryMs = 500;
      onOpen?.();
    });
    ws.addEventListener("message", (ev) => {
      try { onMessage(JSON.parse(ev.data)); }
      catch (e) { console.warn("ws parse", e); }
    });
    ws.addEventListener("close", () => {
      onClose?.();
      if (!stopped) {
        setTimeout(connect, Math.min(retryMs, 5000));
        retryMs = Math.min(retryMs * 1.6, 5000);
      }
    });
    ws.addEventListener("error", () => { try { ws.close(); } catch (_) {} });
  }

  connect();
  return { close: () => { stopped = true; try { ws.close(); } catch (_) {} } };
}
