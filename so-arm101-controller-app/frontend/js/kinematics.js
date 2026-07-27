/**
 * Shared kinematics helpers for SO-ARM101.
 */

export async function loadKinematics() {
  const res = await fetch("./model/kinematics.json");
  if (!res.ok) throw new Error("Failed to load kinematics.json");
  return res.json();
}

/** Counts per full servo revolution. */
export const POS_RANGE = 4096;

/**
 * Map a raw servo count into a joint's calibrated window.
 *
 * On arms where a joint's travel straddles the 0/4095 seam, calibration
 * records an unwrapped window whose max exceeds 4095 (a gripper closed at
 * 3915 that rolls over to 1302 fully open is stored as 3915..5398). Raw
 * counts must be lifted into that window before they're compared against
 * min/max, or the joint renders mirrored and pinned to its extremes.
 *
 * Windows that don't wrap keep max <= 4095 and this is a no-op. Idempotent.
 * Mirrors unwrap_position() in sdk/driver_sdk.py.
 */
export function unwrapPosition(pos, lim) {
  if (pos === null || pos === undefined) return pos;
  if (!lim || lim.max === undefined || lim.max <= 4095) return pos;
  const lo = lim.min ?? 0;
  return lo + (((pos - lo) % POS_RANGE) + POS_RANGE) % POS_RANGE;
}

/**
 * Where `pos` sits in a joint's calibrated window, as 0..1 — or null if the
 * joint has no window. The single place that turns a raw count into a
 * position; the twin's angle and the on-screen readout both come from here so
 * they can't drift apart.
 *
 * A count can land in the arc the joint can't reach — a stop pressed a few
 * counts past where it was measured, say. On a wrapped window that arc is on
 * the far side of the seam, so a hair below min unwraps to nearly a full
 * revolution above max. Snapping to the *circularly* nearest end keeps a
 * closed gripper reading 0 % instead of flipping to 100 %. Mirrors
 * _clamp_position() in backend/controller.py.
 *
 * `flip` means the servo counts down across the window, so min is physically
 * the far end.
 */
export function windowFraction(pos, lim) {
  if (pos === null || pos === undefined) return null;
  if (!lim || lim.min === undefined || lim.max === undefined) return null;
  const span = (lim.max - lim.min) || 1;
  let p = unwrapPosition(pos, lim);
  if (p > lim.max) {
    const pastMax = p - lim.max;
    const belowMin = lim.min + POS_RANGE - p;
    p = pastMax <= belowMin ? lim.max : lim.min;
  }
  const t = Math.max(0, Math.min(1, (p - lim.min) / span));
  return lim.flip ? 1 - t : t;
}

/** Convert STS3215 raw position (0..4095, center 2048) to radians. */
export function positionToRad(pos) {
  if (pos === null || pos === undefined) return 0;
  return ((pos - 2048) / 4095) * Math.PI * 2;
}

/** Radians → raw position. */
export function radToPosition(rad) {
  return Math.round(2048 + (rad / (Math.PI * 2)) * 4095);
}
