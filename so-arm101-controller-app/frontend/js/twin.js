/**
 * Three.js digital twin for SO-ARM101.
 *
 * URDF-style scene graph:
 *   - One THREE.Group per link. Each holds all its <visual> meshes at
 *     their per-visual xyz/rpy offset (URDF allows multiple visuals
 *     per link — SO-ARM101's base_link has 4, for example).
 *   - Links are chained by joints: each joint is a (parent-link
 *     transform) + (axis rotation group). Child link attaches under
 *     the axis group.
 *   - Driven joints (shoulder_pan ... gripper) are indexed 0..5 in
 *     this.driveGroups so servo callbacks can rotate them directly.
 *   - Fixed joints are just static transforms.
 *   - Angle in = STS3215 position (0..4095, center 2048) → radians
 *     via positionToRad(), applied to drive group's Z rotation (URDF
 *     axis is local Z for all SO-ARM101 revolute joints).
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

import { positionToRad, windowFraction } from "./kinematics.js";


const MESH_URL = (file) => `./meshes/${file}`;

const DEFAULT_MAT = { rgba: [0.4, 0.55, 0.85, 1.0] };


export class DigitalTwin {
  constructor(container) {
    this.container = container;
    this.kin = null;
    this.jointLimits = {};    // { id: {min, max, rad_min?, rad_max?} } from /api/info
    this.driveOrder = [];
    this.driveGroups = [];    // indexed by servo ID - 1
    this.targetAngles = [0, 0, 0, 0, 0, 0];
    this.driveAngles = [0, 0, 0, 0, 0, 0];
    this.linkGroups = {};     // name -> THREE.Group
    this.allMeshes = [];      // for wireframe toggle
    this.meshesTotal = 0;
    this.meshesLoaded = 0;

    // Angle smoothing — exponential glide. tau = seconds for ~63% convergence.
    // Small tau (e.g. 0.04) feels snappy but still hides 8 Hz WS ticks and
    // sub-pixel slider increments.
    this.smoothingTau = 0.04;
    this._lastFrameT = performance.now();

    this._initScene();
    this._bindResize();

    this._meshMode = "solid";
    this._frameCount = 0;
    this._fpsTimer = performance.now();
    this._fpsHz = 0;
  }

  // ── Scene ────────────────────────────────────────────────────────────
  _initScene() {
    const rect = this.container.getBoundingClientRect();
    const width = Math.max(rect.width, 400);
    const height = Math.max(rect.height, 400);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.setSize(width, height);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10);
    this.camera.position.set(0.42, 0.32, 0.42);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.10, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.15;
    this.controls.maxDistance = 1.5;

    // Lighting
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const key = new THREE.DirectionalLight(0xffffff, 1.1);
    key.position.set(0.6, 1.0, 0.4);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 3;
    key.shadow.bias = -0.0005;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x8fb6ff, 0.55);
    fill.position.set(-0.6, 0.5, -0.3);
    this.scene.add(fill);
    const rim = new THREE.DirectionalLight(0xffc089, 0.25);
    rim.position.set(0.0, 0.4, -0.7);
    this.scene.add(rim);

    // Ground + grid
    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(0.35, 64),
      new THREE.MeshStandardMaterial({
        color: 0x0a121e, roughness: 0.9, metalness: 0.0,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);
    const grid = new THREE.GridHelper(0.6, 24, 0x1e2a3f, 0x1e2a3f);
    grid.material.transparent = true;
    grid.material.opacity = 0.6;
    this.scene.add(grid);

    // Base frame axes (robot coords: Z up)
    const axes = new THREE.AxesHelper(0.06);
    axes.material.depthTest = false;
    axes.material.transparent = true;
    axes.material.opacity = 0.55;
    axes.renderOrder = 999;
    this.scene.add(axes);

    // World root: URDF is Z-up, Three.js default is Y-up → rotate once
    this.worldRoot = new THREE.Group();
    this.worldRoot.rotation.x = -Math.PI / 2;
    this.scene.add(this.worldRoot);

    this._animate = this._animate.bind(this);
    this.renderer.setAnimationLoop(this._animate);
  }

  // ── Robot graph ──────────────────────────────────────────────────────
  async loadRobot(kin) {
    this.kin = kin;
    this.driveOrder = kin.drive_order.slice();

    // Count total visuals to track load progress
    this.meshesTotal = 0;
    for (const lname of Object.keys(kin.links)) {
      this.meshesTotal += (kin.links[lname].visuals?.length ?? 0);
    }
    this.meshesLoaded = 0;

    // Build link groups first (empty transform containers)
    for (const lname of Object.keys(kin.links)) {
      const g = new THREE.Group();
      g.name = lname;
      this.linkGroups[lname] = g;
    }

    // Attach root link under world
    const rootName = kin.root_link || "base_link";
    this.worldRoot.add(this.linkGroups[rootName]);

    // Wire joints: parent_link → origin xform → axis-rotation group → child_link
    for (const j of kin.joints) {
      const parentG = this.linkGroups[j.parent];
      const childG = this.linkGroups[j.child];
      if (!parentG || !childG) continue;

      const originXform = new THREE.Group();
      originXform.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
      originXform.rotation.order = "ZYX";  // URDF rpy convention
      originXform.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2]);

      const axisGroup = new THREE.Group();
      axisGroup.userData = { axis: j.axis.slice(), jointName: j.name };

      originXform.add(axisGroup);
      axisGroup.add(childG);
      parentG.add(originXform);

      if (j.type === "revolute" && j.id != null) {
        this.driveGroups[j.id - 1] = axisGroup;
      }
    }

    // Load visuals for each link
    const loadPromises = [];
    for (const [lname, linkCfg] of Object.entries(kin.links)) {
      const g = this.linkGroups[lname];
      for (const v of linkCfg.visuals) {
        loadPromises.push(this._loadVisual(g, v));
      }
    }
    await Promise.allSettled(loadPromises);

    // End-effector marker
    const eefParent = this.linkGroups.gripper_frame_link
                    || this.linkGroups.moving_jaw_so101_v1_link
                    || this.linkGroups.gripper_link;
    if (eefParent) {
      this.eefMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.005, 16, 16),
        new THREE.MeshStandardMaterial({
          color: 0xff5a70, emissive: 0x551f26, emissiveIntensity: 0.7,
        }),
      );
      eefParent.add(this.eefMarker);
    }

    this.container.dispatchEvent(new CustomEvent("twinReady"));
    this.snapToTargets();
  }

  _loadVisual(linkGroup, v) {
    return new Promise((resolve) => {
      const loader = new STLLoader();
      const mat = this._materialFor(v.material);

      // Per-visual origin (child of link group)
      const visualXform = new THREE.Group();
      visualXform.position.set(v.xyz[0], v.xyz[1], v.xyz[2]);
      visualXform.rotation.order = "ZYX";  // URDF rpy convention
      visualXform.rotation.set(v.rpy[0], v.rpy[1], v.rpy[2]);
      linkGroup.add(visualXform);

      loader.load(
        MESH_URL(v.mesh),
        (geometry) => {
          geometry.computeVertexNormals();
          const mesh = new THREE.Mesh(geometry, mat);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          visualXform.add(mesh);
          this.allMeshes.push(mesh);
          this._onVisualDone();
          resolve();
        },
        undefined,
        (err) => {
          console.warn("STL load failed:", v.mesh, err);
          this._onVisualDone();
          resolve();
        },
      );
    });
  }

  _materialFor(name) {
    const m = this.kin?.materials?.[name] ?? DEFAULT_MAT;
    const [r, g, b] = m.rgba;
    const color = new THREE.Color(r, g, b);
    return new THREE.MeshStandardMaterial({
      color, roughness: 0.55, metalness: 0.12, flatShading: false,
    });
  }

  _onVisualDone() {
    this.meshesLoaded += 1;
    if (this.meshesLoaded >= this.meshesTotal) {
      this.container.dispatchEvent(new CustomEvent("twinReady"));
    }
  }

  // ── Update ───────────────────────────────────────────────────────────
  /** Apply per-joint calibration so mapping from servo counts to joint
   *  radians matches reality (critical for the gripper — see mapPosToRad). */
  setJointLimits(limits) {
    this.jointLimits = limits || {};
  }

  /**
   * Set target joint angles. Actual rendered angles glide toward these
   * every frame — never snap — so motion looks buttery regardless of
   * whether updates arrive at 60 Hz (user input) or 8 Hz (WebSocket).
   */
  setPositions(positionsDict) {
    for (let i = 0; i < 6; i++) {
      const id = i + 1;
      const pos = positionsDict?.[String(id)];
      if (pos == null) continue;
      this.targetAngles[i] = this._mapPosToRad(id, pos);
    }
  }

  /** pos → joint radians. A calibrated joint declares (rad_min, rad_max), so
   *  its measured window interpolates onto the URDF range — that's what
   *  absorbs this arm's servo-horn mounting offset, and what the gripper
   *  needs anyway since its servo-to-joint mapping isn't 1:1. Uncalibrated
   *  joints fall back to the "servo 4095 = 360°, center 2048" assumption.
   *  The `invert` flag flips the rendered direction for joints whose
   *  physical rotation goes the opposite way from what the URDF assumes. */
  _mapPosToRad(id, pos) {
    const lim = this.jointLimits[id] || {};
    let rad;
    const t = windowFraction(pos, lim);
    if (t !== null && lim.rad_min !== undefined && lim.rad_max !== undefined) {
      rad = lim.rad_min + t * (lim.rad_max - lim.rad_min);
    } else {
      rad = positionToRad(pos);
    }
    return lim.invert ? -rad : rad;
  }

  /** Force-snap to targets (used on initial load to avoid a startup glide). */
  snapToTargets() {
    for (let i = 0; i < 6; i++) this.driveAngles[i] = this.targetAngles[i];
    this._applyAngles();
  }

  _tickSmoothing(dt) {
    // Capped dt so huge gaps (tab backgrounded) don't cause a visible jump.
    const dtc = Math.min(dt, 0.1);
    const a = 1 - Math.exp(-dtc / this.smoothingTau);
    let changed = false;
    for (let i = 0; i < this.driveAngles.length; i++) {
      const diff = (this.targetAngles[i] ?? 0) - this.driveAngles[i];
      if (Math.abs(diff) > 1e-5) {
        this.driveAngles[i] += diff * a;
        changed = true;
      } else if (this.driveAngles[i] !== this.targetAngles[i]) {
        this.driveAngles[i] = this.targetAngles[i];
        changed = true;
      }
    }
    if (changed) this._applyAngles();
  }

  _applyAngles() {
    for (let i = 0; i < this.driveGroups.length; i++) {
      const g = this.driveGroups[i];
      if (!g) continue;
      const axis = g.userData.axis;
      // All SO-ARM101 revolute axes are local Z → short path
      if (axis[2] >= 0.99) {
        g.rotation.set(0, 0, this.driveAngles[i] || 0);
      } else {
        const a = new THREE.Vector3(axis[0], axis[1], axis[2]).normalize();
        const q = new THREE.Quaternion().setFromAxisAngle(
          a, this.driveAngles[i] || 0);
        g.setRotationFromQuaternion(q);
      }
    }
  }

  getEndEffectorPosition() {
    if (!this.eefMarker) return [0, 0, 0];
    const v = new THREE.Vector3();
    this.eefMarker.getWorldPosition(v);
    return [v.x, v.y, v.z];
  }

  setMeshMode(mode) {
    this._meshMode = mode;
    for (const m of this.allMeshes) {
      if (!m) continue;
      m.material.wireframe = (mode === "wireframe");
    }
  }

  resetView() {
    this.camera.position.set(0.42, 0.32, 0.42);
    this.controls.target.set(0, 0.10, 0);
    this.controls.update();
  }

  // ── Render loop ──────────────────────────────────────────────────────
  _animate() {
    const now = performance.now();
    const dt = (now - this._lastFrameT) / 1000;
    this._lastFrameT = now;

    this._tickSmoothing(dt);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this._frameCount += 1;
    if (now - this._fpsTimer >= 1000) {
      this._fpsHz = this._frameCount;
      this._frameCount = 0;
      this._fpsTimer = now;
      this.container.dispatchEvent(new CustomEvent("twinFps",
        { detail: { hz: this._fpsHz } }));
    }
  }

  _bindResize() {
    // Defer the resize handler with rAF. Chromium's ResizeObserver fires
    // synchronously inside the layout phase — if we change canvas size
    // right there the browser emits the harmless but noisy
    // "ResizeObserver loop completed with undelivered notifications"
    // warning. Running next frame breaks that synchronous loop.
    let scheduled = false;
    const ro = new ResizeObserver(() => {
      if (scheduled) return;
      scheduled = true;
      requestAnimationFrame(() => {
        scheduled = false;
        this._resize();
      });
    });
    ro.observe(this.container);
  }

  _resize() {
    const rect = this.container.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    this.renderer.setSize(rect.width, rect.height, false);
    this.camera.aspect = rect.width / rect.height;
    this.camera.updateProjectionMatrix();
  }
}
