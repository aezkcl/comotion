import * as THREE from "three";
import { TrackballControls } from "three/examples/jsm/controls/TrackballControls.js";

const DEFAULT_FILE = "viewer/data/panda_rake_demo.json";
const PLAY_FPS = 1.5;

let scene;
let camera;
let renderer;
let controls;

let resultData = null;
let currentRake = 0;
let isPlaying = false;
let playIntervalId = null;

let rakeReadoutEl;
let rangeReadoutEl;
let simdReadoutEl;
let playPauseBtn;
let sliderEl;

const geometryCache = new Map();
const materialCache = new Map();
const laneEntries = [];

// Match the publication palette used for the Panda rake visualization.
const BLUE_GRADIENT_STOPS = [
  "#d1e5f0",
  "#92c5de",
  "#4393c3",
  "#2166ac",
  "#08306b",
];
const YELLOW_GRADIENT_STOPS = [
  "#fff7bc",
  "#fee090",
  "#fec44f",
];
const COLLISION_ORANGE = "#f28e2b";

const greyMaterial = new THREE.MeshLambertMaterial({
  color: 0x7c8796,
  transparent: true,
  opacity: 0.14,
  depthWrite: false,
});

function withTrailingSlash(url) {
  return url.endsWith("/") ? url : `${url}/`;
}

function getAssetBaseUrl() {
  const params = new URLSearchParams(window.location.search);
  const override = params.get("assetBase");
  if (override) {
    return withTrailingSlash(new URL(override, window.location.href).href);
  }
  try {
    const moduleUrl = new URL(import.meta.url);
    if (moduleUrl.pathname.includes("/viewer/js/")) {
      return withTrailingSlash(new URL("../../", moduleUrl).href);
    }
  } catch (_) {
    /* ignore */
  }
  if (window.location.pathname.includes("/viewer/")) {
    return withTrailingSlash(new URL("../", window.location.href).href);
  }
  return withTrailingSlash(`${window.location.origin}/`);
}

function parseRakeData(text) {
  try {
    const data = JSON.parse(text);
    if (data.kind !== "comotion_panda_rake_demo") {
      console.error("Unexpected kind:", data.kind);
      return null;
    }
    if (!data.metadata || !Array.isArray(data.robots) || !Array.isArray(data.rakes)) {
      console.error("Missing rake viewer fields");
      return null;
    }
    return data;
  } catch (err) {
    console.error("JSON parse error:", err);
    return null;
  }
}

function getSphereGeometry(radius) {
  const key = radius.toFixed(6);
  if (!geometryCache.has(key)) {
    geometryCache.set(key, new THREE.SphereGeometry(radius, 18, 18));
  }
  return geometryCache.get(key);
}

function getMaterial(color, opacity, depthWrite = true) {
  const key = `${color}|${opacity}|${depthWrite ? 1 : 0}`;
  if (!materialCache.has(key)) {
    materialCache.set(
      key,
      new THREE.MeshLambertMaterial({
        color,
        transparent: opacity < 1,
        opacity,
        depthWrite,
      })
    );
  }
  return materialCache.get(key);
}

function interpolateGradient(stops, t) {
  const clamped = Math.max(0, Math.min(t, 1));
  if (stops.length === 1) return stops[0];
  const scaled = clamped * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(scaled));
  const localT = scaled - i;
  const colorA = new THREE.Color(stops[i]);
  const colorB = new THREE.Color(stops[i + 1]);
  colorA.lerp(colorB, localT);
  return `#${colorA.getHexString()}`;
}

function maxSourceTimestep(data) {
  const fromMetadata = data?.metadata?.full_motion_timesteps;
  if (Number.isFinite(fromMetadata) && fromMetadata > 1) {
    return fromMetadata - 1;
  }
  let maxTimestep = 0;
  for (const rake of data?.rakes || []) {
    for (const lane of rake.lanes || []) {
      maxTimestep = Math.max(maxTimestep, lane.timestep_index || 0);
    }
  }
  return maxTimestep;
}

function robotHighlightColor(robotIndex, timestepIndex) {
  const maxT = maxSourceTimestep(resultData);
  const t = maxT > 0 ? timestepIndex / maxT : 0;
  const stops = robotIndex === 0 ? BLUE_GRADIENT_STOPS : YELLOW_GRADIENT_STOPS;
  return interpolateGradient(stops, t);
}

function clearLaneEntries() {
  while (laneEntries.length > 0) {
    const entry = laneEntries.pop();
    scene.remove(entry.group);
  }
}

function applyLaneStyle(entry, highlighted) {
  for (const meshEntry of entry.meshEntries) {
    let material = greyMaterial;
    if (highlighted) {
      const color =
        entry.status === "inter_robot_collision"
          ? COLLISION_ORANGE
          : robotHighlightColor(meshEntry.robotIndex, entry.timestepIndex);
      material = getMaterial(
        color,
        entry.status === "inter_robot_collision" ? 0.94 : 0.9,
        true
      );
    }
    meshEntry.mesh.material = material;
    meshEntry.mesh.renderOrder = highlighted ? 1 : 0;
  }
}

function updateLaneStyles() {
  for (const entry of laneEntries) {
    applyLaneStyle(entry, entry.rakeIndex === currentRake);
  }
}

function updateUI() {
  if (!resultData) {
    rakeReadoutEl.textContent = "Rake 0 / 0";
    rangeReadoutEl.textContent = "Timesteps 0";
    simdReadoutEl.textContent = "Runtime SIMD 0 | Display lanes 0";
    playPauseBtn.disabled = true;
    playPauseBtn.textContent = "Play";
    sliderEl.disabled = true;
    return;
  }

  const totalRakes = resultData.metadata.display_rake_count ?? resultData.rakes.length;
  const maxRake = Math.max(0, totalRakes - 1);
  const laneCount = resultData.metadata.display_lanes_per_rake ?? 0;
  const currentRakeEntry = resultData.rakes[currentRake];
  const timesteps = currentRakeEntry
    ? currentRakeEntry.lanes.map((lane) => lane.timestep_index)
    : [];
  rakeReadoutEl.textContent = `Rake ${currentRake} / ${maxRake}`;
  rangeReadoutEl.textContent =
    timesteps.length > 0
      ? `Timesteps ${timesteps.join(", ")}`
      : "Timesteps ?";
  simdReadoutEl.textContent =
    `Runtime SIMD ${resultData.metadata.runtime_simd_width ?? 0} | Display lanes ${laneCount}`;
  playPauseBtn.disabled = maxRake <= 0;
  playPauseBtn.textContent = isPlaying ? "Pause" : "Play";
  sliderEl.disabled = false;
  sliderEl.min = "0";
  sliderEl.max = String(maxRake);
  sliderEl.value = String(currentRake);
}

function setRake(index) {
  if (!resultData) return;
  const maxRake = Math.max(0, resultData.rakes.length - 1);
  currentRake = Math.max(0, Math.min(index, maxRake));
  updateLaneStyles();
  updateUI();
}

function step(delta) {
  setRake(currentRake + delta);
}

function stopPlayback() {
  isPlaying = false;
  if (playIntervalId) {
    clearInterval(playIntervalId);
    playIntervalId = null;
  }
  updateUI();
}

function togglePlay() {
  if (!resultData) return;
  if (isPlaying) {
    stopPlayback();
    return;
  }

  isPlaying = true;
  playIntervalId = setInterval(() => {
    if (!resultData || currentRake >= resultData.rakes.length - 1) {
      stopPlayback();
      return;
    }
    step(1);
  }, 1000 / PLAY_FPS);
  updateUI();
}

function onKeyDown(event) {
  if (!resultData) return;
  switch (event.code) {
    case "ArrowLeft":
      step(-1);
      event.preventDefault();
      break;
    case "ArrowRight":
      step(1);
      event.preventDefault();
      break;
    case "Home":
      setRake(0);
      event.preventDefault();
      break;
    case "End":
      setRake(resultData.rakes.length - 1);
      event.preventDefault();
      break;
    case "Space":
      togglePlay();
      event.preventDefault();
      break;
  }
}

function fitCameraToData(data) {
  const box = new THREE.Box3();
  for (const rake of data.rakes) {
    for (const lane of rake.lanes) {
      for (const robot of lane.robots) {
        for (const sphere of robot.spheres) {
          const center = new THREE.Vector3(
            sphere.center[0],
            sphere.center[1],
            sphere.center[2]
          );
          const r = sphere.radius;
          box.expandByPoint(center.clone().addScalar(r));
          box.expandByPoint(center.clone().addScalar(-r));
        }
      }
    }
  }

  if (box.isEmpty()) {
    camera.position.set(2.2, 2.2, 1.8);
    controls.target.set(0.25, 0.0, 0.5);
    controls.update();
    return;
  }

  const size = new THREE.Vector3();
  const center = new THREE.Vector3();
  box.getSize(size);
  box.getCenter(center);
  const radius = Math.max(size.x, size.y, size.z, 0.4);
  camera.position.copy(center).add(new THREE.Vector3(radius * 1.3, radius * 1.2, radius));
  controls.target.copy(center);
  camera.near = 0.01;
  camera.far = Math.max(100, radius * 15);
  camera.updateProjectionMatrix();
  controls.update();
}

function buildLaneMeshes(data) {
  clearLaneEntries();

  for (const rake of data.rakes) {
    for (const lane of rake.lanes) {
      const group = new THREE.Group();
      const meshEntries = [];
      for (const robot of lane.robots) {
        for (const sphere of robot.spheres) {
          const mesh = new THREE.Mesh(getSphereGeometry(sphere.radius), greyMaterial);
          mesh.position.set(sphere.center[0], sphere.center[1], sphere.center[2]);
          mesh.castShadow = false;
          mesh.receiveShadow = false;
          group.add(mesh);
          meshEntries.push({
            mesh,
            robotIndex: robot.robot_index,
          });
        }
      }
      scene.add(group);
      laneEntries.push({
        rakeIndex: rake.index,
        laneIndex: lane.lane_index,
        timestepIndex: lane.timestep_index,
        status: lane.status,
        group,
        meshEntries,
      });
    }
  }
}

function loadResult(data) {
  stopPlayback();
  resultData = data;
  currentRake = 0;
  buildLaneMeshes(data);
  fitCameraToData(data);
  updateLaneStyles();
  updateUI();
}

function loadFromFile(file) {
  const reader = new FileReader();
  reader.onload = (event) => {
    const data = parseRakeData(event.target.result);
    if (!data) {
      alert("Invalid rake demo JSON.");
      return;
    }
    loadResult(data);
  };
  reader.readAsText(file);
}

async function loadFromUrl(path) {
  try {
    const url = path.startsWith("http")
      ? path
      : new URL(path.replace(/^\//, ""), getAssetBaseUrl()).href;
    const response = await fetch(url);
    const text = await response.text();
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${path}`);
    }
    const data = parseRakeData(text);
    if (!data) {
      throw new Error("Invalid rake demo JSON.");
    }
    loadResult(data);
  } catch (err) {
    console.error(err);
    alert(`Failed to load ${path}: ${err.message}`);
  }
}

function initScene() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 150);
  camera.position.set(2.2, 2.2, 1.8);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setClearColor(0xffffff, 1);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = false;
  document.getElementById("canvas-container").appendChild(renderer.domElement);

  controls = new TrackballControls(camera, renderer.domElement);
  controls.rotateSpeed = 1.35;
  controls.staticMoving = false;
  controls.dynamicDampingFactor = 0.06;

  const ambient = new THREE.AmbientLight(0xffffff, 0.86);
  scene.add(ambient);

  const sun = new THREE.DirectionalLight(0xffffff, 0.72);
  sun.position.set(3.5, 2.2, 4.8);
  scene.add(sun);

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    controls.handleResize();
  });
  window.addEventListener("keydown", onKeyDown);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

function init() {
  initScene();

  rakeReadoutEl = document.getElementById("rake-readout");
  rangeReadoutEl = document.getElementById("range-readout");
  simdReadoutEl = document.getElementById("simd-readout");
  playPauseBtn = document.getElementById("play-pause");
  sliderEl = document.getElementById("rake-slider");

  document.getElementById("file-input").addEventListener("change", (event) => {
    const file = event.target.files[0];
    if (file) {
      loadFromFile(file);
    }
  });

  playPauseBtn.addEventListener("click", togglePlay);
  sliderEl.addEventListener("input", (event) => {
    setRake(parseInt(event.target.value, 10));
  });

  updateUI();
  animate();

  const params = new URLSearchParams(window.location.search);
  loadFromUrl(params.get("file") || DEFAULT_FILE);
}

init();
