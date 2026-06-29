// ============================================================
//  Map The Farm — Flight Controller Dashboard
//  script.js
// ============================================================

let scene, camera, renderer, cube;

// ── 3D Cube ──────────────────────────────────────────────────

function parentWidth(elem)  { return elem.parentElement.clientWidth; }
function parentHeight(elem) { return elem.parentElement.clientHeight || 400; }

function init3D() {
  const container = document.getElementById('3Dcube');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111318); // match --bg-panel

  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(container) / parentHeight(container),
    0.1,
    1000
  );

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(container), parentHeight(container));
  container.appendChild(renderer.domElement);

  // Drone body shape: wide, flat, rectangular
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  const cubeMaterials = [
    new THREE.MeshBasicMaterial({ color: 0x00e5a0 }), // right  — accent green
    new THREE.MeshBasicMaterial({ color: 0x009966 }), // left   — darker green
    new THREE.MeshBasicMaterial({ color: 0x1a2a20 }), // top    — dark
    new THREE.MeshBasicMaterial({ color: 0x0a0c0f }), // bottom — near black
    new THREE.MeshBasicMaterial({ color: 0x00c880 }), // front
    new THREE.MeshBasicMaterial({ color: 0x007744 }), // back
  ];

  cube = new THREE.Mesh(geometry, cubeMaterials);
  scene.add(cube);

  // Add subtle wireframe overlay so it reads as a technical object
  const wireframe = new THREE.LineSegments(
    new THREE.EdgesGeometry(geometry),
    new THREE.LineBasicMaterial({ color: 0x00e5a0, transparent: true, opacity: 0.15 })
  );
  cube.add(wireframe);

  camera.position.z = 7;
  renderer.render(scene, camera);
}

function onWindowResize() {
  const container = document.getElementById('3Dcube');
  camera.aspect = parentWidth(container) / parentHeight(container);
  camera.updateProjectionMatrix();
  renderer.setSize(parentWidth(container), parentHeight(container));
}

window.addEventListener('resize', onWindowResize, false);

init3D();

// ── Connection status ────────────────────────────────────────

function setConnected(state) {
  const dot   = document.getElementById('connectionDot');
  const label = document.getElementById('connectionLabel');
  if (state) {
    dot.className   = 'status-dot connected';
    label.textContent = 'CONNECTED';
  } else {
    dot.className   = 'status-dot disconnected';
    label.textContent = 'DISCONNECTED';
  }
}

// ── SSE Events ───────────────────────────────────────────────

if (!!window.EventSource) {
  const source = new EventSource('/events');

  source.addEventListener('open', () => setConnected(true), false);

  source.addEventListener('error', (e) => {
    if (e.target.readyState !== EventSource.OPEN) setConnected(false);
  }, false);

  // Orientation → rotate cube
  source.addEventListener('gyro_readings', (e) => {
    const obj = JSON.parse(e.data);
    document.getElementById('gyroX').textContent = parseFloat(obj.gyroX).toFixed(2);
    document.getElementById('gyroY').textContent = parseFloat(obj.gyroY).toFixed(2);
    document.getElementById('gyroZ').textContent = parseFloat(obj.gyroZ).toFixed(2);

    const deg = Math.PI / 180;
    cube.rotation.x = obj.gyroY * deg;
    cube.rotation.z = obj.gyroX * deg;
    cube.rotation.y = obj.gyroZ * deg;

    renderer.render(scene, camera);
  }, false);

  // Accelerometer
  source.addEventListener('accelerometer_readings', (e) => {
    const obj = JSON.parse(e.data);
    document.getElementById('accX').textContent = parseFloat(obj.accX).toFixed(3);
    document.getElementById('accY').textContent = parseFloat(obj.accY).toFixed(3);
    document.getElementById('accZ').textContent = parseFloat(obj.accZ).toFixed(3);
  }, false);

  // Temperature
  source.addEventListener('temperature_reading', (e) => {
    document.getElementById('temp').textContent = parseFloat(e.data).toFixed(1);
  }, false);

  // Altitude (BME280)
  source.addEventListener('altitude_reading', (e) => {
    document.getElementById('altitude').textContent = parseFloat(e.data).toFixed(2);
  }, false);

  // Motor status
  source.addEventListener('motor_readings', (e) => {
    const obj = JSON.parse(e.data);

    updateMotor(1, obj.m1);
    updateMotor(2, obj.m2);
    updateMotor(3, obj.m3);
    updateMotor(4, obj.m4);

    // Sync arm status display with what the ESP says
    const armed = obj.armed;
    const statusEl = document.getElementById('armStatus');
    if (armed) {
      statusEl.textContent = 'ARMED';
      statusEl.classList.add('armed');
    } else {
      statusEl.textContent = 'DISARMED';
      statusEl.classList.remove('armed');
    }

    // Keep slider disabled when disarmed
    document.getElementById('throttleSlider').disabled = !armed;
  }, false);
}

// ── Motor UI helpers ─────────────────────────────────────────

function updateMotor(index, pct) {
  const bar   = document.getElementById('bar'  + index);
  const label = document.getElementById('pct'  + index);
  const card  = document.getElementById('motor' + index);

  bar.style.width     = pct + '%';
  label.textContent   = pct + '%';
  card.classList.toggle('active', pct > 0);
}

// ── Throttle slider ──────────────────────────────────────────

function onThrottleChange(val) {
  document.getElementById('throttleDisplay').textContent = val + '%';
  const normalized = val / 100.0;
  fetch('/throttle?value=' + normalized);
}

// ── Arm / Disarm ─────────────────────────────────────────────

function armMotors() {
  // Safety: reset throttle to 0 before arming
  document.getElementById('throttleSlider').value = 0;
  document.getElementById('throttleDisplay').textContent = '0%';
  fetch('/throttle?value=0');

  fetch('/arm').then(() => {
    document.getElementById('throttleSlider').disabled = false;
  });
}

function disarmMotors() {
  // Zero throttle first, then disarm
  fetch('/throttle?value=0').then(() => fetch('/disarm'));
  document.getElementById('throttleSlider').value = 0;
  document.getElementById('throttleDisplay').textContent = '0%';
  document.getElementById('throttleSlider').disabled = true;
}

// ── Orientation reset ────────────────────────────────────────

function resetPosition(element) {
  fetch('/' + element.id);
}