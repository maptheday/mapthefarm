// ============================================================
//  Map The Farm — Autonomous Flight Dashboard
//  script.js
// ============================================================

let scene, camera, renderer, cube;

const MAX_ALT_FT = 50; // matches ESP32 safety cap

// ── 3D Cube ──────────────────────────────────────────────────

function parentWidth(elem)  { return elem.parentElement.clientWidth; }
function parentHeight(elem) { return elem.parentElement.clientHeight || 400; }

function init3D() {
  const container = document.getElementById('3Dcube');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111318);

  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(container) / parentHeight(container),
    0.1, 1000
  );

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(container), parentHeight(container));
  container.appendChild(renderer.domElement);

  const geometry = new THREE.BoxGeometry(5, 1, 4);

  const materials = [
    new THREE.MeshBasicMaterial({ color: 0x00e5a0 }),
    new THREE.MeshBasicMaterial({ color: 0x009966 }),
    new THREE.MeshBasicMaterial({ color: 0x1a2a20 }),
    new THREE.MeshBasicMaterial({ color: 0x0a0c0f }),
    new THREE.MeshBasicMaterial({ color: 0x00c880 }),
    new THREE.MeshBasicMaterial({ color: 0x007744 }),
  ];

  cube = new THREE.Mesh(geometry, materials);
  scene.add(cube);

  // Subtle wireframe overlay
  const wireframe = new THREE.LineSegments(
    new THREE.EdgesGeometry(geometry),
    new THREE.LineBasicMaterial({ color: 0x00e5a0, transparent: true, opacity: 0.15 })
  );
  cube.add(wireframe);

  camera.position.z = 7;
  renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
  const container = document.getElementById('3Dcube');
  camera.aspect = parentWidth(container) / parentHeight(container);
  camera.updateProjectionMatrix();
  renderer.setSize(parentWidth(container), parentHeight(container));
}, false);

init3D();

// ── Connection status ────────────────────────────────────────

function setConnected(connected) {
  const dot   = document.getElementById('connectionDot');
  const label = document.getElementById('connectionLabel');
  dot.className     = 'status-dot ' + (connected ? 'connected' : 'disconnected');
  label.textContent = connected ? 'CONNECTED' : 'DISCONNECTED';
}

// ── SSE ──────────────────────────────────────────────────────

if (!!window.EventSource) {
  const source = new EventSource('/events');

  source.addEventListener('open',  () => setConnected(true),  false);
  source.addEventListener('error', (e) => {
    if (e.target.readyState !== EventSource.OPEN) setConnected(false);
  }, false);

  // Orientation → rotate cube
  source.addEventListener('gyro_readings', (e) => {
    const obj = JSON.parse(e.data);
    const deg = Math.PI / 180;

    document.getElementById('gyroX').textContent = parseFloat(obj.gyroX).toFixed(1);
    document.getElementById('gyroY').textContent = parseFloat(obj.gyroY).toFixed(1);
    document.getElementById('gyroZ').textContent = parseFloat(obj.gyroZ).toFixed(1);

    cube.rotation.x = obj.gyroY * deg;
    cube.rotation.z = obj.gyroX * deg;
    cube.rotation.y = obj.gyroZ * deg;

    renderer.render(scene, camera);
  }, false);

  // Flight data: altitude, motors, PID
  source.addEventListener('flight_readings', (e) => {
    const obj = JSON.parse(e.data);

    // Altitude display
    const actual = parseFloat(obj.altFt).toFixed(1);
    const target = parseFloat(obj.targetFt).toFixed(1);
    document.getElementById('actualAlt').textContent = actual;
    document.getElementById('targetAlt').textContent = target;

    // Altitude bar — actual (green fill) and target (amber marker)
    const actualPct = Math.min((obj.altFt / MAX_ALT_FT) * 100, 100);
    const targetPct = Math.min((obj.targetFt / MAX_ALT_FT) * 100, 100);
    document.getElementById('altBar').style.width         = actualPct + '%';
    document.getElementById('altBarTarget').style.left    = targetPct + '%';

    // PID debug
    document.getElementById('baseThrottle').textContent   = obj.baseThrottle;
    document.getElementById('rollCorrection').textContent  = obj.rollCorrection;
    document.getElementById('pitchCorrection').textContent = obj.pitchCorrection;

    // Motors
    updateMotor(1, obj.m1);
    updateMotor(2, obj.m2);
    updateMotor(3, obj.m3);
    updateMotor(4, obj.m4);

    // Flight badge
    const badge = document.getElementById('flightBadge');
    if (!obj.flightEnabled) {
      badge.textContent = 'GROUNDED';
      badge.className   = 'flight-badge';
    } else if (obj.targetFt === 0) {
      badge.textContent = 'LANDING';
      badge.className   = 'flight-badge landing';
    } else {
      badge.textContent = 'FLYING';
      badge.className   = 'flight-badge flying';
    }
  }, false);
}

// ── Motor UI ─────────────────────────────────────────────────

function updateMotor(index, pct) {
  document.getElementById('bar'   + index).style.width  = pct + '%';
  document.getElementById('pct'   + index).textContent  = pct + '%';
  document.getElementById('motor' + index).classList.toggle('active', pct > 0);
}

// ── Flight commands ───────────────────────────────────────────

function fly() {
  const val = parseFloat(document.getElementById('altInput').value);
  if (isNaN(val) || val <= 0) {
    alert('Enter a target altitude greater than 0 ft.');
    return;
  }
  if (val > MAX_ALT_FT) {
    alert('Max altitude is ' + MAX_ALT_FT + ' ft.');
    return;
  }
  fetch('/fly?ft=' + val);
}

function land() {
  fetch('/land');
}

function stop() {
  if (!confirm('Emergency stop will cut all motors immediately. Continue?')) return;
  fetch('/stop');
}

function resetPosition(element) {
  fetch('/' + element.id);
}