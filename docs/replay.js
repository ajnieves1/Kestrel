// Canvas mission replay viewer. Plain JS, no dependencies.

const SITES = {
  pylon: {
    label: 'Pylon',
    jsonPath: 'replay_data/pylon_20260719_005900.json',
    photoDirectory: 'pylon_20260719_005900_photos',
    footprint: { type: 'square', center: { north: 15.0, east: 0.0 }, halfWidth: 2.1 },
    groundTruth: [
      { label: 'marker_0', north: 16.66, east: 0.0 },
      { label: 'marker_1', north: 15.0, east: 1.66 },
      { label: 'marker_2', north: 13.34, east: 0.0 },
      { label: 'marker_3', north: 15.0, east: -1.66 },
    ],
  },
  turbine: {
    label: 'Turbine',
    jsonPath: 'replay_data/turbine_20260719_010330.json',
    photoDirectory: 'turbine_20260719_010330_photos',
    footprint: { type: 'circle', center: { north: 15.0, east: 0.0 }, radius: 0.8 },
    groundTruth: [
      { label: 'marker_0', north: 15.81, east: 0.0 },
      { label: 'marker_1', north: 15.0, east: 0.81 },
      { label: 'marker_2', north: 14.19, east: 0.0 },
      { label: 'marker_3', north: 15.0, east: -0.81 },
    ],
  },
};

const canvas = document.getElementById('replay-canvas');
const context = canvas.getContext('2d');
const siteSelect = document.getElementById('site-select');
const playButton = document.getElementById('play-button');
const scrubInput = document.getElementById('scrub-input');
const speedSelect = document.getElementById('speed-select');
const stateLabel = document.getElementById('state-label');
const timeLabel = document.getElementById('time-label');
const photoPanel = document.getElementById('photo-panel');

let activeSite = null;
let activeRecord = null;
let currentTime = 0;
let isPlaying = false;
let lastFrameTime = null;
let visiblePins = [];

// Load one site's replay JSON and reset the viewer to time zero
async function loadSite(siteKey) {
  activeSite = SITES[siteKey];
  const response = await fetch(activeSite.jsonPath);
  activeRecord = await response.json();
  currentTime = 0;
  isPlaying = false;
  playButton.textContent = 'Play';
  scrubInput.max = activeRecord.duration_seconds;
  scrubInput.value = 0;
  photoPanel.innerHTML = '';
  draw(currentTime);
}

// Convert a north east position to a canvas pixel position
function project(bounds, north, east) {
  const scale = Math.min(
    (canvas.width - 40) / (bounds.maxEast - bounds.minEast),
    (canvas.height - 40) / (bounds.maxNorth - bounds.minNorth));
  const x = 20 + (east - bounds.minEast) * scale;
  const y = canvas.height - 20 - (north - bounds.minNorth) * scale;
  return { x, y };
}

// Compute a padded bounding box that covers the path, footprint, and markers
function computeBounds(record, site) {
  const norths = record.poses.map((pose) => pose[1]);
  const easts = record.poses.map((pose) => pose[2]);
  site.groundTruth.forEach((marker) => {
    norths.push(marker.north);
    easts.push(marker.east);
  });
  const padding = 4.0;
  return {
    minNorth: Math.min(...norths) - padding,
    maxNorth: Math.max(...norths) + padding,
    minEast: Math.min(...easts) - padding,
    maxEast: Math.max(...easts) + padding,
  };
}

// Draw the structure footprint from hardcoded site geometry
function drawFootprint(bounds, footprint) {
  context.strokeStyle = '#6b5a47';
  context.lineWidth = 2;
  if (footprint.type === 'circle') {
    const center = project(bounds, footprint.center.north, footprint.center.east);
    const edge = project(bounds, footprint.center.north, footprint.center.east + footprint.radius);
    const radiusPixels = edge.x - center.x;
    context.beginPath();
    context.arc(center.x, center.y, radiusPixels, 0, Math.PI * 2);
    context.stroke();
  } else {
    const half = footprint.halfWidth;
    const corners = [
      project(bounds, footprint.center.north + half, footprint.center.east - half),
      project(bounds, footprint.center.north + half, footprint.center.east + half),
      project(bounds, footprint.center.north - half, footprint.center.east + half),
      project(bounds, footprint.center.north - half, footprint.center.east - half),
    ];
    context.beginPath();
    context.moveTo(corners[0].x, corners[0].y);
    corners.slice(1).forEach((corner) => context.lineTo(corner.x, corner.y));
    context.closePath();
    context.stroke();
  }
}

// Draw a ring for each ground truth marker position
function drawGroundTruth(bounds, groundTruth) {
  context.strokeStyle = '#7d9cb0';
  context.lineWidth = 1.5;
  groundTruth.forEach((marker) => {
    const point = project(bounds, marker.north, marker.east);
    context.beginPath();
    context.arc(point.x, point.y, 6, 0, Math.PI * 2);
    context.stroke();
  });
}

// Draw the flown path up to the given time, and the copter at that time
function drawPath(bounds, poses, time) {
  const flownPoses = poses.filter((pose) => pose[0] <= time);
  if (flownPoses.length === 0) {
    return;
  }

  context.strokeStyle = '#cf6636';
  context.lineWidth = 2;
  context.beginPath();
  flownPoses.forEach((pose, index) => {
    const point = project(bounds, pose[1], pose[2]);
    if (index === 0) {
      context.moveTo(point.x, point.y);
    } else {
      context.lineTo(point.x, point.y);
    }
  });
  context.stroke();

  const lastPose = flownPoses[flownPoses.length - 1];
  const copterPoint = project(bounds, lastPose[1], lastPose[2]);
  context.fillStyle = '#cf6636';
  context.beginPath();
  context.arc(copterPoint.x, copterPoint.y, 5, 0, Math.PI * 2);
  context.fill();
}

// Draw a pin for each defect event fired by the given time, tracked for clicks
function drawEvents(bounds, events, time) {
  visiblePins = [];
  context.fillStyle = '#f2e7cf';
  events.filter((event) => event.t <= time).forEach((event) => {
    const point = project(bounds, event.north, event.east);
    context.beginPath();
    context.arc(point.x, point.y, 7, 0, Math.PI * 2);
    context.fill();
    visiblePins.push({ x: point.x, y: point.y, event });
  });
}

// Show the clicked pin's photo, or clear the panel if no pin was hit
function handleCanvasClick(clickEvent) {
  const canvasRect = canvas.getBoundingClientRect();
  const clickX = (clickEvent.clientX - canvasRect.left) * (canvas.width / canvasRect.width);
  const clickY = (clickEvent.clientY - canvasRect.top) * (canvas.height / canvasRect.height);

  const hitPin = visiblePins.find((pin) => {
    const distance = Math.hypot(pin.x - clickX, pin.y - clickY);
    return distance <= 10;
  });

  if (!hitPin) {
    return;
  }

  const photoUrl = `replay_data/${activeSite.photoDirectory}/${hitPin.event.photo}`;
  photoPanel.innerHTML =
    `<p><strong>${hitPin.event.label}</strong>, confidence ${hitPin.event.confidence.toFixed(2)}</p>` +
    `<img src="${photoUrl}" alt="${hitPin.event.label} photo">`;
}

// Find the mission state active at the given time
function stateAtTime(states, time) {
  let current = states[0] ? states[0][1] : 'IDLE';
  states.forEach((entry) => {
    if (entry[0] <= time) {
      current = entry[1];
    }
  });
  return current;
}

// Render one frame of the replay at the given time
function draw(time) {
  if (!activeRecord) {
    return;
  }

  context.clearRect(0, 0, canvas.width, canvas.height);
  const bounds = computeBounds(activeRecord, activeSite);
  drawFootprint(bounds, activeSite.footprint);
  drawGroundTruth(bounds, activeSite.groundTruth);
  drawPath(bounds, activeRecord.poses, time);
  drawEvents(bounds, activeRecord.events, time);

  stateLabel.textContent = stateAtTime(activeRecord.states, time);
  timeLabel.textContent = `${time.toFixed(1)}s / ${activeRecord.duration_seconds.toFixed(1)}s`;
  scrubInput.value = time;
}

// Advance playback by real elapsed time times the selected speed
function playbackStep(frameTime) {
  if (!isPlaying) {
    return;
  }
  if (lastFrameTime === null) {
    lastFrameTime = frameTime;
  }
  const deltaSeconds = (frameTime - lastFrameTime) / 1000;
  lastFrameTime = frameTime;

  const speed = parseFloat(speedSelect.value);
  currentTime = Math.min(currentTime + deltaSeconds * speed, activeRecord.duration_seconds);
  draw(currentTime);

  if (currentTime >= activeRecord.duration_seconds) {
    isPlaying = false;
    playButton.textContent = 'Play';
    return;
  }
  requestAnimationFrame(playbackStep);
}

canvas.addEventListener('click', handleCanvasClick);

siteSelect.addEventListener('change', () => loadSite(siteSelect.value));

playButton.addEventListener('click', () => {
  isPlaying = !isPlaying;
  playButton.textContent = isPlaying ? 'Pause' : 'Play';
  if (isPlaying) {
    lastFrameTime = null;
    requestAnimationFrame(playbackStep);
  }
});

scrubInput.addEventListener('input', () => {
  isPlaying = false;
  playButton.textContent = 'Play';
  currentTime = parseFloat(scrubInput.value);
  draw(currentTime);
});

loadSite(siteSelect.value);
