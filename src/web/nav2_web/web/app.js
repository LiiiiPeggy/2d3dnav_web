'use strict';

const state = {
  socket: null,
  connected: false,
  map: null,
  mapImage: null,
  pose: null,
  cmdVel: { vx: 0, vy: 0, wz: 0, age: null },
  goal: null,
  path: [],
  mppiTrajectories: {
    frame_id: 'odom', target_frame: 'map', transform_ready: false,
    candidate_count: 0, optimal_count: 0,
    candidate_points: [], optimal_points: [],
  },
  trail: [],
  scanPoints: {
    source_frame: 'base_scan', target_frame: 'map',
    local_points: [], map_points: [], transform_ready: false, reset_pending: false,
  },
  particles: { frame_id: 'map', count: 0, points: [], spread: null },
  localCostmap: { frame_id: 'odom', age: null },
  costmaps: { global: null, local: null },
  inflation: {
    global: { ready: false, state: 'waiting' },
    local: { ready: false, state: 'waiting' },
  },
  overlays: {
    scan: true, particles: true,
    globalCostmap: false, localCostmap: true,
    globalPath: true, mppiTrajectories: true,
  },
  nav: { state: 'idle', message: '等待导航目标' },
  workflow: {
    stage: 'waiting', title: '等待 ROS 流程',
    message: '请先启动 SLAM 建图，或加载地图启动定位/Nav2',
    slam_ready: false, amcl_ready: false,
    cartographer_localization_ready: false, localizer_ready: false,
    localizer_type: 'amcl', planner_ready: false,
  },
  mapping: null,
  localizationReset: { state: 'idle', message: '定位器未重置' },
  localizationChoice: null,
  saveMap: { state: 'idle', message: '尚未保存地图', path: null },
  mapSaveDirectory: '',
  launchControl: {
    enabled: false,
    profiles: [],
    maps: [],
    active: null,
    logs: [],
    map_directory: '',
  },
  ready: {
    map: false, tf: false, nav2: false, scan: false,
    saveMap: false, localizationReset: false,
  },
  topics: { cmd_vel: '/cmd_vel' },
  frames: {
    map: 'map', odom: 'odom', base: 'base_link',
    scan: 'base_scan', local_costmap: 'odom',
  },
  mode: 'view',
  mapExpanded: false,
  autoFitMap: true,
  view: { zoom: 1, panX: 0, panY: 0 },
  pointerStart: null,
  pointerCurrent: null,
  dragLast: null,
  dirty: true,
};

const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const localScanCanvas = document.getElementById('localScanCanvas');
const localScanContext = localScanCanvas.getContext('2d');
const $ = (id) => document.getElementById(id);
const activeViewPointers = new Map();
let wsPort = Number(window.NAV2_ANDROID_WS_PORT) || 8891;
let toastTimer = null;
let pinchGesture = null;

function isGraphLocalization() {
  return state.workflow?.localizer_type === 'cartographer';
}

function localizerReady() {
  return Boolean(state.workflow?.localizer_ready ?? state.workflow?.amcl_ready);
}

function activeLocalizationType() {
  if (!localizerReady()) return null;
  return state.workflow?.localizer_type === 'cartographer'
    ? 'cartographer' : 'amcl';
}

function localizationSelectionMatchesBackend() {
  const active = activeLocalizationType();
  return !state.localizationChoice || !active
    || state.localizationChoice === active;
}

function setHealth(id, ok, okText, waitingText, isError = false) {
  const element = $(id);
  element.classList.toggle('ok', ok);
  element.classList.toggle('pending', !ok && !isError);
  element.classList.toggle('error', isError);
  element.querySelector('span').textContent = ok ? okText : waitingText;
}

function showToast(message) {
  const toast = $('toast');
  toast.textContent = message;
  toast.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove('show'), 2600);
}

function syncViewportHeight() {
  const height = Math.max(1, window.visualViewport?.height || window.innerHeight);
  const width = Math.max(1, window.visualViewport?.width || window.innerWidth);
  const pixelHeight = `${Math.ceil(height)}px`;
  const portrait = width <= height && width <= 900;
  const compactLandscape = width > height && width <= 850;
  const root = document.documentElement;
  root.classList.toggle('portrait-layout', portrait);
  root.classList.toggle('compact-landscape', compactLandscape);
  root.classList.toggle('small-landscape', compactLandscape && width <= 700);
  root.classList.toggle('short-viewport', height <= 570);
  root.style.height = pixelHeight;
  document.body.style.height = pixelHeight;
  document.querySelector('.app-shell').style.height = pixelHeight;
}

async function boot() {
  syncViewportHeight();
  document.documentElement.classList.toggle(
    'android-app', Boolean(window.Nav2Android),
  );
  try {
    const response = await fetch('/config.json', { cache: 'no-store' });
    if (response.ok) {
      const config = await response.json();
      wsPort = Number(config.ws_port) || wsPort;
    }
  } catch (_error) {
    // Use the default port when the config endpoint is unavailable.
  }
  connectWebSocket();
  resizeCanvas();
  updateClock();
  setInterval(updateClock, 1000);
  requestAnimationFrame(renderLoop);
}

function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
  const url = `${protocol}://${window.location.hostname}:${wsPort}`;
  setHealth('wsHealth', false, 'Web 已连接', 'Web 连接中');

  const socket = new WebSocket(url);
  state.socket = socket;
  socket.onopen = () => {
    state.connected = true;
    clearDynamicVisualization(true, false);
    setHealth('wsHealth', true, 'Web 已连接', 'Web 连接中');
    send({ type: 'request_snapshot' });
  };
  socket.onclose = () => {
    state.connected = false;
    clearDynamicVisualization(true, false);
    state.ready.tf = false;
    state.ready.nav2 = false;
    state.ready.scan = false;
    state.ready.localizationReset = false;
    updateMatchCard();
    updateNavigationCard();
    updateHealth();
    setHealth('wsHealth', false, 'Web 已连接', 'Web 已断开', true);
    setTimeout(connectWebSocket, 1600);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = (event) => {
    try {
      handleMessage(JSON.parse(event.data));
    } catch (error) {
      console.error('消息解析失败', error);
    }
  };
}

function send(message) {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    showToast('WebSocket 尚未连接');
    return false;
  }
  state.socket.send(JSON.stringify(message));
  return true;
}

function handleMessage(message) {
  switch (message.type) {
    case 'map':
      loadMap(message);
      break;
    case 'telemetry':
      const previousStage = state.workflow.stage;
      state.pose = message.pose;
      state.cmdVel = message.cmd_vel || state.cmdVel;
      state.goal = message.goal;
      state.nav = message.nav || state.nav;
      state.workflow = message.workflow || state.workflow;
      state.mapping = message.mapping || state.mapping;
      state.localizationReset = message.localization_reset || state.localizationReset;
      if (['idle', 'completed', 'error'].includes(state.localizationReset.state)) {
        state.scanPoints.reset_pending = false;
      }
      state.saveMap = message.save_map || state.saveMap;
      state.mapSaveDirectory = message.map_save_directory || state.mapSaveDirectory;
      state.ready.map = Boolean(message.map_ready);
      state.ready.tf = Boolean(message.tf_ready);
      state.ready.nav2 = Boolean(message.nav2_ready);
      state.ready.scan = Boolean(message.scan_ready);
      state.ready.saveMap = Boolean(message.save_map_ready);
      state.ready.localizationReset = Boolean(message.localization_reset_ready);
      state.topics = message.topics || state.topics;
      state.frames = message.frames || state.frames;
      state.localCostmap = message.local_costmap || state.localCostmap;
      state.inflation = message.inflation || state.inflation;
      if (previousStage !== state.workflow.stage && state.workflow.stage === 'mapping') {
        state.trail = [];
      }
      recordMappingTrail();
      updateTelemetry();
      break;
    case 'path':
      state.path = message.points || [];
      state.dirty = true;
      break;
    case 'mppi_trajectories':
      state.mppiTrajectories = message;
      state.dirty = true;
      break;
    case 'scan_points':
      state.scanPoints = message;
      drawLocalScan();
      updateMatchCard();
      state.dirty = true;
      break;
    case 'particles':
      state.particles = message;
      updateMatchCard();
      state.dirty = true;
      break;
    case 'costmap':
      loadCostmap(message);
      break;
    case 'inflation_status':
      if (state.inflation[message.scope]) {
        state.inflation[message.scope].state = message.state;
        state.inflation[message.scope].message = message.message;
      }
      if (message.state === 'succeeded') clearInflationDirty(message.scope);
      updateInflationCard();
      showToast(message.message || '膨胀参数状态已更新');
      break;
    case 'nav_status':
      state.nav = message;
      updateNavigationCard();
      break;
    case 'save_map_status':
      state.saveMap = message;
      updateMappingCard();
      showToast(message.message || '地图保存状态已更新');
      break;
    case 'visualization_cleared':
      clearDynamicVisualization(false);
      showToast(message.message || '已清除旧显示');
      break;
    case 'localization_reset_status':
      state.localizationReset = {
        state: message.state,
        message: message.message || '定位重置状态已更新',
      };
      clearDynamicVisualization(
        message.state === 'succeeded',
        message.state === 'succeeded',
      );
      updateMatchCard();
      updateActionAvailability();
      if (message.state === 'succeeded'
          && state.ready.map && localizerReady()) setMode('initial');
      showToast(state.localizationReset.message);
      break;
    case 'launch_status':
      state.launchControl = {
        enabled: Boolean(message.enabled),
        profiles: message.profiles || [],
        maps: message.maps || [],
        active: message.active || null,
        logs: message.logs || [],
        map_directory: message.map_directory || '',
      };
      updateLaunchCard();
      break;
    case 'launch_log':
      if (message.entry) {
        const logs = state.launchControl.logs || [];
        if (!logs.some((entry) => entry.seq === message.entry.seq)) {
          logs.push(message.entry);
          if (logs.length > 600) logs.splice(0, logs.length - 600);
        }
        updateLaunchLog();
      }
      break;
    case 'launch_error':
      showToast(message.message || 'Launch 操作失败');
      break;
    case 'notice':
      showToast(message.message);
      break;
    case 'error':
      showToast(message.message || '发生错误');
      break;
    default:
      break;
  }
}

function clearDynamicVisualization(clearPose = false, resetPending = false) {
  state.path = [];
  state.mppiTrajectories = {
    frame_id: state.frames.odom || 'odom',
    target_frame: state.frames.map || 'map',
    transform_ready: false,
    candidate_count: 0,
    optimal_count: 0,
    candidate_points: [],
    optimal_points: [],
  };
  if (!['sending', 'navigating', 'canceling'].includes(state.nav?.state)) {
    state.goal = null;
  }
  state.scanPoints = {
    source_frame: state.frames.scan || 'base_scan',
    target_frame: state.frames.map || 'map',
    local_points: [], map_points: [], transform_ready: false,
    reset_pending: resetPending,
  };
  state.particles = {
    frame_id: state.frames.map || 'map', count: 0, points: [], spread: null,
  };
  if (clearPose) state.pose = null;
  drawLocalScan();
  state.dirty = true;
}

function loadMap(message) {
  const width = Number(message.width);
  const height = Number(message.height);
  const binary = atob(message.data);
  if (!width || !height || binary.length !== width * height) {
    showToast('地图数据尺寸不正确');
    return;
  }

  const imageCanvas = document.createElement('canvas');
  imageCanvas.width = width;
  imageCanvas.height = height;
  const imageContext = imageCanvas.getContext('2d');
  const image = imageContext.createImageData(width, height);
  let minKnownX = width;
  let minKnownY = height;
  let maxKnownX = -1;
  let maxKnownY = -1;

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const sourceIndex = x + y * width;
      const imageY = height - 1 - y;
      const destinationIndex = (x + imageY * width) * 4;
      const occupancy = binary.charCodeAt(sourceIndex) - 1;
      if (occupancy >= 0) {
        minKnownX = Math.min(minKnownX, x);
        minKnownY = Math.min(minKnownY, imageY);
        maxKnownX = Math.max(maxKnownX, x);
        maxKnownY = Math.max(maxKnownY, imageY);
      }
      let color;
      if (occupancy < 0) color = 69;
      else color = Math.round(242 - Math.min(100, occupancy) * 2.18);
      image.data[destinationIndex] = color;
      image.data[destinationIndex + 1] = occupancy < 0 ? 78 : color;
      image.data[destinationIndex + 2] = occupancy < 0 ? 86 : color;
      image.data[destinationIndex + 3] = 255;
    }
  }
  imageContext.putImageData(image, 0, 0);

  const firstMap = !state.map;
  state.map = {
    width,
    height,
    resolution: Number(message.resolution),
    origin: message.origin,
    frameId: message.frame_id,
    knownBounds: maxKnownX >= minKnownX && maxKnownY >= minKnownY
      ? {
        minX: minKnownX, minY: minKnownY,
        maxX: maxKnownX + 1, maxY: maxKnownY + 1,
      }
      : { minX: 0, minY: 0, maxX: width, maxY: height },
  };
  state.mapImage = imageCanvas;
  state.ready.map = true;
  if (firstMap || state.autoFitMap) fitKnownMap();
  $('mapEmpty').classList.add('hidden');
  $('mapMeta').textContent = `${width}×${height} · ${message.resolution.toFixed(3)} m/px`;
  updateHealth();
  state.dirty = true;
}

function loadCostmap(message) {
  const scope = message.scope;
  const width = Number(message.width);
  const height = Number(message.height);
  const binary = atob(message.data || '');
  if (!['global', 'local'].includes(scope)
      || !width || !height || binary.length !== width * height) return;

  const imageCanvas = document.createElement('canvas');
  imageCanvas.width = width;
  imageCanvas.height = height;
  const imageContext = imageCanvas.getContext('2d');
  const image = imageContext.createImageData(width, height);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const sourceIndex = x + y * width;
      const destinationIndex = (x + (height - 1 - y) * width) * 4;
      const cost = binary.charCodeAt(sourceIndex) - 1;
      if (cost <= 0) continue;
      if (cost >= 98) {
        image.data[destinationIndex] = 255;
        image.data[destinationIndex + 1] = 73;
        image.data[destinationIndex + 2] = 93;
        image.data[destinationIndex + 3] = 205;
      } else {
        const ratio = Math.max(0, Math.min(1, cost / 100));
        image.data[destinationIndex] = 255;
        image.data[destinationIndex + 1] = Math.round(205 - ratio * 95);
        image.data[destinationIndex + 2] = 50;
        image.data[destinationIndex + 3] = Math.round(30 + ratio * 125);
      }
    }
  }
  imageContext.putImageData(image, 0, 0);
  state.costmaps[scope] = { ...message, image: imageCanvas };
  state.dirty = true;
}

function updateTelemetry() {
  const pose = state.pose;
  $('poseX').textContent = pose ? pose.x.toFixed(2) : '--';
  $('poseY').textContent = pose ? pose.y.toFixed(2) : '--';
  $('poseYaw').textContent = pose ? radiansToDegrees(pose.yaw).toFixed(1) : '--';

  const cmd = state.cmdVel || {};
  $('cmdVx').textContent = finite(cmd.vx).toFixed(3);
  $('cmdVy').textContent = finite(cmd.vy).toFixed(3);
  $('cmdWz').textContent = finite(cmd.wz).toFixed(3);
  updateMeter('meterVx', finite(cmd.vx), 1.0);
  updateMeter('meterVy', finite(cmd.vy), 1.0);
  updateMeter('meterWz', finite(cmd.wz), 1.0);
  $('cmdTopic').textContent = state.topics.cmd_vel || '/cmd_vel';

  const age = cmd.age;
  if (age === null || age === undefined) {
    $('cmdAge').textContent = '尚未收到速度';
  } else if (age < 0.5) {
    $('cmdAge').textContent = `实时 · ${(age * 1000).toFixed(0)} ms`;
  } else {
    $('cmdAge').textContent = `最后消息 · ${age.toFixed(1)} s 前`;
  }

  $('frameMeta').textContent = `${state.frames.map} / ${state.frames.base}`;
  updateWorkflow();
  updateMappingCard();
  updateMatchCard();
  updateInflationCard();
  updateNavigationCard();
  updateHealth();
  state.dirty = true;
}

function updateNavigationCard() {
  const nav = state.nav || {};
  const labels = {
    idle: '空闲', sending: '提交中', navigating: '导航中',
    canceling: '取消中', canceled: '已取消', succeeded: '已到达',
    aborted: '失败', rejected: '已拒绝', unavailable: '未就绪',
    error: '异常', finished: '已结束',
  };
  $('navStateText').textContent = labels[nav.state] || nav.state || '空闲';
  $('navMessage').textContent = nav.message || '等待地图目标';
  $('distanceRemaining').textContent = numberOrDash(nav.distance_remaining, 2);
  $('navigationTime').textContent = numberOrDash(nav.navigation_time, 1);
  $('cancelNavigation').disabled = !['sending', 'navigating', 'canceling'].includes(nav.state);

  if (state.goal) {
    $('goalPose').textContent = `x ${state.goal.x.toFixed(2)} · y ${state.goal.y.toFixed(2)} · ${radiansToDegrees(state.goal.yaw).toFixed(0)}°`;
  } else {
    $('goalPose').textContent = '尚未设置';
  }
}

function updateHealth() {
  const stageOrder = { mapping: 1, localization: 2, planning: 3 };
  const activeOrder = stageOrder[state.workflow.stage] || 0;
  [
    ['stageMapping', 1],
    ['stageLocalization', 2],
    ['stagePlanning', 3],
  ].forEach(([id, order]) => {
    const element = $(id);
    element.classList.toggle('active', order === activeOrder);
    element.classList.toggle('complete', activeOrder > order);
  });
  document.body.dataset.stage = state.workflow.stage || 'waiting';
  updateActionAvailability();
}

function setReadyIndicator(id, ready) {
  const element = $(id);
  element.classList.toggle('ready', Boolean(ready));
}

function updateWorkflow() {
  const workflow = state.workflow || {};
  const graphMode = isGraphLocalization();
  const labels = {
    waiting: '等待启动', mapping: '1 / 建图',
    localization: graphMode ? '2 / 图 SLAM 定位' : '2 / AMCL 重定位',
    planning: '3 / 轨迹规划',
  };
  $('stageLocalization').querySelector('span').textContent = graphMode
    ? '图 SLAM 定位' : 'AMCL 重定位';
  $('amclReady').textContent = graphMode ? '● 图 SLAM' : '● AMCL';
  $('workflowStage').textContent = labels[workflow.stage] || workflow.stage || '等待启动';
  $('workflowTitle').textContent = workflow.title || '等待 ROS 流程';
  $('workflowMessage').textContent = workflow.message || '';
  setReadyIndicator('slamReady', workflow.slam_ready);
  setReadyIndicator('scanReady', state.ready.scan);
  setReadyIndicator('amclReady', localizerReady());
  setReadyIndicator('plannerReady', workflow.planner_ready);
  updateLocalizationSelector();
}

function updateLocalizationSelector() {
  const active = activeLocalizationType();
  const selected = state.localizationChoice || active;
  document.querySelectorAll('[data-localizer]').forEach((button) => {
    const method = button.dataset.localizer;
    const isSelected = method === selected;
    button.classList.toggle('selected', isSelected);
    button.classList.toggle('running', method === active);
    button.classList.toggle('mismatch', isSelected && active && method !== active);
    button.setAttribute('aria-checked', String(isSelected));
  });

  const hint = $('localizationSelectorHint');
  hint.classList.remove('warning');
  if (selected && active && selected === active) {
    hint.textContent = active === 'cartographer'
      ? '当前运行图 SLAM 定位；使用冻结的 dog_map.pbstream'
      : '当前运行 AMCL；使用 dog_map.yaml/.pgm 与里程计';
  } else if (selected) {
    hint.classList.add('warning');
    hint.textContent = selected === 'cartographer'
      ? '已选择图 SLAM：停止当前 launch，再启动 cartographer_mppi_navigation.launch.py'
      : '已选择 AMCL：停止当前 launch，再启动 nav2_mppi_navigation.launch.py';
  } else {
    hint.textContent = '导航前选择定位方法；切换方法需要停止当前 launch';
  }
}

function selectedLaunchProfile() {
  const profileId = $('launchProfile').value;
  return (state.launchControl.profiles || []).find(
    (profile) => profile.id === profileId,
  ) || null;
}

function compatibleLaunchMaps(profile) {
  const maps = state.launchControl.maps || [];
  if (!profile?.requires_pbstream) return maps;
  return maps.filter((map) => map.has_pbstream);
}

function syncLaunchOptions() {
  const profileSelect = $('launchProfile');
  const previousProfile = profileSelect.value;
  const profiles = state.launchControl.profiles || [];
  profileSelect.replaceChildren();
  if (profiles.length === 0) {
    profileSelect.add(new Option(
      state.launchControl.enabled ? '没有可用 Launch' : 'Launch 控制未启用',
      '',
    ));
  } else {
    profiles.forEach((profile) => {
      profileSelect.add(new Option(profile.label, profile.id));
    });
    const preferred = profiles.some((profile) => profile.id === previousProfile)
      ? previousProfile
      : state.launchControl.active?.profile_id || profiles[0].id;
    profileSelect.value = preferred;
  }

  const profile = selectedLaunchProfile();
  const mapSelect = $('launchMap');
  const previousMap = mapSelect.value;
  const maps = compatibleLaunchMaps(profile);
  mapSelect.replaceChildren();
  if (maps.length === 0) {
    mapSelect.add(new Option(
      profile?.requires_pbstream
        ? '没有 YAML + 同名 PBSTREAM' : 'maps 目录中没有 YAML 地图',
      '',
    ));
  } else {
    maps.forEach((map) => {
      const suffix = map.has_pbstream ? ' + PBSTREAM' : '';
      mapSelect.add(new Option(`${map.label}${suffix}`, map.name));
    });
    mapSelect.value = maps.some((map) => map.name === previousMap)
      ? previousMap : maps[0].name;
  }
  $('launchMapRow').classList.toggle('hidden', !profile?.requires_map);
}

function updateLaunchLog() {
  const panel = $('launchLog');
  const wasAtBottom = panel.scrollHeight - panel.scrollTop - panel.clientHeight < 28;
  const logs = (state.launchControl.logs || []).slice(-600);
  panel.textContent = logs.length > 0
    ? logs.map((entry) => {
      const marker = entry.level === 'error'
        ? '!' : entry.level === 'system' ? '◆' : '·';
      return `[${entry.time || '--:--:--'}] ${marker} ${entry.line || ''}`;
    }).join('\n')
    : '等待 Launch 输出…';
  $('launchLogCount').textContent = `${logs.length} 行`;
  if (wasAtBottom || logs.length <= 2) panel.scrollTop = panel.scrollHeight;
}

function updateLaunchCard() {
  syncLaunchOptions();
  const control = state.launchControl;
  const active = control.active;
  const profile = selectedLaunchProfile();
  const maps = compatibleLaunchMaps(profile);
  const stateLabels = {
    idle: '未启动', starting: '启动中', running: '运行中',
    stopping: '停止中', exited: '已退出', error: '启动失败',
  };
  const launchState = active?.state || 'idle';
  $('launchCard').dataset.state = control.enabled ? launchState : 'disabled';
  $('launchState').textContent = control.enabled
    ? stateLabels[launchState] || launchState : '服务端未启用';
  $('launchActiveName').textContent = active?.label || '当前未启动流程';
  if (active) {
    const details = [];
    if (active.pid) details.push(`PID ${active.pid}`);
    if (active.map_name) details.push(active.map_name);
    if (active.exit_code !== null && active.exit_code !== undefined) {
      details.push(`退出码 ${active.exit_code}`);
    }
    $('launchActiveDetail').textContent = details.join(' · ') || '等待进程状态';
  } else {
    $('launchActiveDetail').textContent = 'Web Bridge 会保持独立运行';
  }
  $('launchProfileDescription').textContent = profile?.description
    || (control.enabled
      ? '请选择白名单中的启动入口'
      : '用 enable_launch_control:=True 启动 Web Bridge 才能使用');

  const busy = Boolean(active?.running)
    || ['starting', 'stopping'].includes(active?.state);
  const missingMap = Boolean(profile?.requires_map) && maps.length === 0;
  $('launchProfile').disabled = !control.enabled || busy;
  $('launchMap').disabled = !control.enabled || busy || maps.length === 0;
  $('launchStart').disabled = !control.enabled || busy || !profile || missingMap;
  $('launchStop').disabled = !control.enabled || !active?.running
    || active?.state === 'stopping';
  $('launchClearLogs').disabled = (control.logs || []).length === 0;
  $('launchSecurityHint').textContent = control.enabled
    ? `白名单控制 · 地图目录 ${control.map_directory || 'maps'}`
    : 'Launch 控制默认关闭；开启后仅用于可信局域网。';
  updateLaunchLog();
}

function updateMappingCard() {
  const mapping = state.mapping || {};
  const map = mapping.map || {};
  const scan = mapping.scan || {};
  const labels = {
    inactive: '未开始', blocked: '数据中断', starting: '启动中',
    warning: '需检查', stale: '地图停更', changing: '变化较大',
    exploring: '探索中', stable: '更新稳定',
  };
  const mappingState = $('mappingState');
  mappingState.textContent = labels[mapping.state] || mapping.state || '未开始';
  mappingState.dataset.state = mapping.state || 'inactive';
  $('mappingAdvice').textContent = mapping.message || '等待 SLAM Toolbox';
  const score = Number(mapping.health_score);
  $('mappingScore').querySelector('strong').textContent = Number.isFinite(score) ? `${score}` : '--';
  $('mappingScore').style.setProperty('--score', Number.isFinite(score) ? score : 0);

  $('knownArea').textContent = numberOrDash(map.known_area, 1);
  $('knownRatio').textContent = numberOrDash(map.known_ratio, 1);
  $('mapStability').textContent = numberOrDash(map.stability, 1);
  $('scanValid').textContent = numberOrDash(scan.valid_ratio, 1);
  $('mapFreshness').textContent = map.age === null || map.age === undefined
    ? '地图尚未更新' : `地图 ${formatAge(map.age)}`;
  $('mapGrowth').textContent = Number.isFinite(Number(map.known_delta_area))
    ? `增量 ${Number(map.known_delta_area) >= 0 ? '+' : ''}${Number(map.known_delta_area).toFixed(2)} m²`
    : '增量 --';
  $('mapUpdates').textContent = `更新 ${Number(map.update_count) || 0} 次`;

  const checks = mapping.checks || {};
  setReadyIndicator('checkSlam', checks.slam);
  setReadyIndicator('checkScan', checks.scan);
  setReadyIndicator('checkTf', checks.tf);
  setReadyIndicator('checkMap', checks.map);

  const saving = state.saveMap && state.saveMap.state === 'saving';
  $('saveMap').disabled = !state.ready.saveMap || !state.ready.map || saving;
  $('saveMap').textContent = saving ? '保存中…' : '保存地图';
  let saveHint = state.saveMap?.message || '尚未保存地图';
  if (!state.ready.saveMap) saveHint = '等待 /map_saver/save_map';
  else if (state.saveMap?.state === 'idle' && state.mapSaveDirectory) {
    saveHint = `保存到 ${state.mapSaveDirectory}`;
  }
  $('saveMapHint').textContent = saveHint;
}

function updateActionAvailability() {
  const goalButton = document.querySelector('[data-mode="goal"]');
  const initialButton = document.querySelector('[data-mode="initial"]');
  const selectedBackendReady = localizationSelectionMatchesBackend();
  goalButton.disabled = !(
    state.ready.map && state.ready.tf && state.ready.nav2 && localizerReady()
  )
    || !selectedBackendReady
    || state.workflow.stage === 'mapping';
  initialButton.disabled = !(state.ready.map && localizerReady())
    || !selectedBackendReady;
  const resetState = state.localizationReset?.state;
  const navActive = ['sending', 'navigating', 'canceling'].includes(state.nav?.state);
  const resetPending = ['resetting', 'succeeded', 'initializing'].includes(resetState);
  $('resetLocalization').disabled = !state.ready.localizationReset
    || !localizerReady() || !selectedBackendReady || navActive || resetPending;
  if ((state.mode === 'goal' && goalButton.disabled)
      || (state.mode === 'initial' && initialButton.disabled)) {
    setMode('view');
  }
}

function updateMatchCard() {
  const graphMode = isGraphLocalization();
  const scan = state.scanPoints || {};
  const particles = state.particles || {};
  const count = Number(particles.count) || 0;
  const spread = particles.spread === null || particles.spread === undefined
    ? Number.NaN : Number(particles.spread);
  $('scanPointCount').textContent = (scan.local_points || []).length;
  $('particleCount').textContent = count;
  $('particleSpread').textContent = Number.isFinite(spread) ? spread.toFixed(2) : '--';
  $('globalFrame').textContent = state.frames.map || 'map';
  $('localFrame').textContent = state.frames.local_costmap || state.frames.odom || 'odom';
  $('baseFrame').textContent = state.frames.base || 'base_link';
  $('scanFrame').textContent = scan.source_frame || state.frames.scan || 'base_scan';
  $('particleFrame').textContent = particles.frame_id || state.frames.map || 'map';

  const resetState = state.localizationReset || {};
  const resetPending = Boolean(scan.reset_pending)
    || ['resetting', 'succeeded', 'initializing'].includes(resetState.state);
  let status = graphMode ? '等待图 SLAM 初始位置' : '等待 AMCL 初始位置';
  if (resetPending) {
    status = resetState.message || '定位器已重置，请重新设置初始位置';
  } else if (!localizerReady() && state.workflow.stage === 'mapping') {
    status = scan.transform_ready
      ? '建图模式：激光已投影到 map'
      : '建图模式：等待激光 TF';
  } else if (!scan.transform_ready) {
    status = `等待 ${state.frames.map || 'map'} → ${scan.source_frame || 'base_scan'} TF`;
  } else if (graphMode) {
    status = '图 SLAM 激光匹配与定位 TF 已就绪';
  } else if (state.workflow.stage === 'planning' && count === 0) {
    status = '定位 TF 已就绪，等待粒子云更新';
  } else if (count > 0 && Number.isFinite(spread) && spread <= 0.5) {
    status = 'AMCL 粒子已收敛';
  } else if (count > 0 && Number.isFinite(spread)) {
    status = spread > 1.0 ? '粒子较分散，继续慢速旋转' : 'AMCL 正在收敛';
  }
  $('matchState').textContent = status;
  $('matchHint').textContent = resetPending
    ? '全局匹配点已暂停显示；请点击“初始位置”，在地图上设置真实位置和朝向'
    : scan.transform_ready
    ? `局部 ${scan.source_frame || 'base_scan'} 点已变换到 ${scan.target_frame || 'map'}；红点与黑色墙体越重合，定位越准`
    : '局部激光正常，获得 map TF 后才能显示全局匹配点';
  $('resetLocalization').textContent = resetState.state === 'resetting'
    ? '重置中…' : resetPending ? '等待初始位置' : '重置定位';
  $('resetHint').textContent = !state.ready.localizationReset
    ? '等待 /reinitialize_global_localization 服务'
    : ['sending', 'navigating', 'canceling'].includes(state.nav?.state)
      ? '导航中不可重置，请先取消导航'
      : resetPending
        ? '旧定位轨迹/状态已清除，请重新设置初始位置'
        : '先清除显示判断是否为网页缓存；仍错位再重置定位';
  updateActionAvailability();
  drawLocalScan();
}

function inflationIds(scope) {
  const prefix = scope === 'global' ? 'global' : 'local';
  return {
    enabled: `${prefix}Enabled`,
    radius: `${prefix}Radius`,
    scaling: `${prefix}Scaling`,
    inflateUnknown: `${prefix}InflateUnknown`,
    inflateAroundUnknown: `${prefix}InflateAroundUnknown`,
    apply: scope === 'global' ? 'applyGlobalInflation' : 'applyLocalInflation',
  };
}

function syncInflationInput(id, value, isCheckbox = false) {
  const input = $(id);
  if (value === null || value === undefined || input.dataset.dirty === 'true') return;
  if (isCheckbox) input.checked = Boolean(value);
  else input.value = Number(value).toFixed(id.includes('Radius') ? 2 : 1);
}

function updateInflationCard() {
  let readyCount = 0;
  let setting = false;
  ['global', 'local'].forEach((scope) => {
    const config = state.inflation[scope] || {};
    const ids = inflationIds(scope);
    syncInflationInput(ids.enabled, config.enabled, true);
    syncInflationInput(ids.radius, config.inflation_radius);
    syncInflationInput(ids.scaling, config.cost_scaling_factor);
    syncInflationInput(ids.inflateUnknown, config.inflate_unknown, true);
    syncInflationInput(ids.inflateAroundUnknown, config.inflate_around_unknown, true);
    if (config.ready) readyCount += 1;
    if (config.state === 'setting') setting = true;
    $(ids.apply).disabled = !config.ready || config.state === 'setting';
  });
  if (setting) $('inflationState').textContent = '正在应用参数';
  else if (readyCount === 2) $('inflationState').textContent = '全局/局部参数已同步';
  else if (readyCount === 1) $('inflationState').textContent = '部分 Costmap 已就绪';
  else $('inflationState').textContent = '等待 Costmap 参数服务';
}

function clearInflationDirty(scope) {
  const ids = inflationIds(scope);
  [ids.enabled, ids.radius, ids.scaling, ids.inflateUnknown, ids.inflateAroundUnknown]
    .forEach((id) => { $(id).dataset.dirty = 'false'; });
}

function applyInflation(scope) {
  const ids = inflationIds(scope);
  const inflationRadius = Number($(ids.radius).value);
  const costScalingFactor = Number($(ids.scaling).value);
  if (!Number.isFinite(inflationRadius) || inflationRadius < 0 || inflationRadius > 10) {
    showToast('膨胀半径必须在 0–10 m 之间');
    return;
  }
  if (!Number.isFinite(costScalingFactor)
      || costScalingFactor < 0.01 || costScalingFactor > 100) {
    showToast('代价衰减系数必须在 0.01–100 之间');
    return;
  }
  if (send({
    type: 'set_inflation',
    scope,
    enabled: $(ids.enabled).checked,
    inflation_radius: inflationRadius,
    cost_scaling_factor: costScalingFactor,
    inflate_unknown: $(ids.inflateUnknown).checked,
    inflate_around_unknown: $(ids.inflateAroundUnknown).checked,
  })) {
    state.inflation[scope].state = 'setting';
    state.inflation[scope].message = '正在应用膨胀参数';
    updateInflationCard();
  }
}

function drawLocalScan() {
  const width = localScanCanvas.width;
  const height = localScanCanvas.height;
  const centerX = width / 2;
  const centerY = height * 0.72;
  const points = state.scanPoints?.local_points || [];
  const maxRange = 4.0;
  const scale = Math.min(width * 0.42, height * 0.62) / maxRange;
  localScanContext.clearRect(0, 0, width, height);
  localScanContext.fillStyle = '#07111a';
  localScanContext.fillRect(0, 0, width, height);

  localScanContext.strokeStyle = 'rgba(148, 184, 208, 0.12)';
  localScanContext.lineWidth = 1;
  [1, 2, 3, 4].forEach((range) => {
    localScanContext.beginPath();
    localScanContext.arc(centerX, centerY, range * scale, Math.PI, Math.PI * 2);
    localScanContext.stroke();
  });
  localScanContext.beginPath();
  localScanContext.moveTo(centerX, 7);
  localScanContext.lineTo(centerX, centerY);
  localScanContext.stroke();

  localScanContext.fillStyle = 'rgba(255, 101, 119, 0.9)';
  points.forEach(([x, y]) => {
    if (Math.hypot(x, y) > maxRange) return;
    const screenX = centerX - y * scale;
    const screenY = centerY - x * scale;
    localScanContext.fillRect(screenX - 1, screenY - 1, 2, 2);
  });
  localScanContext.fillStyle = '#26d9e8';
  localScanContext.beginPath();
  localScanContext.moveTo(centerX, centerY - 8);
  localScanContext.lineTo(centerX - 6, centerY + 5);
  localScanContext.lineTo(centerX + 6, centerY + 5);
  localScanContext.closePath();
  localScanContext.fill();
  localScanContext.fillStyle = '#668296';
  localScanContext.font = '9px ui-monospace, monospace';
  localScanContext.fillText(state.scanPoints?.source_frame || 'base_scan', 8, 14);
}

function formatAge(seconds) {
  const value = Number(seconds);
  if (!Number.isFinite(value)) return '未知';
  if (value < 1) return `${Math.round(value * 1000)} ms 前`;
  return `${value.toFixed(1)} s 前`;
}

function recordMappingTrail() {
  if (state.workflow.stage !== 'mapping' || !state.pose) return;
  const last = state.trail[state.trail.length - 1];
  if (!last || Math.hypot(state.pose.x - last[0], state.pose.y - last[1]) >= 0.05) {
    state.trail.push([state.pose.x, state.pose.y]);
    if (state.trail.length > 2400) state.trail.splice(0, 400);
  }
}

function updateMeter(id, value, limit) {
  const meter = $(id);
  const ratio = Math.max(-1, Math.min(1, value / limit));
  meter.style.width = `${Math.abs(ratio) * 50}%`;
  meter.style.transform = ratio < 0 ? 'translateX(-100%)' : 'translateX(0)';
}

function numberOrDash(value, digits) {
  if (value === null || value === undefined || value === '') return '--';
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : '--';
}

function finite(value) {
  return Number.isFinite(Number(value)) ? Number(value) : 0;
}

function radiansToDegrees(value) {
  return value * 180 / Math.PI;
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  state.dirty = true;
}

function canvasMetrics() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = canvas.width / dpr;
  const height = canvas.height / dpr;
  if (!state.map) return { dpr, width, height, scale: 1 };
  const fit = Math.min(width / state.map.width, height / state.map.height) * 0.98;
  return { dpr, width, height, scale: fit * state.view.zoom };
}

function fitKnownMap() {
  if (!state.map) return;
  state.autoFitMap = true;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = canvas.width / dpr;
  const height = canvas.height / dpr;
  const bounds = state.map.knownBounds
    || { minX: 0, minY: 0, maxX: state.map.width, maxY: state.map.height };
  const contentWidth = Math.max(1, bounds.maxX - bounds.minX);
  const contentHeight = Math.max(1, bounds.maxY - bounds.minY);
  const padding = Math.max(8, Math.min(28, Math.max(contentWidth, contentHeight) * 0.08));
  const fittedWidth = contentWidth + padding * 2;
  const fittedHeight = contentHeight + padding * 2;
  const baseScale = Math.min(
    width / state.map.width,
    height / state.map.height,
  ) * 0.98;
  const zoom = Math.max(0.35, Math.min(
    8,
    Math.min(width / (fittedWidth * baseScale), height / (fittedHeight * baseScale)),
  ));
  const centerX = (bounds.minX + bounds.maxX) / 2;
  const centerY = (bounds.minY + bounds.maxY) / 2;
  const scale = baseScale * zoom;
  state.view = {
    zoom,
    panX: -(centerX - state.map.width / 2) * scale,
    panY: -(centerY - state.map.height / 2) * scale,
  };
  state.dirty = true;
}

function setMapExpanded(expanded) {
  state.mapExpanded = Boolean(expanded);
  document.body.classList.toggle('map-expanded', state.mapExpanded);
  $('toggleMapSize').textContent = state.mapExpanded ? '恢复面板' : '大地图';
  $('toggleMapSize').setAttribute('aria-pressed', String(state.mapExpanded));
  $('toggleMapSize').setAttribute(
    'aria-label', state.mapExpanded ? '恢复状态面板' : '最大化地图',
  );
  requestAnimationFrame(() => {
    resizeCanvas();
    fitKnownMap();
  });
}

function worldToMapPixel(x, y) {
  if (!state.map) return null;
  const origin = state.map.origin;
  const dx = x - origin.x;
  const dy = y - origin.y;
  const c = Math.cos(origin.yaw);
  const s = Math.sin(origin.yaw);
  const localX = c * dx + s * dy;
  const localY = -s * dx + c * dy;
  return {
    x: localX / state.map.resolution,
    y: state.map.height - localY / state.map.resolution,
  };
}

function mapPixelToWorld(x, y) {
  if (!state.map) return null;
  const localX = x * state.map.resolution;
  const localY = (state.map.height - y) * state.map.resolution;
  const origin = state.map.origin;
  const c = Math.cos(origin.yaw);
  const s = Math.sin(origin.yaw);
  return {
    x: origin.x + c * localX - s * localY,
    y: origin.y + s * localX + c * localY,
  };
}

function mapToScreen(point, metrics = canvasMetrics()) {
  if (!state.map || !point) return null;
  return {
    x: metrics.width / 2 + (point.x - state.map.width / 2) * metrics.scale + state.view.panX,
    y: metrics.height / 2 + (point.y - state.map.height / 2) * metrics.scale + state.view.panY,
  };
}

function screenToWorld(x, y) {
  if (!state.map) return null;
  const metrics = canvasMetrics();
  const mapX = (x - metrics.width / 2 - state.view.panX) / metrics.scale + state.map.width / 2;
  const mapY = (y - metrics.height / 2 - state.view.panY) / metrics.scale + state.map.height / 2;
  return mapPixelToWorld(mapX, mapY);
}

function renderLoop() {
  if (state.dirty) {
    draw();
    state.dirty = false;
  }
  requestAnimationFrame(renderLoop);
}

function draw() {
  const metrics = canvasMetrics();
  ctx.setTransform(metrics.dpr, 0, 0, metrics.dpr, 0, 0);
  ctx.clearRect(0, 0, metrics.width, metrics.height);
  ctx.fillStyle = '#07111a';
  ctx.fillRect(0, 0, metrics.width, metrics.height);
  drawBackdrop(metrics);
  if (!state.map || !state.mapImage) return;

  const topLeft = mapToScreen({ x: 0, y: 0 }, metrics);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(
    state.mapImage,
    topLeft.x,
    topLeft.y,
    state.map.width * metrics.scale,
    state.map.height * metrics.scale,
  );

  drawMapBorder(metrics);
  drawCostmap('global', metrics);
  drawCostmap('local', metrics);
  drawMppiCandidates(metrics);
  drawGlobalScan(metrics);
  drawParticles(metrics);
  drawMappingTrail(metrics);
  drawGlobalPath(metrics);
  drawMppiOptimal(metrics);
  drawGoal(metrics);
  drawRobot(metrics);
  drawInteractionPreview(metrics);
}

function drawBackdrop(metrics) {
  ctx.strokeStyle = 'rgba(64, 137, 160, 0.07)';
  ctx.lineWidth = 1;
  const spacing = 32;
  for (let x = 0; x < metrics.width; x += spacing) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, metrics.height); ctx.stroke();
  }
  for (let y = 0; y < metrics.height; y += spacing) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(metrics.width, y); ctx.stroke();
  }
}

function drawMapBorder(metrics) {
  const topLeft = mapToScreen({ x: 0, y: 0 }, metrics);
  ctx.strokeStyle = 'rgba(107, 220, 232, 0.24)';
  ctx.lineWidth = 1;
  ctx.strokeRect(topLeft.x, topLeft.y, state.map.width * metrics.scale, state.map.height * metrics.scale);
}

function drawGlobalPath(metrics) {
  if (!state.overlays.globalPath || !state.path || state.path.length < 2) return;
  ctx.strokeStyle = '#39dfa0';
  ctx.lineWidth = 2.5;
  ctx.shadowColor = 'rgba(57, 223, 160, 0.5)';
  ctx.shadowBlur = 6;
  ctx.beginPath();
  state.path.forEach(([x, y], index) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function drawMppiCandidates(metrics) {
  const trajectories = state.mppiTrajectories || {};
  const points = trajectories.candidate_points || [];
  if (!state.overlays.mppiTrajectories
      || !trajectories.transform_ready || points.length === 0) return;
  ctx.save();
  ctx.fillStyle = 'rgba(50, 205, 235, 0.22)';
  points.forEach(([x, y]) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    ctx.fillRect(point.x - 1, point.y - 1, 2, 2);
  });
  ctx.restore();
}

function drawMppiOptimal(metrics) {
  const trajectories = state.mppiTrajectories || {};
  const points = trajectories.optimal_points || [];
  if (!state.overlays.mppiTrajectories
      || !trajectories.transform_ready || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = '#ff79bd';
  ctx.lineWidth = 3;
  ctx.shadowColor = 'rgba(255, 121, 189, 0.72)';
  ctx.shadowBlur = 7;
  ctx.beginPath();
  points.forEach(([x, y], index) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawMappingTrail(metrics) {
  if (state.workflow.stage !== 'mapping' || !state.trail || state.trail.length < 2) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(255, 180, 84, 0.72)';
  ctx.lineWidth = 1.6;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  state.trail.forEach(([x, y], index) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawGlobalScan(metrics) {
  const scan = state.scanPoints || {};
  if (!state.overlays.scan || !scan.transform_ready || !scan.map_points) return;
  ctx.save();
  ctx.fillStyle = 'rgba(255, 74, 101, 0.9)';
  ctx.shadowColor = 'rgba(255, 74, 101, 0.55)';
  ctx.shadowBlur = 3;
  scan.map_points.forEach(([x, y]) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    ctx.beginPath();
    ctx.arc(point.x, point.y, 1.7, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.restore();
}

function drawCostmap(scope, metrics) {
  const costmap = state.costmaps[scope];
  const visible = scope === 'global'
    ? state.overlays.globalCostmap : state.overlays.localCostmap;
  if (!visible || !costmap || !costmap.image || !costmap.transform_ready) return;

  const pose = costmap.pose;
  const widthMeters = costmap.width * costmap.resolution;
  const heightMeters = costmap.height * costmap.resolution;
  const cosine = Math.cos(pose.yaw);
  const sine = Math.sin(pose.yaw);
  const toWorld = (x, y) => ({
    x: pose.x + cosine * x - sine * y,
    y: pose.y + sine * x + cosine * y,
  });
  const topLeft = mapToScreen(worldToMapPixel(
    toWorld(0, heightMeters).x,
    toWorld(0, heightMeters).y,
  ), metrics);
  const topRightWorld = toWorld(widthMeters, heightMeters);
  const topRight = mapToScreen(worldToMapPixel(topRightWorld.x, topRightWorld.y), metrics);
  const bottomLeftWorld = toWorld(0, 0);
  const bottomLeft = mapToScreen(worldToMapPixel(bottomLeftWorld.x, bottomLeftWorld.y), metrics);
  const a = (topRight.x - topLeft.x) / costmap.width;
  const b = (topRight.y - topLeft.y) / costmap.width;
  const c = (bottomLeft.x - topLeft.x) / costmap.height;
  const d = (bottomLeft.y - topLeft.y) / costmap.height;

  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.globalAlpha = scope === 'global' ? 0.72 : 0.92;
  ctx.transform(a, b, c, d, topLeft.x, topLeft.y);
  ctx.drawImage(costmap.image, 0, 0);
  ctx.restore();
}

function drawParticles(metrics) {
  const particles = state.particles || {};
  if (!state.overlays.particles || !particles.points || state.workflow.stage === 'mapping') return;
  ctx.save();
  ctx.fillStyle = 'rgba(169, 139, 255, 0.42)';
  particles.points.forEach(([x, y]) => {
    const point = mapToScreen(worldToMapPixel(x, y), metrics);
    ctx.beginPath();
    ctx.arc(point.x, point.y, 2, 0, Math.PI * 2);
    ctx.fill();
  });
  if (particles.mean && Number.isFinite(Number(particles.spread))) {
    const mean = mapToScreen(
      worldToMapPixel(particles.mean[0], particles.mean[1]), metrics,
    );
    const radius = Math.max(
      7,
      Number(particles.spread) / state.map.resolution * metrics.scale,
    );
    ctx.strokeStyle = 'rgba(169, 139, 255, 0.9)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(mean.x, mean.y, radius, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = '#d7c9ff';
    ctx.beginPath();
    ctx.arc(mean.x, mean.y, 3, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

function drawRobot(metrics) {
  if (!state.pose) return;
  const point = mapToScreen(worldToMapPixel(state.pose.x, state.pose.y), metrics);
  const size = 13;
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-state.pose.yaw);
  ctx.fillStyle = '#26d9e8';
  ctx.strokeStyle = '#d7fcff';
  ctx.lineWidth = 1.5;
  ctx.shadowColor = 'rgba(38, 217, 232, 0.7)';
  ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.72, -size * 0.7);
  ctx.lineTo(-size * 0.4, 0);
  ctx.lineTo(-size * 0.72, size * 0.7);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawGoal(metrics) {
  if (state.workflow.stage !== 'planning' || !state.goal) return;
  drawPoseMarker(state.goal, metrics, '#a98bff', false);
}

function drawPoseMarker(pose, metrics, color, dashed) {
  const point = mapToScreen(worldToMapPixel(pose.x, pose.y), metrics);
  const length = 28;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.shadowColor = color;
  ctx.shadowBlur = 8;
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.arc(point.x, point.y, 9, 0, Math.PI * 2);
  ctx.stroke();
  const endX = point.x + Math.cos(-pose.yaw) * length;
  const endY = point.y + Math.sin(-pose.yaw) * length;
  ctx.beginPath(); ctx.moveTo(point.x, point.y); ctx.lineTo(endX, endY); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(endX - Math.cos(-pose.yaw - 0.55) * 8, endY - Math.sin(-pose.yaw - 0.55) * 8);
  ctx.lineTo(endX - Math.cos(-pose.yaw + 0.55) * 8, endY - Math.sin(-pose.yaw + 0.55) * 8);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

function drawInteractionPreview(metrics) {
  if (!state.pointerStart || !state.pointerCurrent || state.mode === 'view') return;
  const dx = state.pointerCurrent.x - state.pointerStart.x;
  const dy = state.pointerCurrent.y - state.pointerStart.y;
  const yaw = Math.atan2(-dy, dx);
  drawPoseMarker(
    { x: state.pointerStart.world.x, y: state.pointerStart.world.y, yaw },
    metrics,
    state.mode === 'goal' ? '#a98bff' : '#ffb454',
    true,
  );
}

function setMode(mode) {
  state.mode = mode;
  document.querySelectorAll('[data-mode]').forEach((button) => {
    button.classList.toggle('active', button.dataset.mode === mode);
  });
  canvas.classList.toggle('action-mode', mode !== 'view');
  const hints = {
    view: '单指拖动地图，双指或按钮缩放',
    goal: '在地图按下目标位置，拖动箭头设置朝向',
    initial: '在地图按下机器人位置，拖动箭头设置初始朝向',
  };
  $('interactionHint').textContent = hints[mode];
  state.pointerStart = null;
  state.pointerCurrent = null;
  state.dragLast = null;
  activeViewPointers.clear();
  pinchGesture = null;
  state.dirty = true;
}

function eventPoint(event) {
  const rect = canvas.getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

function beginPinchGesture() {
  const entries = [...activeViewPointers.entries()].slice(0, 2);
  if (entries.length < 2) return;
  const [[firstId, first], [secondId, second]] = entries;
  const center = {
    x: (first.x + second.x) / 2,
    y: (first.y + second.y) / 2,
  };
  pinchGesture = {
    pointerIds: [firstId, secondId],
    distance: Math.max(1, Math.hypot(second.x - first.x, second.y - first.y)),
    zoom: state.view.zoom,
    panX: state.view.panX,
    panY: state.view.panY,
    center,
    anchorWorld: screenToWorld(center.x, center.y),
  };
  state.dragLast = null;
}

function updatePinchGesture() {
  if (!pinchGesture) return;
  const [firstId, secondId] = pinchGesture.pointerIds;
  const first = activeViewPointers.get(firstId);
  const second = activeViewPointers.get(secondId);
  if (!first || !second) return;
  const center = {
    x: (first.x + second.x) / 2,
    y: (first.y + second.y) / 2,
  };
  const distance = Math.max(1, Math.hypot(second.x - first.x, second.y - first.y));
  state.autoFitMap = false;
  state.view.zoom = Math.max(
    0.35,
    Math.min(8, pinchGesture.zoom * distance / pinchGesture.distance),
  );
  state.view.panX = pinchGesture.panX + center.x - pinchGesture.center.x;
  state.view.panY = pinchGesture.panY + center.y - pinchGesture.center.y;

  if (pinchGesture.anchorWorld && state.map) {
    const anchorScreen = mapToScreen(worldToMapPixel(
      pinchGesture.anchorWorld.x,
      pinchGesture.anchorWorld.y,
    ));
    state.view.panX += center.x - anchorScreen.x;
    state.view.panY += center.y - anchorScreen.y;
  }
  state.dirty = true;
}

function finishViewPointer(pointerId) {
  activeViewPointers.delete(pointerId);
  pinchGesture = null;
  const remaining = [...activeViewPointers.values()];
  state.dragLast = remaining.length === 1 ? remaining[0] : null;
  if (remaining.length >= 2) beginPinchGesture();
}

canvas.addEventListener('pointerdown', (event) => {
  event.preventDefault();
  canvas.setPointerCapture(event.pointerId);
  const point = eventPoint(event);
  if (state.mode === 'view') {
    activeViewPointers.set(event.pointerId, point);
    if (activeViewPointers.size === 1) state.dragLast = point;
    if (activeViewPointers.size === 2) beginPinchGesture();
    return;
  }
  if (!event.isPrimary) return;
  if (!state.map) {
    showToast('地图尚未加载');
    return;
  }
  if (state.mode === 'goal' && !state.ready.nav2) {
    showToast('Nav2 尚未就绪');
    return;
  }
  const world = screenToWorld(point.x, point.y);
  state.pointerStart = { ...point, world };
  state.pointerCurrent = point;
  state.dirty = true;
});

canvas.addEventListener('pointermove', (event) => {
  const point = eventPoint(event);
  const world = screenToWorld(point.x, point.y);
  if (world) $('pointerReadout').textContent = `x ${world.x.toFixed(2)} · y ${world.y.toFixed(2)}`;

  if (state.mode === 'view' && activeViewPointers.has(event.pointerId)) {
    activeViewPointers.set(event.pointerId, point);
    if (pinchGesture) {
      updatePinchGesture();
      return;
    }
  }

  if (state.mode === 'view' && state.dragLast && activeViewPointers.size === 1) {
    state.autoFitMap = false;
    state.view.panX += point.x - state.dragLast.x;
    state.view.panY += point.y - state.dragLast.y;
    state.dragLast = point;
    state.dirty = true;
  } else if (state.pointerStart) {
    state.pointerCurrent = point;
    state.dirty = true;
  }
});

canvas.addEventListener('pointerup', (event) => {
  if (state.mode === 'view') {
    finishViewPointer(event.pointerId);
    return;
  }
  state.dragLast = null;
  if (!state.pointerStart || state.mode === 'view') return;
  const end = eventPoint(event);
  const dx = end.x - state.pointerStart.x;
  const dy = end.y - state.pointerStart.y;
  const dragLength = Math.hypot(dx, dy);
  let yaw = dragLength > 10 ? Math.atan2(-dy, dx) : 0;
  if (dragLength <= 10 && state.pose) yaw = state.pose.yaw;
  const message = {
    type: state.mode === 'goal' ? 'nav_goal' : 'initial_pose',
    x: state.pointerStart.world.x,
    y: state.pointerStart.world.y,
    yaw,
  };
  if (send(message)) {
    showToast(state.mode === 'goal' ? '导航目标已发送' : '初始位置已发送');
    if (state.mode === 'goal') state.goal = { x: message.x, y: message.y, yaw };
  }
  state.pointerStart = null;
  state.pointerCurrent = null;
  setMode('view');
});

canvas.addEventListener('pointercancel', (event) => {
  finishViewPointer(event.pointerId);
  state.pointerStart = null;
  state.pointerCurrent = null;
  state.dragLast = null;
  state.dirty = true;
});

canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  zoomBy(event.deltaY < 0 ? 1.15 : 1 / 1.15, eventPoint(event));
}, { passive: false });

canvas.addEventListener('contextmenu', (event) => event.preventDefault());

function zoomBy(factor, center = null) {
  const anchorWorld = center && state.map
    ? screenToWorld(center.x, center.y) : null;
  state.autoFitMap = false;
  state.view.zoom = Math.max(0.35, Math.min(8, state.view.zoom * factor));
  if (anchorWorld) {
    const anchorScreen = mapToScreen(worldToMapPixel(anchorWorld.x, anchorWorld.y));
    state.view.panX += center.x - anchorScreen.x;
    state.view.panY += center.y - anchorScreen.y;
  }
  state.dirty = true;
}

document.querySelectorAll('[data-mode]').forEach((button) => {
  button.addEventListener('click', () => {
    if (!button.disabled) setMode(button.dataset.mode);
  });
});
document.querySelectorAll('[data-localizer]').forEach((button) => {
  button.addEventListener('click', () => {
    state.localizationChoice = button.dataset.localizer;
    updateLocalizationSelector();
    updateActionAvailability();
    const active = activeLocalizationType();
    if (active && active === state.localizationChoice) {
      showToast(active === 'cartographer'
        ? '当前已经运行图 SLAM 定位' : '当前已经运行 AMCL 定位');
    } else {
      showToast('已记录定位选择；请停止当前 launch 后启动对应定位入口');
    }
  });
});
$('zoomIn').addEventListener('click', () => zoomBy(1.25));
$('zoomOut').addEventListener('click', () => zoomBy(0.8));
$('fitMap').addEventListener('click', () => {
  fitKnownMap();
});
$('centerRobot').addEventListener('click', () => {
  if (!state.pose || !state.map) {
    showToast('机器人地图位置尚未就绪');
    return;
  }
  const metrics = canvasMetrics();
  const mapPoint = worldToMapPixel(state.pose.x, state.pose.y);
  state.autoFitMap = false;
  state.view.panX = -(mapPoint.x - state.map.width / 2) * metrics.scale;
  state.view.panY = -(mapPoint.y - state.map.height / 2) * metrics.scale;
  state.dirty = true;
});
$('toggleMapSize').addEventListener('click', () => {
  setMapExpanded(!state.mapExpanded);
});
$('cancelNavigation').addEventListener('click', () => {
  if (send({ type: 'cancel_navigation' })) showToast('正在取消导航');
});
$('clearVisualization').addEventListener('click', () => {
  if (send({ type: 'clear_visualization' })) clearDynamicVisualization(false);
});
$('resetLocalization').addEventListener('click', () => {
  if (!window.confirm('重置定位后必须重新设置初始位置。地图和 odom 不会被删除，继续吗？')) return;
  if (send({ type: 'reset_localization' })) {
    state.localizationReset = { state: 'resetting', message: '正在重置定位器' };
    clearDynamicVisualization(true, true);
    updateMatchCard();
  }
});
$('toggleScan').addEventListener('click', () => {
  state.overlays.scan = !state.overlays.scan;
  $('toggleScan').classList.toggle('active', state.overlays.scan);
  $('toggleScan').setAttribute('aria-pressed', String(state.overlays.scan));
  state.dirty = true;
});
$('toggleParticles').addEventListener('click', () => {
  state.overlays.particles = !state.overlays.particles;
  $('toggleParticles').classList.toggle('active', state.overlays.particles);
  $('toggleParticles').setAttribute('aria-pressed', String(state.overlays.particles));
  state.dirty = true;
});
$('toggleGlobalCostmap').addEventListener('click', () => {
  state.overlays.globalCostmap = !state.overlays.globalCostmap;
  $('toggleGlobalCostmap').classList.toggle('active', state.overlays.globalCostmap);
  $('toggleGlobalCostmap').setAttribute('aria-pressed', String(state.overlays.globalCostmap));
  state.dirty = true;
});
$('toggleLocalCostmap').addEventListener('click', () => {
  state.overlays.localCostmap = !state.overlays.localCostmap;
  $('toggleLocalCostmap').classList.toggle('active', state.overlays.localCostmap);
  $('toggleLocalCostmap').setAttribute('aria-pressed', String(state.overlays.localCostmap));
  state.dirty = true;
});
$('toggleGlobalPath').addEventListener('click', () => {
  state.overlays.globalPath = !state.overlays.globalPath;
  $('toggleGlobalPath').classList.toggle('active', state.overlays.globalPath);
  $('toggleGlobalPath').setAttribute('aria-pressed', String(state.overlays.globalPath));
  state.dirty = true;
});
$('toggleMppiTrajectories').addEventListener('click', () => {
  state.overlays.mppiTrajectories = !state.overlays.mppiTrajectories;
  $('toggleMppiTrajectories').classList.toggle('active', state.overlays.mppiTrajectories);
  $('toggleMppiTrajectories').setAttribute(
    'aria-pressed', String(state.overlays.mppiTrajectories),
  );
  state.dirty = true;
});
$('applyGlobalInflation').addEventListener('click', () => applyInflation('global'));
$('applyLocalInflation').addEventListener('click', () => applyInflation('local'));
[
  'globalEnabled', 'globalRadius', 'globalScaling',
  'globalInflateUnknown', 'globalInflateAroundUnknown',
  'localEnabled', 'localRadius', 'localScaling',
  'localInflateUnknown', 'localInflateAroundUnknown',
].forEach((id) => {
  $(id).addEventListener('input', () => { $(id).dataset.dirty = 'true'; });
  $(id).addEventListener('change', () => { $(id).dataset.dirty = 'true'; });
});
$('saveMap').addEventListener('click', () => {
  const name = $('mapName').value.trim();
  if (!name) {
    showToast('请输入地图名称');
    return;
  }
  if (send({ type: 'save_map', name })) {
    state.saveMap = { state: 'saving', message: '正在保存地图', path: null };
    updateMappingCard();
  }
});

$('launchProfile').addEventListener('change', updateLaunchCard);
$('launchMap').addEventListener('change', updateLaunchCard);
$('launchStart').addEventListener('click', () => {
  const profile = selectedLaunchProfile();
  if (!profile) {
    showToast('请选择要启动的 Launch');
    return;
  }
  const message = { type: 'launch_start', profile_id: profile.id };
  if (profile.requires_map) {
    const mapName = $('launchMap').value;
    if (!mapName) {
      showToast('请先保存并选择导航地图');
      return;
    }
    message.map_name = mapName;
  }
  if (send(message)) showToast(`正在启动 ${profile.label}`);
});
$('launchStop').addEventListener('click', () => {
  const activeName = state.launchControl.active?.label || '当前 Launch';
  if (!window.confirm(
    `停止“${activeName}”会关闭其雷达、定位和导航节点，继续吗？`,
  )) return;
  if (send({ type: 'launch_stop' })) showToast('正在停止当前 Launch');
});
$('launchClearLogs').addEventListener('click', () => {
  send({ type: 'launch_clear_logs' });
});

function updateClock() {
  $('clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
}

window.addEventListener('resize', () => {
  syncViewportHeight();
  resizeCanvas();
});
window.addEventListener('orientationchange', () => setTimeout(() => {
  syncViewportHeight();
  resizeCanvas();
}, 150));
window.visualViewport?.addEventListener('resize', () => {
  syncViewportHeight();
  resizeCanvas();
});
window.addEventListener('keydown', (event) => {
  if (event.key === 'Escape' && state.mapExpanded) setMapExpanded(false);
});
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) {
    state.dirty = true;
    if (state.connected) send({ type: 'request_launch_status' });
  }
});

updateLaunchCard();
boot();
