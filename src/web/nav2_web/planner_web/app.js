'use strict';

const $ = (id) => document.getElementById(id);
const stageOrder = ['lidar', 'localization', 'planning', 'control'];
const state = {
  socket: null,
  connected: false,
  reconnectTimer: null,
  reconnectDelay: 800,
  launch: { enabled: false, profiles: [], active: null, logs: [], maps: [], mapDirectory: '' },
  selectedProfile: null,
  sceneLayers: {},
  parameterValues: {},
};

function notifyAndroid(connected) {
  try { window.Nav2Android?.connectionChanged(Boolean(connected)); } catch (_error) { /* no-op */ }
}

function showToast(text, danger = false) {
  const toast = $('toast');
  toast.textContent = text;
  toast.classList.toggle('danger', danger);
  toast.classList.add('show');
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => toast.classList.remove('show'), 2600);
}
window.showToast = showToast;

function websocketUrl() {
  const port = Number(window.NAV2_ANDROID_WS_PORT) || 8891;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${protocol}://${location.hostname}:${port}`;
}

function send(payload) {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    showToast('机器人连接尚未就绪', true);
    return false;
  }
  state.socket.send(JSON.stringify(payload));
  return true;
}
window.nav2Send = send;

function setConnection(connected, label) {
  state.connected = connected;
  const pill = $('wsHealth');
  pill.classList.toggle('online', connected);
  pill.classList.toggle('pending', !connected);
  pill.querySelector('b').textContent = label;
  notifyAndroid(connected);
  window.scanScene?.connectionChanged(connected);
  updateLaunchUi();
}

function connect() {
  clearTimeout(state.reconnectTimer);
  setConnection(false, '连接中');
  let socket;
  try { socket = new WebSocket(websocketUrl()); } catch (_error) {
    scheduleReconnect();
    return;
  }
  state.socket = socket;
  socket.addEventListener('open', () => {
    state.reconnectDelay = 800;
    setConnection(true, '机器人在线');
    send({ type: 'request_snapshot' });
    send({ type: 'request_scene_snapshot' });
    send({ type: 'request_launch_status' });
  });
  socket.addEventListener('message', (event) => {
    try { handleMessage(JSON.parse(event.data)); } catch (_error) { /* ignore invalid frame */ }
  });
  socket.addEventListener('close', () => {
    if (state.socket !== socket) return;
    setConnection(false, '连接断开');
    scheduleReconnect();
  });
  socket.addEventListener('error', () => socket.close());
}

function scheduleReconnect() {
  clearTimeout(state.reconnectTimer);
  state.reconnectTimer = setTimeout(connect, state.reconnectDelay);
  state.reconnectDelay = Math.min(8000, state.reconnectDelay * 1.6);
}

function handleMessage(message) {
  if (String(message.type || '').startsWith('scene_')) {
    if (message.type === 'scene_status') state.sceneLayers = message.layers || {};
    window.scanScene?.handleMessage(message);
    updatePipeline();
    return;
  }
  switch (message.type) {
    case 'launch_status':
      state.launch = {
        enabled: Boolean(message.enabled),
        profiles: Array.isArray(message.profiles) ? message.profiles : [],
        active: message.active || null,
        logs: Array.isArray(message.logs) ? message.logs : [],
        maps: Array.isArray(message.maps) ? message.maps : [],
        mapDirectory: String(message.map_directory || ''),
      };
      if (!state.selectedProfile || !state.launch.profiles.some((item) => item.id === state.selectedProfile)) {
        state.selectedProfile = state.launch.active?.profile_id || state.launch.profiles[0]?.id || null;
      }
      renderProfiles();
      renderMapLibrary();
      renderParameterFields();
      syncSceneBodyHeight();
      window.scanScene?.setWorkflow(state.launch.active);
      renderLogs();
      updateLaunchUi();
      break;
    case 'launch_log':
      if (message.entry) {
        state.launch.logs.push(message.entry);
        if (state.launch.logs.length > 1200) state.launch.logs.shift();
        renderLogs();
      }
      break;
    case 'launch_error':
    case 'error':
      showToast(message.message || '操作失败', true);
      break;
    case 'launch_map_save_status':
      showToast(
        message.message || 'FAST-LIO 地图保存状态已更新',
        message.state === 'error',
      );
      break;
    case 'scanplanner_goal_status':
      showToast(message.message || '目标点已发送');
      window.scanScene?.goalPublished(message.goal || null);
      break;
    case 'pct_waypoints_status':
      showToast(message.message || 'PCT 多点路线已发送');
      window.scanScene?.routePublished(message.waypoints || []);
      break;
    case 'initial_pose_status':
      showToast(message.message || '粗定位初始位姿已发送');
      window.scanScene?.initialPosePublished(message.pose || null);
      break;
    case 'notice':
      showToast(message.message || '操作完成');
      break;
    default:
      break;
  }
}

function selectedProfile() {
  return state.launch.profiles.find((profile) => profile.id === state.selectedProfile) || null;
}

function syncSceneBodyHeight() {
  const activeValue = state.launch.active?.parameters?.body_height;
  const profile = selectedProfile();
  const selectedValue = profileParameterValues(profile)?.body_height;
  const height = Number(activeValue ?? selectedValue ?? 0.4);
  window.scanScene?.setBodyHeight(Number.isFinite(height) ? height : 0.4);
}

function scanplannerMaps() {
  return (state.launch.maps || []).filter((entry) => entry.kind === 'scanplanner');
}

function normalizeMapName(value) {
  return String(value || '').trim().replace(/\.(?:pcd|pickle)$/i, '');
}

function mapEntry(name) {
  const normalized = normalizeMapName(name);
  return scanplannerMaps().find((entry) => entry.name === normalized) || null;
}

function mapFitsRole(entry, role) {
  if (!entry) return false;
  if (role === 'tomogram') return Boolean(entry.has_pcd);
  if (role === 'bundle') return Boolean(entry.has_pcd && entry.has_tomogram);
  return true;
}

function renderMapLibrary() {
  const panel = $('launchMapLibrary');
  const profile = selectedProfile();
  const role = profile?.map_role || '';
  panel.hidden = !role;
  if (!role) return;

  const maps = scanplannerMaps();
  const choices = $('launchMapChoices');
  choices.replaceChildren();
  maps.forEach((entry) => {
    const option = document.createElement('option');
    option.value = entry.name;
    const files = [entry.has_pcd ? 'PCD' : null, entry.has_tomogram ? 'PCT' : null].filter(Boolean);
    option.label = `${entry.name} · ${files.join(' + ') || '文件不完整'}`;
    choices.appendChild(option);
  });

  const input = $('launchMapName');
  let current = normalizeMapName(input.value);
  let saved = '';
  try { saved = normalizeMapName(localStorage.getItem('planner.map.name') || ''); } catch (_error) { saved = ''; }
  const active = normalizeMapName(state.launch.active?.map_name || '');
  const preferred = normalizeMapName(profile.default_map_name || '');
  if (!current || (role !== 'pcd_output' && !mapFitsRole(mapEntry(current), role))) {
    const candidate = maps.find((entry) => mapFitsRole(entry, role));
    current = role === 'pcd_output'
      ? (active || saved || preferred)
      : active || (mapFitsRole(mapEntry(saved), role) ? saved : '')
        || (mapFitsRole(mapEntry(preferred), role) ? preferred : '')
        || candidate?.name || '';
    input.value = current;
  }

  $('launchMapDirectory').textContent = `安全目录：${state.launch.mapDirectory || '--'}`;
  const entry = mapEntry(current);
  const status = $('launchMapStatus');
  status.className = 'launch-map-status';
  if (role === 'pcd_output') {
    if (entry) {
      status.textContent = `${current} 已存在；建图保存会被后端拒绝，请输入新名字`;
      status.classList.add('warning');
    } else {
      status.textContent = current
        ? `将新建 ${current}.pcd` : '请输入新地图名称';
    }
  } else if (!entry) {
    status.textContent = '列表中没有这个地图名';
    status.classList.add('warning');
  } else if (role === 'tomogram') {
    status.textContent = entry.has_pcd
      ? `${current}.pcd → ${current}.pickle`
      : `${current} 缺少 PCD`;
    if (!entry.has_pcd) status.classList.add('warning');
  } else {
    status.textContent = `${current}：PCD ${entry.has_pcd ? '就绪' : '缺少'} · PCT ${entry.has_tomogram ? '就绪' : '缺少'}`;
    if (!entry.has_pcd || !entry.has_tomogram) status.classList.add('warning');
  }
}

function parameterStorageKey(profileId) {
  return `planner.launch.parameters.${profileId}`;
}

function profileParameterValues(profile, reset = false) {
  if (!profile) return {};
  if (!reset && state.parameterValues[profile.id]) return state.parameterValues[profile.id];
  let stored = {};
  if (!reset) {
    try { stored = JSON.parse(localStorage.getItem(parameterStorageKey(profile.id)) || '{}'); } catch (_error) { stored = {}; }
  }
  const values = {};
  (profile.parameters || []).forEach((field) => {
    values[field.name] = Object.prototype.hasOwnProperty.call(stored, field.name)
      ? stored[field.name] : field.default;
  });
  state.parameterValues[profile.id] = values;
  return values;
}

function saveProfileParameters(profile) {
  if (!profile) return;
  try {
    localStorage.setItem(
      parameterStorageKey(profile.id),
      JSON.stringify(state.parameterValues[profile.id] || {}),
    );
  } catch (_error) { /* WebView storage can be disabled by device policy. */ }
}

function renderParameterFields(reset = false) {
  const container = $('launchParameterFields');
  const panel = $('launchParameters');
  const profile = selectedProfile();
  const hasParameters = Boolean(
    profile && Array.isArray(profile.parameters) && profile.parameters.length,
  );
  if (panel) panel.hidden = !hasParameters;
  container.replaceChildren();
  if (!hasParameters) return;
  const values = profileParameterValues(profile, reset);
  let previousGroup = '';
  profile.parameters.forEach((field) => {
    if (field.group !== previousGroup) {
      const group = document.createElement('div');
      group.className = 'parameter-group-title';
      group.textContent = field.group || '其他参数';
      container.appendChild(group);
      previousGroup = field.group;
    }
    const label = document.createElement('label');
    label.className = 'parameter-field';
    const text = document.createElement('span');
    const title = document.createElement('b');
    title.textContent = field.label;
    text.appendChild(title);
    if (field.description) {
      const hint = document.createElement('small');
      hint.textContent = field.description;
      text.appendChild(hint);
    }
    let input;
    if (field.kind === 'bool') {
      input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = Boolean(values[field.name]);
    } else if (field.kind === 'choice') {
      input = document.createElement('select');
      (field.choices || []).forEach((choice) => {
        const option = document.createElement('option');
        option.value = choice.value;
        option.textContent = choice.label;
        input.appendChild(option);
      });
      input.value = String(values[field.name] ?? field.default ?? '');
    } else {
      input = document.createElement('input');
      input.type = ['float', 'int'].includes(field.kind) ? 'number' : 'text';
      input.value = String(values[field.name] ?? field.default ?? '');
      if (field.kind === 'float') input.step = 'any';
      if (field.kind === 'int') input.step = '1';
      if (field.minimum != null) input.min = String(field.minimum);
      if (field.maximum != null) input.max = String(field.maximum);
    }
    input.dataset.parameterName = field.name;
    input.addEventListener('change', () => {
      const current = state.parameterValues[profile.id] || {};
      current[field.name] = field.kind === 'bool' ? input.checked : input.value;
      state.parameterValues[profile.id] = current;
      saveProfileParameters(profile);
      if (field.name === 'body_height') syncSceneBodyHeight();
    });
    label.append(text, input);
    container.appendChild(label);
  });
  if (reset) saveProfileParameters(profile);
}

function renderProfiles() {
  const list = $('profileList');
  const rank = (profile) => {
    if (profile.id === 'fastlio_global_mapping') return 10;
    if (profile.id === 'pct_tomogram_build') return 20;
    if (profile.id === 'pct_offline_demo') return 25;
    if (profile.id === 'mid360s_terminal') return 30;
    if (profile.id === 'fastlio_terminal') return 40;
    if (profile.id.startsWith('scanplanner_mode') && !profile.control_enabled) return 50 + (profile.navigation_mode || 0);
    if (profile.id === 'pct_scanplanner_mode3_preview') return 60;
    if (profile.id.startsWith('scanplanner_mode') && profile.control_enabled) return 70 + (profile.navigation_mode || 0);
    if (profile.id === 'pct_scanplanner_mode3_control') return 80;
    return 100;
  };
  const profiles = [...state.launch.profiles].sort((left, right) => rank(left) - rank(right));
  $('profileCount').textContent = `${profiles.length} 个模式`;
  list.replaceChildren();
  if (!profiles.length) {
    const empty = document.createElement('div');
    empty.className = 'profile-placeholder';
    empty.textContent = state.connected ? '启动服务未启用，请使用 planner_app.launch.py' : '等待机器人连接…';
    list.appendChild(empty);
    return;
  }
  let previousGroup = '';
  profiles.forEach((profile, index) => {
    let group;
    if (profile.id === 'fastlio_global_mapping' || profile.id === 'pct_tomogram_build') {
      group = '01 · 地图离线准备';
    } else if (profile.id === 'pct_offline_demo') {
      group = '02 · PCT 示例地图离线验证';
    } else if (profile.id.startsWith('pct_scanplanner')) {
      group = profile.control_enabled ? '07 · 历史地图 PCT 实机控制' : '05 · 历史地图 PCT 安全预览';
    } else if (profile.navigation_mode == null) {
      group = '03 · 雷达与 FAST-LIO 独立诊断';
    } else {
      group = profile.control_enabled ? '06 · SCAN 模式 1 / 2 / 3 实机控制' : '04 · SCAN 模式 1 / 2 / 3 安全预览';
    }
    if (group !== previousGroup) {
      const heading = document.createElement('div');
      heading.className = 'profile-group-title';
      heading.textContent = group;
      list.appendChild(heading);
      previousGroup = group;
    }
    const button = document.createElement('button');
    button.type = 'button';
    button.disabled = profile.available === false;
    button.className = `profile-option${profile.id === state.selectedProfile ? ' selected' : ''}${profile.dangerous ? ' dangerous' : ''}${profile.available === false ? ' unavailable' : ''}`;
    button.dataset.profileId = profile.id;
    const number = document.createElement('b');
    number.textContent = String(index + 1).padStart(2, '0');
    const body = document.createElement('span');
    const title = document.createElement('strong');
    title.textContent = profile.label;
    const detail = document.createElement('small');
    detail.textContent = profile.available === false
      ? profile.unavailable_reason || '尚未完成配置' : profile.description;
    body.append(title, detail);
    const tag = document.createElement('em');
    tag.textContent = profile.navigation_mode == null
      ? profile.stage.toUpperCase() : `MODE ${profile.navigation_mode}`;
    button.append(number, body, tag);
    button.addEventListener('click', () => {
      if (profile.available === false) return;
      state.selectedProfile = profile.id;
      renderProfiles();
      renderMapLibrary();
      renderParameterFields();
      syncSceneBodyHeight();
      updateLaunchUi();
    });
    list.appendChild(button);
  });
}

function renderLogs() {
  const panel = $('launchLog');
  const atBottom = panel.scrollHeight - panel.scrollTop - panel.clientHeight < 80;
  panel.replaceChildren();
  const logs = state.launch.logs || [];
  if (!logs.length) {
    const wait = document.createElement('span');
    wait.className = 'terminal-wait';
    wait.textContent = '等待 Launch 输出…';
    panel.appendChild(wait);
  } else {
    const fragment = document.createDocumentFragment();
    logs.forEach((entry) => {
      const line = document.createElement('div');
      line.className = `terminal-line ${entry.level || 'output'}`;
      const time = document.createElement('time');
      time.textContent = entry.time || '--:--:--';
      const text = document.createElement('span');
      text.textContent = entry.line || '';
      line.append(time, text);
      fragment.appendChild(line);
    });
    panel.appendChild(fragment);
  }
  $('launchLogCount').textContent = `${logs.length} 行`;
  if (atBottom) panel.scrollTop = panel.scrollHeight;
}

function updatePipeline() {
  const active = state.launch.active;
  const running = Boolean(active?.running);
  const profile = state.launch.profiles.find((item) => item.id === active?.profile_id);
  const activeIndex = running ? stageOrder.indexOf(profile?.stage) : -1;
  const lidarFresh = Number(state.sceneLayers.lidar?.age) < 2.5;
  const registeredFresh = Number(state.sceneLayers.registered?.age) < 2.5;
  const planningFresh = Number(state.sceneLayers.planning?.age) < 2.5
    || Number(state.sceneLayers.occupancy?.age) < 2.5
    || Number(state.sceneLayers.traversable?.age) < 2.5;
  stageOrder.forEach((stage, index) => {
    const readyByMode = running && index <= activeIndex;
    const readyByData = (stage === 'lidar' && lidarFresh)
      || (stage === 'localization' && registeredFresh)
      || (stage === 'planning' && planningFresh);
    const ready = stage === 'control' ? readyByMode : readyByData;
    const name = stage[0].toUpperCase() + stage.slice(1);
    $(`stage${name}`)?.classList.toggle('live', ready);
    const miniName = stage === 'localization' ? 'Fastlio' : name;
    $(`mini${miniName}`)?.classList.toggle('live', ready);
  });
  $('pipelineTitle').textContent = running && !lidarFresh
    ? '进程已启动，等待 Mid-360S 数据'
    : running ? `${active.label}运行中` : '实机链路待机';
}

function updateLaunchUi() {
  const active = state.launch.active;
  const running = Boolean(active?.running);
  const selected = selectedProfile();
  const busy = ['starting', 'saving', 'stopping'].includes(active?.state);
  const badge = $('launchStateBadge');
  const labels = { starting: '启动中', running: '运行中', saving: '保存地图中', stopping: '停止中', exited: '已退出', error: '启动失败' };
  badge.className = `state-badge ${active?.state || 'idle'}`;
  badge.querySelector('span').textContent = labels[active?.state] || (state.launch.enabled ? '未运行' : '启动服务关闭');
  $('selectedDescription').textContent = selected?.available === false
    ? selected.unavailable_reason || '该模式尚未完成配置'
    : selected?.description || '连接机器人后选择模式';
  $('selectedDescription').classList.toggle('danger', Boolean(selected?.dangerous));
  $('launchStart').disabled = !state.connected || !state.launch.enabled || !selected
    || selected.available === false || running || busy
    || Boolean(selected.map_role && !normalizeMapName($('launchMapName').value));
  const saveBeforeStop = Boolean(active?.save_before_stop);
  $('launchStop').disabled = !running || ['saving', 'stopping'].includes(active?.state);
  $('launchStop').querySelector('b').textContent = saveBeforeStop
    ? '保存地图并停止' : '停止当前流程';
  $('terminalStop').disabled = $('launchStop').disabled;
  $('terminalStop').textContent = saveBeforeStop ? '保存并停止' : '停止流程';
  $('terminalSubtitle').textContent = active ? `${active.label} · ${labels[active.state] || active.state}` : '等待启动流程';
  $('terminalPid').textContent = active?.pid ? `PID ${active.pid}` : 'PID --';
  $('terminalExit').textContent = active?.exit_code == null ? 'exit --' : `exit ${active.exit_code}`;
  $('terminalCommand').textContent = active ? `planner@robot: ${active.label}` : 'planner@robot: ros2 launch …';
  $('safetyHint').textContent = selected?.dangerous
    ? '高风险模式：会发布 /cmd_vel。启动前必须清空周围人员与障碍，并准备急停。'
    : '启动 FAST-LIO 后请先让雷达静止 5–10 秒完成 IMU 初始化。';
  $('safetyHint').classList.toggle('danger', Boolean(selected?.dangerous));
  updatePipeline();
}

function switchScreen(name) {
  document.querySelectorAll('[data-screen]').forEach((button) => {
    button.classList.toggle('active', button.dataset.screen === name);
  });
  document.querySelectorAll('.screen').forEach((screen) => screen.classList.remove('active'));
  $(`${name}Screen`)?.classList.add('active');
  document.body.dataset.screen = name;
  if (name === 'scene') {
    window.scanScene?.setMode(true);
    setTimeout(() => window.scanScene?.resize(), 30);
  }
}

function startSelected() {
  const profile = selectedProfile();
  if (!profile) return;
  if (profile.dangerous) {
    const accepted = window.confirm('该模式会向真实机器狗发布 /cmd_vel。\n\n请确认：机器狗已架起或场地安全、急停可用。\n\n继续启动实机控制？');
    if (!accepted) return;
  }
  const fields = [...document.querySelectorAll('[data-parameter-name]')];
  if (fields.some((input) => typeof input.reportValidity === 'function' && !input.reportValidity())) return;
  const parameters = profileParameterValues(profile);
  fields.forEach((input) => {
    const spec = (profile.parameters || []).find((item) => item.name === input.dataset.parameterName);
    parameters[input.dataset.parameterName] = spec?.kind === 'bool' ? input.checked : input.value;
  });
  saveProfileParameters(profile);
  const mapName = profile.map_role
    ? normalizeMapName($('launchMapName').value) : null;
  if (mapName) {
    try { localStorage.setItem('planner.map.name', mapName); } catch (_error) { /* no-op */ }
  }
  if (send({ type: 'launch_start', profile_id: profile.id, map_name: mapName, parameters })) {
    showToast(`正在启动：${profile.label}`);
    switchScreen('terminal');
  }
}

function stopActive() {
  const active = state.launch.active;
  if (!active?.running) return;
  const saveBeforeStop = Boolean(active.save_before_stop);
  const target = active.map_name ? `${active.map_name}.pcd` : '当前 PCD';
  const prompt = saveBeforeStop
    ? `保存 ${target} 并停止“${active.label}”？\n\n保存完成前请勿关闭 Web 或 FAST-LIO。`
    : `停止“${active.label}”？`;
  if (!window.confirm(prompt)) return;
  if (send({ type: 'launch_stop' })) {
    showToast(saveBeforeStop
      ? '正在保存 PCD，成功后自动停止'
      : '正在停止当前流程');
  }
}

document.querySelectorAll('[data-screen]').forEach((button) => {
  button.addEventListener('click', () => switchScreen(button.dataset.screen));
});
$('openLaunch').addEventListener('click', () => switchScreen('launch'));
$('terminalToScene').addEventListener('click', () => switchScreen('scene'));
$('launchStart').addEventListener('click', startSelected);
$('launchStop').addEventListener('click', stopActive);
$('terminalStop').addEventListener('click', stopActive);
$('launchClearLogs').addEventListener('click', () => {
  if (send({ type: 'launch_clear_logs' })) {
    state.launch.logs = [];
    renderLogs();
  }
});
$('launchResetParameters').addEventListener('click', () => {
  const profile = selectedProfile();
  if (!profile) return;
  try { localStorage.removeItem(parameterStorageKey(profile.id)); } catch (_error) { /* no-op */ }
  renderParameterFields(true);
  showToast('已恢复当前模式的默认参数');
});
$('launchMapName').addEventListener('change', () => {
  renderMapLibrary();
  updateLaunchUi();
});
$('launchRefreshMaps').addEventListener('click', () => {
  if (send({ type: 'request_launch_status' })) showToast('正在刷新地图库');
});

setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false }); }, 1000);
setInterval(() => { if (state.connected) send({ type: 'request_launch_status' }); }, 2000);
renderProfiles();
renderParameterFields();
renderLogs();
updateLaunchUi();
connect();
