const MODULES = {
  impedance: {
    title: "阻抗控制",
    subtitle: "/home/lumos/franka_ros2_ws/src/serl_franka_controllers_ros2",
  },
  hand: {
    title: "灵巧手控制",
    subtitle: "/home/lumos/franka_ros2_ws/src/wujihandros2 + src/wujihandpy",
  },
  inference: {
    title: "推理控制",
    subtitle: "/home/lumos/luolei/easydp",
  },
  teleop: {
    title: "摇操控制",
    subtitle: "/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz",
  },
  unified: {
    title: "统一控制",
    subtitle: "/home/lumos/franka_ros2_ws/src/unified_impedance_control",
  },
};

const params = new URLSearchParams(window.location.search);
const moduleName = params.get("module") || "impedance";
const isEmbedded = params.get("embedded") === "1";
const moduleInfo = MODULES[moduleName] || MODULES.impedance;
let statusCache = null;
let optionsCache = null;
let settingsCache = null;
let previousManagedProcesses = null;
let refreshPending = false;
let connectionFailed = false;
let runtimeLogReadFailed = false;
let confirmResolver = null;
let inferenceWorkflowBusy = false;
let previousDeviceConnections = {};
const announcedDisconnections = new Set();
const runtimeLogFollow = { arm: true, hand: true };
const runtimeLogCache = {
  arm: { source: "", text: "" },
  hand: { source: "", text: "" },
};

const $ = (selector) => document.querySelector(selector);

function setText(selector, text) {
  const el = $(selector);
  if (el) el.textContent = text;
}

function toast(message, ok = true) {
  const stack = $("#toast-stack");
  if (!stack) return;
  const el = document.createElement("div");
  const type = ok === "warn" ? "warn" : ok ? "ok" : "err";
  el.className = `toast ${type}`;
  el.textContent = message;
  el.addEventListener("click", () => el.remove());
  stack.append(el);
  while (stack.children.length > 4) stack.firstElementChild?.remove();
  window.setTimeout(() => el.remove(), type === "err" ? 7000 : 4400);
}

function setOperation(state, title, message, dismissible = state !== "working") {
  const bar = $("#operation-bar");
  if (!bar) return;
  bar.dataset.state = state;
  setText("#operation-state", title);
  setText("#operation-message", message);
  $("#operation-dismiss")?.classList.toggle("hidden", !dismissible);
}

function setButtonBusy(button, busy, label = "处理中...") {
  if (!button) return;
  if (busy) {
    button.dataset.originalText = button.textContent;
    button.textContent = label;
    button.classList.add("is-busy");
    button.disabled = true;
  } else {
    button.textContent = button.dataset.originalText || button.textContent;
    delete button.dataset.originalText;
    button.classList.remove("is-busy");
    button.disabled = false;
  }
}

function requestConfirmation(title, message, confirmLabel = "确认执行") {
  const modal = $("#confirm-modal");
  if (!modal) return Promise.resolve(true);
  setText("#confirm-title", title);
  setText("#confirm-message", message);
  setText("#confirm-submit", confirmLabel);
  modal.classList.remove("hidden");
  $("#confirm-submit")?.focus();
  return new Promise((resolve) => {
    confirmResolver = resolve;
  });
}

function resolveConfirmation(confirmed) {
  $("#confirm-modal")?.classList.add("hidden");
  const resolve = confirmResolver;
  confirmResolver = null;
  resolve?.(confirmed);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const contentType = response.headers.get("content-type") || "";
  if (!contentType.includes("application/json")) {
    throw new Error(`服务器返回了无效响应 (HTTP ${response.status})`);
  }
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || `HTTP ${response.status}`);
  }
  return data;
}

function optionTags(items, { valueKey = "label", labelKey = "label", selected = "" } = {}) {
  return (items || []).map((item) => {
    const value = item[valueKey] || "";
    const label = item[labelKey] || value;
    return `<option value="${escapeHtml(value)}" ${value === selected ? "selected" : ""}>${escapeHtml(label)}</option>`;
  }).join("");
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (char) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "\"": "&quot;",
    "'": "&#39;",
  })[char]);
}

function selectValue(id) {
  const el = document.getElementById(id);
  return el ? el.value : "";
}

function buildSelect(id, label, items, opts = {}) {
  return `
    <label>${label}
      <select id="${id}">
        ${optionTags(items, opts)}
      </select>
    </label>
  `;
}

function withCurrentOption(items, current, valueKey = "value") {
  const value = String(current ?? "");
  const exists = (items || []).some((item) => String(item[valueKey] ?? item.value ?? item.path ?? item.label ?? "") === value);
  if (exists || value === "") return items || [];
  return [{ label: value, [valueKey]: value }, ...(items || [])];
}

const IMPEDANCE_DEFAULTS = {
  launch: "http_control.launch.py",
  arm: "both",
  transK: "2600",
  transD: "170",
  rotK: "550",
  rotD: "4.5",
};

const IMPEDANCE_CHOICES = {
  arms: [
    { label: "left + right", value: "both" },
    { label: "left", value: "left" },
    { label: "right", value: "right" },
  ],
  transK: ["1500", "2020", "2600", "3000"].map((value) => ({ label: value, value })),
  transD: ["80", "89", "120", "170"].map((value) => ({ label: value, value })),
  rotK: ["250", "300", "550", "650"].map((value) => ({ label: value, value })),
  rotD: ["4.5", "7", "10", "15"].map((value) => ({ label: value, value })),
};

const IMPEDANCE_PROFILE_SECTIONS = [
  { name: "franka_stack", title: "Franka Stack 参数" },
  { name: "policy_inference", title: "Policy 阻抗栈参数" },
];

function storedImpedanceSettings() {
  try {
    return JSON.parse(window.localStorage.getItem("fkviewer.impedance.settings") || "{}");
  } catch {
    return {};
  }
}

function currentImpedanceSettings() {
  const stored = storedImpedanceSettings();
  const modalOpen = !$("#settings-modal")?.classList.contains("hidden");
  const value = (id, key) => (modalOpen ? selectValue(id) : "") || stored[key] || IMPEDANCE_DEFAULTS[key];
  return {
    launch: value("impedance-launch", "launch"),
    arm: value("impedance-arm", "arm"),
    transK: value("impedance-trans-k", "transK"),
    transD: value("impedance-trans-d", "transD"),
    rotK: value("impedance-rot-k", "rotK"),
    rotD: value("impedance-rot-d", "rotD"),
  };
}

function impedanceSettingsMarkup(options, settings) {
  return `
    <section class="settings-section">
      <h3>启动选择</h3>
      <div class="settings-fields">
        ${buildSelect("impedance-launch", "SERL Launch", options.launches || [], { selected: settings.launch })}
        ${buildSelect("impedance-arm", "目标手臂", IMPEDANCE_CHOICES.arms, { valueKey: "value", selected: settings.arm })}
      </div>
    </section>
    <section class="settings-section">
      <h3>阻抗参数</h3>
      <div class="settings-fields">
        ${buildSelect("impedance-trans-k", "平移刚度", IMPEDANCE_CHOICES.transK, { valueKey: "value", selected: settings.transK })}
        ${buildSelect("impedance-trans-d", "平移阻尼", IMPEDANCE_CHOICES.transD, { valueKey: "value", selected: settings.transD })}
        ${buildSelect("impedance-rot-k", "旋转刚度", IMPEDANCE_CHOICES.rotK, { valueKey: "value", selected: settings.rotK })}
        ${buildSelect("impedance-rot-d", "旋转阻尼", IMPEDANCE_CHOICES.rotD, { valueKey: "value", selected: settings.rotD })}
      </div>
    </section>
  `;
}

function profileParamOptions(key, value, options) {
  if (key === "config_file" || key === "wuji_config_file") {
    return { items: options.quest_configs || [], valueKey: "path" };
  }
  if (key === "data_recorder_config_file") {
    return { items: options.data_recorder_configs || [], valueKey: "path" };
  }
  if (key === "robot_type") {
    return { items: options.robot_types || [], valueKey: "value" };
  }
  if (key.endsWith("_robot_ip")) {
    return { items: options.robot_ips || [], valueKey: "value" };
  }
  if (key === "base_frame") {
    return { items: options.base_frames || [], valueKey: "value" };
  }
  if (key === "wuji_control_mode") {
    return { items: options.control_modes || [], valueKey: "value" };
  }
  if (key.endsWith("_cpu")) {
    return { items: options.cpu_choices || [], valueKey: "value" };
  }
  if (key.endsWith("_port")) {
    return { items: options.port_choices || [], valueKey: "value" };
  }
  if (key.endsWith("_serial")) {
    return { items: options.wuji_serials || [], valueKey: "value" };
  }
  if (key === "wujihand_state_rate") {
    return { items: options.rate_choices || [], valueKey: "value" };
  }
  if (/^(start_|auto_|load_|left_wuji_enabled|right_wuji_enabled|wuji_dry_run)/.test(key)) {
    return { items: options.boolean_choices || [], valueKey: "value" };
  }
  return { items: [{ label: value || "\"\"", value }], valueKey: "value" };
}

function profileParamSelect(profileName, param, options) {
  const spec = profileParamOptions(param.key, param.value, options);
  const items = withCurrentOption(spec.items, param.value, spec.valueKey);
  return `
    <label>${escapeHtml(param.key)}
      <select data-profile="${escapeHtml(profileName)}" data-key="${escapeHtml(param.key)}">
        ${optionTags(items, { valueKey: spec.valueKey, selected: param.value })}
      </select>
    </label>
  `;
}

function impedanceProfileSettingsMarkup(yaml, options) {
  const profiles = yaml?.profiles || [];
  return IMPEDANCE_PROFILE_SECTIONS.map((section) => {
    const profile = profiles.find((item) => item.name === section.name) || { name: section.name, params: [] };
    return `
      <section class="settings-section">
        <h3>${escapeHtml(section.title)}</h3>
        <div class="settings-fields">
          ${(profile.params || []).map((param) => profileParamSelect(section.name, param, options)).join("")}
        </div>
      </section>
    `;
  }).join("");
}

const HAND_JOINT_COUNT = 20;
const HAND_DEFAULTS = {
  launch: "wujihand_dual.launch.py",
  wujiConfig: "",
  wujiPolicy: "",
  leftTopic: "/hand_left/joint_commands",
  rightTopic: "/hand_right/joint_commands",
  leftSetEnabled: "/hand_left/set_enabled",
  rightSetEnabled: "/hand_right/set_enabled",
  leftResetError: "/hand_left/reset_error",
  rightResetError: "/hand_right/reset_error",
  rviz: "false",
  foxglove: "false",
  handName: "hand_0",
  handNames: "hand_left,hand_right",
  handSerial: "",
  handSide: "",
  publishRate: "1000.0",
  filterCutoff: "10.0",
  diagnosticsRate: "10.0",
  min: "-0.5",
  max: "1.65",
  step: "0.001",
  throttleMs: "120",
};
let handValues = {
  left: Array(HAND_JOINT_COUNT).fill(0),
  right: Array(HAND_JOINT_COUNT).fill(0),
};
let handSendTimers = {};
let handSending = {};
let handPending = {};

function storedHandSettings() {
  try {
    return JSON.parse(window.localStorage.getItem("fkviewer.hand.settings") || "{}");
  } catch {
    return {};
  }
}

function currentHandSettings() {
  const stored = storedHandSettings();
  const modalOpen = !$("#settings-modal")?.classList.contains("hidden");
  const fallbackConfig = optionsCache?.hand?.config_file || HAND_DEFAULTS.wujiConfig;
  const fallbackPolicy = (optionsCache?.hand?.bridge_configs || []).find((item) => item.label.includes("wuji_policy_bridge"))?.path || HAND_DEFAULTS.wujiPolicy;
  const params = optionsCache?.hand?.params || {};
  const value = (id, key, fallback = HAND_DEFAULTS[key]) => (modalOpen ? selectValue(id) : "") || stored[key] || fallback;
  const inputValue = (id, key, fallback = HAND_DEFAULTS[key]) => {
    const el = modalOpen ? document.getElementById(id) : null;
    return (el ? el.value : "") || stored[key] || fallback;
  };
  return {
    launch: value("hand-launch", "launch"),
    wujiConfig: value("hand-trigger-config", "wujiConfig", fallbackConfig),
    wujiPolicy: value("hand-policy-config", "wujiPolicy", fallbackPolicy),
    leftTopic: value("hand-left-topic", "leftTopic", params.left_command_topic || HAND_DEFAULTS.leftTopic),
    rightTopic: value("hand-right-topic", "rightTopic", params.right_command_topic || HAND_DEFAULTS.rightTopic),
    leftSetEnabled: inputValue("hand-left-set-enabled", "leftSetEnabled"),
    rightSetEnabled: inputValue("hand-right-set-enabled", "rightSetEnabled"),
    leftResetError: inputValue("hand-left-reset-error", "leftResetError"),
    rightResetError: inputValue("hand-right-reset-error", "rightResetError"),
    rviz: value("hand-rviz", "rviz"),
    foxglove: value("hand-foxglove", "foxglove"),
    handName: inputValue("hand-name", "handName"),
    handNames: value("hand-names", "handNames"),
    handSerial: inputValue("hand-serial", "handSerial"),
    handSide: value("hand-side-setting", "handSide"),
    publishRate: value("hand-publish-rate", "publishRate"),
    filterCutoff: inputValue("hand-filter-cutoff", "filterCutoff"),
    diagnosticsRate: inputValue("hand-diagnostics-rate", "diagnosticsRate"),
    min: inputValue("hand-slider-min", "min"),
    max: inputValue("hand-slider-max", "max"),
    step: inputValue("hand-slider-step", "step"),
    throttleMs: inputValue("hand-throttle-ms", "throttleMs"),
  };
}

function handSettingsMarkup(options, settings) {
  const bridge = options.bridge_configs || [];
  const topicChoices = options.topic_choices || [];
  return `
    <section class="settings-section">
      <h3>启动与配置</h3>
      <div class="settings-fields">
        ${buildSelect("hand-launch", "wujihandros2 Launch", options.ros2_launches || [], { selected: settings.launch })}
        ${buildSelect("hand-trigger-config", "姿态 YAML", bridge, { valueKey: "path", selected: settings.wujiConfig })}
        ${buildSelect("hand-policy-config", "Policy Bridge YAML", bridge, { valueKey: "path", selected: settings.wujiPolicy })}
        ${buildSelect("hand-rviz", "RViz", options.boolean_choices || [], { valueKey: "value", selected: settings.rviz })}
        ${buildSelect("hand-foxglove", "Foxglove", options.boolean_choices || [], { valueKey: "value", selected: settings.foxglove })}
        ${buildSelect("hand-names", "home hand_names", withCurrentOption(options.hand_names || [], settings.handNames), { valueKey: "value", selected: settings.handNames })}
        <label>single hand_name<input id="hand-name" value="${escapeHtml(settings.handName)}"></label>
        <label>single serial<input id="hand-serial" value="${escapeHtml(settings.handSerial)}"></label>
        ${buildSelect("hand-side-setting", "single hand_side", options.hand_sides || [], { valueKey: "value", selected: settings.handSide })}
        ${buildSelect("hand-publish-rate", "publish_rate", options.rate_choices || [], { valueKey: "value", selected: settings.publishRate })}
        <label>filter_cutoff_freq<input id="hand-filter-cutoff" value="${escapeHtml(settings.filterCutoff)}"></label>
        <label>diagnostics_rate<input id="hand-diagnostics-rate" value="${escapeHtml(settings.diagnosticsRate)}"></label>
      </div>
    </section>
    <section class="settings-section">
      <h3>ROS2 控制接口</h3>
      <div class="settings-fields">
        ${buildSelect("hand-left-topic", "左手 joint_commands", withCurrentOption(topicChoices, settings.leftTopic), { valueKey: "value", selected: settings.leftTopic })}
        ${buildSelect("hand-right-topic", "右手 joint_commands", withCurrentOption(topicChoices, settings.rightTopic), { valueKey: "value", selected: settings.rightTopic })}
        <label>左手 set_enabled<input id="hand-left-set-enabled" value="${escapeHtml(settings.leftSetEnabled)}"></label>
        <label>右手 set_enabled<input id="hand-right-set-enabled" value="${escapeHtml(settings.rightSetEnabled)}"></label>
        <label>左手 reset_error<input id="hand-left-reset-error" value="${escapeHtml(settings.leftResetError)}"></label>
        <label>右手 reset_error<input id="hand-right-reset-error" value="${escapeHtml(settings.rightResetError)}"></label>
      </div>
    </section>
    <section class="settings-section">
      <h3>滑条参数</h3>
      <div class="settings-fields">
        <label>最小值<input id="hand-slider-min" type="number" step="0.001" value="${escapeHtml(settings.min)}"></label>
        <label>最大值<input id="hand-slider-max" type="number" step="0.001" value="${escapeHtml(settings.max)}"></label>
        <label>步长<input id="hand-slider-step" type="number" step="0.001" value="${escapeHtml(settings.step)}"></label>
        <label>发送防抖 ms<input id="hand-throttle-ms" type="number" step="10" value="${escapeHtml(settings.throttleMs)}"></label>
      </div>
    </section>
  `;
}

function poseArrayText(values) {
  return `[${values.map((value) => Number(Number(value || 0).toFixed(6))).join(", ")}]`;
}

function parsePoseArrayText(rawText) {
  const text = String(rawText || "").trim();
  if (!text) throw new Error("请先粘贴 20 维数组");
  let parsed = null;
  try {
    parsed = JSON.parse(text);
  } catch {
    parsed = null;
  }
  let values = [];
  if (Array.isArray(parsed)) {
    values = parsed;
  } else if (parsed && Array.isArray(parsed.values)) {
    values = parsed.values;
  } else {
    values = (text.match(/[-+]?(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?/gi) || []).map((value) => Number(value));
  }
  if (values.length !== HAND_JOINT_COUNT) throw new Error(`需要 20 个数字，当前识别到 ${values.length} 个`);
  return values.map((value, index) => {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) throw new Error(`第 ${index + 1} 个值不是有效数字`);
    return numeric;
  });
}

async function copyText(text) {
  if (navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(text);
    return;
  }
  const input = document.createElement("textarea");
  input.value = text;
  input.setAttribute("readonly", "");
  input.style.position = "fixed";
  input.style.opacity = "0";
  document.body.appendChild(input);
  input.select();
  document.execCommand("copy");
  document.body.removeChild(input);
}

async function init() {
  if (isEmbedded) document.body.classList.add("embedded-module");
  document.title = `FKViewer - ${moduleInfo.title}`;
  setText("#window-title", moduleInfo.title);
  setText("#window-subtitle", moduleInfo.subtitle);
  initRuntimeLogControls();
  await loadOptions();
  await refresh();
  window.setInterval(() => refresh(), 2500);
}

function initRuntimeLogControls() {
  for (const key of ["arm", "hand"]) {
    const panel = $(`#runtime-log-${key}`);
    if (!panel) continue;
    updateLogFollowButton(key);
    panel.addEventListener("scroll", () => {
      if (panel.dataset.programmaticScroll) return;
      const distanceFromBottom = panel.scrollHeight - panel.scrollTop - panel.clientHeight;
      setLogFollow(key, distanceFromBottom <= 24);
    });
  }
}

async function loadOptions() {
  optionsCache = await api(`/api/options?module=${encodeURIComponent(moduleName)}`);
  const roots = optionsCache.roots || {};
  setText("#config-root", roots[moduleName] || roots.inference || "--");
  if (moduleName === "impedance") renderImpedanceConfig(optionsCache.impedance);
  if (moduleName === "hand") {
    await hydrateSelectedHandConfig();
    renderHandConfig(optionsCache.hand);
  }
  if (moduleName === "inference") renderInferenceConfig(optionsCache.inference);
  if (moduleName === "teleop") renderTeleopConfig(optionsCache.teleop);
  if (moduleName === "unified") renderUnifiedConfig(optionsCache.unified);
}

async function hydrateSelectedHandConfig() {
  const settings = currentHandSettings();
  if (!settings.wujiConfig) return;
  try {
    const data = await api(`/api/hand_config?path=${encodeURIComponent(settings.wujiConfig)}`);
    optionsCache.hand = {
      ...(optionsCache.hand || {}),
      config_file: data.path,
      poses: data.poses || {},
      params: data.params || {},
    };
  } catch (error) {
    toast(`灵巧手姿态配置读取失败: ${error.message}`, false);
  }
}

async function refresh() {
  if (refreshPending) return;
  refreshPending = true;
  try {
    const nextStatus = await api("/api/status");
    statusCache = nextStatus;
    const live = $("#live-chip");
    live.className = statusCache.live_control ? "chip err" : "chip warn";
    live.innerHTML = `<span class="dot"></span>${statusCache.live_control ? "真机模式" : "仿真模式"}`;
    setText("#action-safety-note", moduleName === "unified"
      ? `${statusCache.live_control ? "真机模式" : "仿真模式"}：先启动统一栈；推理客户端与 Quest 层随后可任选顺序或只启动一个`
      : statusCache.live_control
        ? "真机控制已启用，灵巧手滑条变化会实时下发"
        : "仿真模式下只预览命令，不会操作真机");
    const refreshChip = $("#refresh-chip");
    refreshChip.className = "chip ok";
    refreshChip.innerHTML = `<span class="dot"></span>${statusCache.now.split(" ").pop()}`;
    monitorDeviceConnections(statusCache);
    detectProcessTransitions(statusCache.managed_processes || {});
    renderStatus();
    renderEvents(statusCache.events || []);
    syncProcessButtons();
    if (["teleop", "impedance", "hand", "inference", "unified"].includes(moduleName)) await refreshRuntimeLogs();
    if (connectionFailed) {
      toast("已恢复与 FKViewer 服务的连接");
      setOperation("success", "连接恢复", "状态自动刷新已恢复");
    }
    connectionFailed = false;
  } catch (error) {
    const chip = $("#refresh-chip");
    if (chip) {
      chip.className = "chip err";
      chip.innerHTML = '<span class="dot"></span>连接失败';
    }
    setOperation("error", "连接失败", `无法获取最新状态：${error.message}`);
    if (!connectionFailed) toast(`状态刷新失败：${error.message}`, false);
    connectionFailed = true;
  } finally {
    refreshPending = false;
  }
}

function processDisplayName(name) {
  return ({
    impedance_franka_stack: "Franka Stack",
    impedance_policy_stack: "Policy Stack",
    impedance_selected: "SERL Launch",
    easydp_client: "双臂推理客户端",
    easydp_reset: "一键恢复双臂位置",
    hand_selected: "灵巧手 ROS2 Driver",
    wuji_trigger_service: "Quest Bridge",
    easydp_server: "推理服务",
    easydp_client_debug: "双臂调试客户端",
    policy_profile: "机器人 Policy Profile",
    teleop_terminal1_franka_stack: "机械臂控制栈",
    teleop_terminal2_quest: "手柄摇操层",
    unified_control_stack: "统一阻抗与仲裁栈",
    unified_inference_client: "统一栈推理客户端",
    unified_quest_layer: "Quest 接管与录制层",
  })[name] || name;
}

function monitorDeviceConnections(status) {
  const managed = status.managed_processes || {};
  const detected = status.detected_processes || {};
  const anyRunning = (names) => names.some((name) => managed[name]?.running || (detected[name] || []).length);
  const armStackRunning = anyRunning([
    "impedance_franka_stack",
    "impedance_policy_stack",
    "impedance_selected",
    "teleop_terminal1_franka_stack",
    "franka_stack",
    "unified_control_stack",
  ]);
  const devices = {
    left_arm: {
      label: "左臂 HTTP 服务",
      online: Boolean(status.arms?.left?.health?.ok),
      expected: armStackRunning,
      detail: status.arms?.left?.health?.error,
    },
    right_arm: {
      label: "右臂 HTTP 服务",
      online: Boolean(status.arms?.right?.health?.ok),
      expected: armStackRunning,
      detail: status.arms?.right?.health?.error,
    },
    wuji: {
      label: "Wuji 服务",
      online: Boolean(status.wuji?.health?.ok),
      expected: anyRunning(["wuji_trigger_service", "wuji", "teleop_terminal2_quest"]),
      detail: status.wuji?.health?.error,
    },
  };

  for (const [key, device] of Object.entries(devices)) {
    const wasOnline = previousDeviceConnections[key];
    if (!device.online && (device.expected || wasOnline === true) && !announcedDisconnections.has(key)) {
      const detail = device.detail ? `：${String(device.detail).slice(0, 180)}` : "";
      const message = `${device.label}连接已断开${detail}`;
      announcedDisconnections.add(key);
      toast(message, false);
      setOperation("error", "设备断开", message);
    } else if (device.online && announcedDisconnections.has(key)) {
      announcedDisconnections.delete(key);
      const message = `${device.label}连接已恢复`;
      toast(message);
      setOperation("success", "连接恢复", message);
    }
    previousDeviceConnections[key] = device.online;
  }
}

function detectProcessTransitions(managed) {
  if (previousManagedProcesses) {
    for (const [name, before] of Object.entries(previousManagedProcesses)) {
      const after = managed[name];
      if (before?.running && after && !after.running) {
        const success = after.returncode === 0 || after.returncode === -2;
        const message = `${processDisplayName(name)}已结束${after.returncode == null ? "" : `，退出码 ${after.returncode}`}`;
        setOperation(success ? "success" : "warning", "运行结束", message);
        toast(message, success ? true : "warn");
      }
    }
  }
  previousManagedProcesses = JSON.parse(JSON.stringify(managed));
}

function syncProcessButtons() {
  const managed = statusCache?.managed_processes || {};
  document.querySelectorAll("button[data-launch]").forEach((button) => {
    if (button.classList.contains("is-busy")) return;
    const running = isProcessRunning(button.dataset.launch);
    button.disabled = running;
    button.title = running ? `${processDisplayName(button.dataset.launch)}正在运行` : "";
  });
  document.querySelectorAll("button[data-stop]").forEach((button) => {
    if (button.classList.contains("is-busy")) return;
    const item = managed[button.dataset.stop];
    button.disabled = !item?.running;
    button.title = item?.running ? `停止 PID ${item.pid}` : "没有由 FKViewer 启动的运行实例";
  });
  syncInferenceWorkflow();
  syncUnifiedWorkflow();
}

function syncUnifiedWorkflow() {
  if (moduleName !== "unified" || !statusCache) return;
  const stackRunning = isProcessRunning("unified_control_stack");
  const clientRunning = isProcessRunning("unified_inference_client");
  const questRunning = isProcessRunning("unified_quest_layer");
  const ready = inferencePrerequisites();
  const teleopActive = Boolean(statusCache.unified?.teleop_active);
  for (const name of ["unified_inference_client", "unified_quest_layer"]) {
    const button = document.querySelector(`button[data-launch="${name}"]`);
    if (!button || button.classList.contains("is-busy") || isProcessRunning(name)) continue;
    button.disabled = !stackRunning || !ready.all || (name === "unified_inference_client" && teleopActive);
    button.title = !stackRunning
      ? "请先启动统一阻抗与仲裁栈"
      : !ready.all
        ? "等待左右臂 HTTP 与 Wuji Driver 就绪"
        : teleopActive && name === "unified_inference_client"
          ? "当前为摇操控制权，请先按 Y 交还推理"
          : "";
  }
  const stopStack = document.querySelector('button[data-stop="unified_control_stack"]');
  if (stopStack && !stopStack.classList.contains("is-busy") && (clientRunning || questRunning)) {
    stopStack.disabled = true;
    stopStack.title = "请先停止推理客户端和 Quest 接管与录制层";
  }
}

function isProcessRunning(name, status = statusCache) {
  if (!status) return false;
  return Boolean(status.managed_processes?.[name]?.running || status.detected_processes?.[name]?.length);
}

function inferencePrerequisites(status = statusCache) {
  const left = Boolean(status?.arms?.left?.health?.ok);
  const right = Boolean(status?.arms?.right?.health?.ok);
  const wuji = Boolean(status?.wuji?.health?.ok);
  return { left, right, wuji, all: left && right && wuji };
}

function updateStateChip(selector, state, label) {
  const chip = $(selector);
  if (!chip) return;
  chip.className = `chip ${state}`.trim();
  chip.innerHTML = `<span class="dot"></span>${label}`;
}

function syncInferenceWorkflow() {
  if (moduleName !== "inference" || !statusCache) return;
  const policyRunning = isProcessRunning("impedance_policy_stack");
  const clientRunning = isProcessRunning("easydp_client");
  const resetRunning = isProcessRunning("easydp_reset");
  const ready = inferencePrerequisites();
  updateStateChip(
    "#inference-impedance-state",
    ready.all ? "ok" : policyRunning ? "warn" : "",
    ready.all ? "服务已就绪" : policyRunning ? "启动中" : "未运行",
  );
  updateStateChip(
    "#inference-reset-state",
    resetRunning ? "warn" : ready.all ? "ok" : "",
    resetRunning ? "复位中" : ready.all ? "可以复位" : "等待前置服务",
  );
  updateStateChip(
    "#inference-client-state",
    clientRunning ? "ok" : ready.all ? "warn" : "",
    clientRunning ? "推理运行中" : ready.all ? "可以启动" : "等待前置服务",
  );
  const clientButton = document.querySelector('button[data-launch="easydp_client"]');
  if (clientButton && !clientButton.classList.contains("is-busy")) {
    clientButton.disabled = clientRunning || resetRunning || !ready.all || inferenceWorkflowBusy;
    clientButton.title = resetRunning
      ? "双臂正在复位，请等待复位完成"
      : !ready.all ? "双臂 HTTP 与 Wuji 服务就绪后才能启动" : "";
  }
  const policyStop = document.querySelector('button[data-stop="impedance_policy_stack"]');
  if (policyStop && !policyStop.classList.contains("is-busy") && (clientRunning || resetRunning)) {
    policyStop.disabled = true;
    policyStop.title = resetRunning ? "请等待双臂复位完成" : "请先停止双臂推理客户端";
  }
  const startAll = document.querySelector("[data-inference-start-all]");
  if (startAll && !startAll.classList.contains("is-busy")) startAll.disabled = clientRunning || resetRunning || inferenceWorkflowBusy;
  const stopAll = document.querySelector("[data-inference-stop-all]");
  if (stopAll && !stopAll.classList.contains("is-busy")) {
    stopAll.disabled = resetRunning || inferenceWorkflowBusy || !(statusCache.managed_processes?.easydp_client?.running || statusCache.managed_processes?.impedance_policy_stack?.running);
    stopAll.title = resetRunning ? "请等待双臂复位完成" : "";
  }
}

function renderImpedanceConfig(options) {
  document.body.classList.add("impedance-simple");
  $("#config-panel")?.classList.add("hidden");
  $("#runtime-log-panel")?.classList.remove("hidden");
  $("#settings-button")?.classList.remove("hidden");
  setText("#status-subtitle", "Franka HTTP / 阻抗控制栈实时探测");
  setText("#runtime-log-arm-title", "Franka Stack 日志");
  setText("#runtime-log-hand-title", "Policy / SERL 日志");
  $("#config-form").innerHTML = "";
  $("#action-area").innerHTML = `
    <div class="teleop-actions impedance-actions">
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">双臂 Franka Stack</div>
          <div class="teleop-card-sub">profile: franka_stack / simple_dual_split_launch.yaml</div>
        </div>
        <button class="primary teleop-main-button" data-launch="impedance_franka_stack">启动 Franka Stack</button>
        <button class="danger" data-stop="impedance_franka_stack">停止 Franka Stack</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">Policy 阻抗栈</div>
          <div class="teleop-card-sub">profile: policy_inference / simple_dual_split_launch.yaml</div>
        </div>
        <button class="primary teleop-main-button" data-launch="impedance_policy_stack">启动 Policy Stack</button>
        <button class="danger" data-stop="impedance_policy_stack">停止 Policy Stack</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">SERL 所选 Launch</div>
          <div class="teleop-card-sub">由设置中的 Launch 下拉决定</div>
        </div>
        <button class="primary teleop-main-button" data-launch="impedance_selected">启动所选 Launch</button>
        <button class="danger" data-stop="impedance_selected">停止所选 Launch</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">控制器操作</div>
          <div class="teleop-card-sub">作用于设置中的目标手臂</div>
        </div>
        <button data-imp-op="start_impedance">Start Impedance</button>
        <button data-imp-op="stop_impedance">Stop Impedance</button>
        <button data-imp-op="clear_error">Clear Error</button>
        <button data-imp-params>应用刚度/阻尼</button>
      </div>
    </div>
  `;
}

function handPose(side, poseName) {
  return (optionsCache?.hand?.poses || {})[`${side}_${poseName}_pose`] || [];
}

function renderJointSliders(side) {
  const settings = currentHandSettings();
  const min = Number(settings.min);
  const max = Number(settings.max);
  const step = Number(settings.step);
  return handValues[side].map((value, index) => `
    <label class="joint-slider" data-hand-joint="${side}:${index}">
      <div class="slider-head">
        <span>J${String(index + 1).padStart(2, "0")}</span>
        <input class="joint-number" type="number" step="${Number.isFinite(step) ? step : 0.001}" value="${Number(value || 0).toFixed(3)}">
      </div>
      <input class="joint-range" type="range" min="${Number.isFinite(min) ? min : -0.5}" max="${Number.isFinite(max) ? max : 1.65}" step="${Number.isFinite(step) ? step : 0.001}" value="${Number(value || 0)}">
    </label>
  `).join("");
}

function renderHandCard(side, title) {
  const settings = currentHandSettings();
  const topic = side === "left" ? settings.leftTopic : settings.rightTopic;
  return `
    <div class="hand-card" data-hand-card="${side}">
      <div class="hand-card-head">
        <div>
          <div class="teleop-card-title">${title}</div>
          <div class="teleop-card-sub">${escapeHtml(topic)}</div>
        </div>
        <span class="chip ok"><span class="dot"></span>拖动即发送</span>
      </div>
      <div class="hand-actions">
        <button class="primary" data-hand-side="${side}" data-hand-pose="released">打开</button>
        <button class="primary" data-hand-side="${side}" data-hand-pose="closed">关闭</button>
        <button data-hand-side="${side}" data-hand-copy>复制当前</button>
        <button data-hand-side="${side}" data-hand-op="enable">使能</button>
        <button class="danger" data-hand-side="${side}" data-hand-op="disable">关闭使能</button>
        <button data-hand-side="${side}" data-hand-op="reset_error">Reset Error</button>
      </div>
      <div class="array-import">
        <textarea id="${side}-pose-import" placeholder="粘贴 20 维数组或 {&quot;values&quot;:[...]}"></textarea>
        <button data-hand-side="${side}" data-hand-import>导入并运行</button>
      </div>
      <div class="joint-grid" id="${side}-joint-grid">
        ${renderJointSliders(side)}
      </div>
    </div>
  `;
}

function refreshHandSliders(side) {
  const target = document.getElementById(`${side}-joint-grid`);
  if (target) target.innerHTML = renderJointSliders(side);
}

function renderHandConfig(options) {
  document.body.classList.add("hand-simple");
  $("#config-panel")?.classList.add("hidden");
  $("#runtime-log-panel")?.classList.remove("hidden");
  $("#settings-button")?.classList.remove("hidden");
  setText("#status-subtitle", "官方 wujihandros2 driver / joint_commands 实时探测");
  setText("#runtime-log-arm-title", "官方 ROS2 Driver 日志");
  setText("#runtime-log-hand-title", "Quest Bridge 日志");
  const leftOpen = handPose("left", "released");
  const rightOpen = handPose("right", "released");
  if (leftOpen.length === HAND_JOINT_COUNT) handValues.left = leftOpen.map(Number);
  if (rightOpen.length === HAND_JOINT_COUNT) handValues.right = rightOpen.map(Number);
  $("#config-form").innerHTML = "";
  $("#action-area").innerHTML = `
    <div class="hand-launch-strip">
      <button class="primary" data-launch="hand_selected">启动官方 ROS2 Driver</button>
      <button class="danger" data-stop="hand_selected">停止 Driver</button>
      <button data-launch="wuji_trigger_service">启动 Quest Bridge</button>
      <button class="danger" data-stop="wuji_trigger_service">停止 Bridge</button>
    </div>
    <div class="hand-launch-strip hand-batch-strip">
      <button class="primary" data-hand-both-pose="released">打开双手</button>
      <button class="primary" data-hand-both-pose="closed">关闭双手</button>
    </div>
    <div class="hand-panel-grid">
      ${renderHandCard("left", "左手")}
      ${renderHandCard("right", "右手")}
    </div>
  `;
}

function renderInferenceConfig(options) {
  document.body.classList.add("inference-simple");
  $("#config-panel")?.classList.add("hidden");
  $("#runtime-log-panel")?.classList.remove("hidden");
  $("#settings-button")?.classList.add("hidden");
  setText("#status-subtitle", "推理阻抗、双臂 HTTP、Wuji 与推理客户端状态");
  setText("#runtime-log-arm-title", "推理阻抗日志");
  setText("#runtime-log-hand-title", "双臂推理日志");
  $("#config-form").innerHTML = "";
  $("#action-area").innerHTML = `
    <div class="inference-workflow">
      <div class="inference-step" data-inference-step="impedance">
        <div class="step-number">1</div>
        <div class="step-content">
          <div class="teleop-card-title">启动推理阻抗</div>
          <div class="teleop-card-sub">profile: policy_inference / 等待双臂与 Wuji 就绪</div>
        </div>
        <span class="chip" id="inference-impedance-state"><span class="dot"></span>未运行</span>
        <div class="step-actions">
          <button class="primary" data-launch="impedance_policy_stack">启动推理阻抗</button>
          <button class="danger" data-stop="impedance_policy_stack">停止推理阻抗</button>
        </div>
      </div>
      <div class="workflow-connector"></div>
      <div class="inference-step" data-inference-step="client">
        <div class="step-number">2</div>
        <div class="step-content">
          <div class="teleop-card-title">启动双臂推理</div>
          <div class="teleop-card-sub">${escapeHtml(options.client_file || "projects/task_insertion_stage2/client/client_dual.py")}</div>
        </div>
        <span class="chip" id="inference-client-state"><span class="dot"></span>等待前置服务</span>
        <div class="step-actions">
          <button class="primary" data-launch="easydp_client">启动双臂推理</button>
          <button class="danger" data-stop="easydp_client">停止双臂推理</button>
        </div>
      </div>
      <div class="inference-step" data-inference-step="reset">
        <div class="step-number">↺</div>
        <div class="step-content">
          <div class="teleop-card-title">一键恢复双臂位置（可选）</div>
          <div class="teleop-card-sub">scripts/reset.sh / 推理未运行且前置服务就绪后可执行</div>
        </div>
        <span class="chip" id="inference-reset-state"><span class="dot"></span>等待前置服务</span>
        <div class="step-actions">
          <button class="primary span-all" data-inference-reset data-launch="easydp_reset">恢复机械臂位置</button>
        </div>
      </div>
      <div class="workflow-actions">
        <button class="primary" data-inference-start-all>按顺序启动全部</button>
        <button class="danger" data-inference-stop-all>按顺序停止全部</button>
      </div>
    </div>
  `;
}

function renderTeleopConfig(options) {
  document.body.classList.add("teleop-simple");
  $("#config-panel")?.classList.add("hidden");
  $("#runtime-log-panel")?.classList.remove("hidden");
  $("#settings-button")?.classList.remove("hidden");
  setText("#status-subtitle", "机械臂控制栈 / 手柄摇操实时探测");
  $("#action-area").innerHTML = `
    <div class="teleop-actions">
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">机械臂控制栈</div>
          <div class="teleop-card-sub">左右 Franka / impedance / HTTP</div>
        </div>
        <button class="primary teleop-main-button" data-launch="teleop_terminal1_franka_stack">启动机械臂</button>
        <button class="danger" data-stop="teleop_terminal1_franka_stack">停止机械臂程序</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">手柄摇操层</div>
          <div class="teleop-card-sub">Quest reader / teleop / Wuji / recorder</div>
        </div>
        <label class="teleop-keep-close">
          <input id="teleop-keep-close" type="checkbox">
          <span>keep close（保持双手关闭并忽略手柄控制）</span>
        </label>
        <button class="primary teleop-main-button" data-launch="teleop_terminal2_quest">启动手柄</button>
        <button class="danger" data-stop="teleop_terminal2_quest">停止手柄程序</button>
      </div>
    </div>
  `;
}

function renderUnifiedConfig(options) {
  document.body.classList.add("unified-simple");
  $("#config-panel")?.classList.add("hidden");
  $("#runtime-log-panel")?.classList.remove("hidden");
  $("#settings-button")?.classList.add("hidden");
  setText("#status-subtitle", "共享阻抗控制 / Y 键控制权 / 原格式录制实时状态");
  setText("#runtime-log-arm-title", "统一阻抗与仲裁栈日志");
  setText("#runtime-log-hand-title", "推理 / Quest 接管与录制日志");
  $("#config-form").innerHTML = "";
  $("#action-area").innerHTML = `
    <div class="teleop-actions unified-actions">
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">① 统一阻抗与仲裁栈（必须先启动）</div>
          <div class="teleop-card-sub">推理与 Quest 的共同前置：双臂阻抗 / HTTP Gate / Wuji Driver</div>
        </div>
        <button class="primary teleop-main-button" data-launch="unified_control_stack">启动统一控制栈</button>
        <button class="danger" data-stop="unified_control_stack">停止统一控制栈</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">②A EasyDP 推理客户端（可选）</div>
          <div class="teleop-card-sub">使用 5000 / 5001 / 8765 仲裁入口；摇操接管时请求会被拒绝</div>
        </div>
        <button class="primary teleop-main-button" data-launch="unified_inference_client">启动推理客户端</button>
        <button class="danger" data-stop="unified_inference_client">停止推理客户端</button>
      </div>
      <div class="teleop-action-card">
        <div>
          <div class="teleop-card-title">②B Quest 接管与原格式录制层（可选）</div>
          <div class="teleop-card-sub">Y 切换接管；A 开始、B 停止、X 删除，HDF5 数据链路不变</div>
        </div>
        <button class="primary teleop-main-button" data-launch="unified_quest_layer">启动 Quest 层</button>
        <button class="danger" data-stop="unified_quest_layer">停止 Quest 层</button>
      </div>
    </div>
  `;
}

function moduleSelections() {
  if (moduleName === "impedance") {
    const settings = currentImpedanceSettings();
    return {
      impedance_launch: settings.launch,
      profile_file: optionsCache?.impedance?.profile_file || "",
    };
  }
  if (moduleName === "hand") {
    const settings = currentHandSettings();
    return {
      hand_launch: settings.launch,
      wuji_config: settings.wujiConfig,
      wuji_policy: settings.wujiPolicy,
      left_command_topic: settings.leftTopic,
      right_command_topic: settings.rightTopic,
      left_set_enabled_service: settings.leftSetEnabled,
      right_set_enabled_service: settings.rightSetEnabled,
      left_reset_error_service: settings.leftResetError,
      right_reset_error_service: settings.rightResetError,
      hand_rviz: settings.rviz,
      hand_foxglove: settings.foxglove,
      hand_name: settings.handName,
      hand_names: settings.handNames,
      hand_serial: settings.handSerial,
      hand_side: settings.handSide,
      hand_publish_rate: settings.publishRate,
      hand_filter_cutoff_freq: settings.filterCutoff,
      hand_diagnostics_rate: settings.diagnosticsRate,
    };
  }
  if (moduleName === "teleop") {
    return {
      profile_file: optionsCache?.teleop?.profile_file || "",
      keep_close: Boolean($("#teleop-keep-close")?.checked),
    };
  }
  return {};
}

function renderStatus() {
  if (moduleName === "impedance") {
    $("#module-status").innerHTML = `
      ${renderArmStatus(statusCache.arms || {})}
      ${renderProcessSubset(["impedance_franka_stack", "impedance_policy_stack", "impedance_selected"])}
    `;
  } else if (moduleName === "hand") {
    const settings = currentHandSettings();
    $("#module-status").innerHTML = `
      <div class="kv">
        <span>Left Topic</span><strong>${escapeHtml(settings.leftTopic)}</strong>
        <span>Right Topic</span><strong>${escapeHtml(settings.rightTopic)}</strong>
        <span>Config</span><strong>${escapeHtml(settings.wujiConfig || "--")}</strong>
      </div>
      ${renderProcessSubset(["hand_selected", "wujihand_driver", "wuji"])}
    `;
  } else if (moduleName === "inference") {
    const ready = inferencePrerequisites(statusCache);
    $("#module-status").innerHTML = `
      ${renderArmStatus(statusCache.arms || {})}
      <div class="inference-readiness">
        <span class="chip ${ready.left ? "ok" : "err"}"><span class="dot"></span>左臂 HTTP</span>
        <span class="chip ${ready.right ? "ok" : "err"}"><span class="dot"></span>右臂 HTTP</span>
        <span class="chip ${ready.wuji ? "ok" : "err"}"><span class="dot"></span>Wuji 服务</span>
        <strong>${ready.all ? "推理前置服务已就绪" : "请先启动推理阻抗并等待服务就绪"}</strong>
      </div>
      ${renderProcessSubset(["impedance_policy_stack", "easydp_reset", "easydp_client"])}
    `;
  } else if (moduleName === "unified") {
    const authority = statusCache.unified || {};
    const teleop = authority.authority === "teleop";
    const questRunning = isProcessRunning("unified_quest_layer");
    const recording = questRunning && Boolean(authority.recording);
    $("#module-status").innerHTML = `
      ${renderArmStatus(statusCache.arms || {})}
      <div class="inference-readiness">
        <span class="chip ${authority.ok ? (teleop ? "warn" : "ok") : "err"}"><span class="dot"></span>${authority.ok ? (teleop ? "摇操控制权" : "推理控制权") : "仲裁器离线"}</span>
        <span class="chip ${statusCache.wuji?.health?.ok ? "ok" : "err"}"><span class="dot"></span>Wuji ${statusCache.wuji?.health?.ok ? "就绪" : "未就绪"}</span>
        <span class="chip ${questRunning ? (recording ? "err" : "ok") : ""}"><span class="dot"></span>${!questRunning ? "录制节点未运行" : recording ? "正在录制" : "录制待命"}</span>
        <strong>${teleop ? "推理机械臂 /pose 与 Wuji HTTP 命令已拦截" : "Quest 机械臂与 Wuji 指令已隔离"}</strong>
      </div>
      <div class="kv">
        <span>切换方式</span><strong>Quest Y 键（上升沿切换）</strong>
        <span>默认控制权</span><strong>推理</strong>
        <span>最近 Episode</span><strong>${escapeHtml(authority.last_episode || "--")}</strong>
        <span>录制输出</span><strong>${escapeHtml(optionsCache?.unified?.record_out_dir || "/home/lumos/quest3_recordings")}</strong>
      </div>
      ${renderProcessSubset(["unified_control_stack", "unified_inference_client", "unified_quest_layer"])}
    `;
  } else {
    $("#module-status").innerHTML = renderProcessSubset([
      "teleop_terminal1_franka_stack",
      "teleop_terminal2_quest",
    ]);
  }
}

function renderArmStatus(arms) {
  return `<div class="arm-grid">${["left", "right"].map((arm) => {
    const item = arms[arm] || {};
    const body = item.state || {};
    return `
      <div class="arm-box">
        <div class="arm-top">
          <div class="arm-title">${arm}</div>
          <span class="${item.health?.ok ? "chip ok" : "chip err"}"><span class="dot"></span>${item.health?.ok ? "在线" : "离线"}</span>
        </div>
        <div class="arm-metrics">
          <span>URL</span><strong>${escapeHtml(item.url || "--")}</strong>
          <span>Pose</span><strong>${fmtArray(body.pose)}</strong>
          <span>Force</span><strong>${fmtArray(body.force)}</strong>
          <span>Joint</span><strong>${fmtArray(body.q)}</strong>
        </div>
      </div>`;
  }).join("")}</div>`;
}

function renderProcessSubset(keys) {
  const detected = statusCache.detected_processes || {};
  const managed = statusCache.managed_processes || {};
  return `<div class="process-list">${keys.map((key) => {
    const lines = detected[key] || [];
    const process = managed[key];
    const running = Boolean(process?.running);
    const stateText = running
      ? `运行中 · PID ${process.pid}`
      : process ? `已结束 · 退出码 ${process.returncode ?? "--"}` : (lines.length ? "外部进程已检测" : "未运行");
    const stateClass = running ? "running" : process ? "exited" : "";
    return `
      <div class="proc-row">
        <div class="proc-head">
          <div class="proc-name">${escapeHtml(processDisplayName(key))}</div>
          <div class="proc-state ${stateClass}">${stateText}</div>
        </div>
        ${lines.length ? lines.map((line) => `<div class="proc-line">${escapeHtml(line)}</div>`).join("") : '<div class="empty">未检测到相关系统进程</div>'}
      </div>`;
  }).join("")}</div>`;
}

function fmtArray(values) {
  if (!Array.isArray(values)) return "--";
  return values.slice(0, 3).map((value) => Number(value).toFixed(3)).join(", ");
}

function renderEvents(events) {
  $("#event-log").innerHTML = events.slice().reverse().map((event) => `
    <div class="log-row">
      <div class="log-time">${escapeHtml(event.time || "--")}</div>
      <div class="log-level">${escapeHtml(event.level || "")}</div>
      <div class="log-message">${escapeHtml(event.message || "")}</div>
    </div>
  `).join("") || '<div class="log-row"><div></div><div></div><div class="empty">暂无操作记录</div></div>';
}

function filterRuntimeLog(text) {
  return String(text || "").split("\n").filter((line) => {
    const routineHttpGet = /\bHTTP\s+GET\b/i.test(line);
    const accessLogGet = /"GET\s+\/[^\s]*\s+HTTP\/[\d.]+"\s+\d{3}/i.test(line);
    return !routineHttpGet && !accessLogGet;
  }).join("\n").trimEnd();
}

function updateLogFollowButton(key) {
  const button = document.querySelector(`[data-log-follow="${key}"]`);
  if (!button) return;
  const following = runtimeLogFollow[key];
  button.textContent = following ? "暂停跟随" : "回到底部";
  button.classList.toggle("is-paused", !following);
  button.title = following ? "暂停自动跟随最新日志" : "回到日志底部并继续自动跟随";
}

function setLogFollow(key, following, { scroll = false } = {}) {
  runtimeLogFollow[key] = following;
  const panel = $(`#runtime-log-${key}`);
  if (following && scroll && panel) {
    panel.dataset.programmaticScroll = "1";
    panel.scrollTop = panel.scrollHeight;
    window.requestAnimationFrame(() => delete panel.dataset.programmaticScroll);
  }
  updateLogFollowButton(key);
}

function renderRuntimeLog(panel, text, key, source = key) {
  const previousScrollTop = panel.scrollTop;
  const cache = runtimeLogCache[key];
  if (cache.source !== source) {
    cache.source = source;
    cache.text = "";
  }
  const filtered = filterRuntimeLog(text);
  if (filtered) cache.text = filtered;
  panel.dataset.programmaticScroll = "1";
  panel.textContent = filtered || cache.text || "当前日志仅包含已隐藏的 HTTP GET 健康探测记录。";
  if (runtimeLogFollow[key]) {
    panel.scrollTop = panel.scrollHeight;
  } else {
    panel.scrollTop = previousScrollTop;
  }
  window.requestAnimationFrame(() => delete panel.dataset.programmaticScroll);
}

async function refreshRuntimeLogs() {
  const armPanel = $("#runtime-log-arm");
  const handPanel = $("#runtime-log-hand");
  if (!armPanel || !handPanel) return;
  let targets = ["teleop_terminal1_franka_stack", "teleop_terminal2_quest"];
  if (moduleName === "impedance") {
    targets = ["impedance_franka_stack", "impedance_policy_stack", "impedance_selected"];
  } else if (moduleName === "hand") {
    targets = ["hand_selected", "wuji_trigger_service"];
  } else if (moduleName === "inference") {
    targets = ["impedance_policy_stack", "easydp_client", "easydp_reset"];
  } else if (moduleName === "unified") {
    targets = ["unified_control_stack", "unified_inference_client", "unified_quest_layer"];
  }
  try {
    const data = await api(`/api/logs?names=${targets.join(",")}`);
    if (runtimeLogReadFailed) toast("运行日志连接已恢复");
    runtimeLogReadFailed = false;
    const logEntry = (name) => {
      const item = data.logs?.[name] || {};
      if (item.ok) {
        return { text: String(item.text || "").trimEnd(), source: `${name}:${item.path || "active"}` };
      }
      return {
        text: item.error || "尚未由 FKViewer 启动，暂无运行日志。",
        source: `${name}:unavailable`,
      };
    };
    if (["impedance", "inference", "unified"].includes(moduleName)) {
      const armEntry = logEntry(targets[0]);
      const secondaryEntries = targets.slice(1).map((name) => ({ name, ...logEntry(name) }));
      renderRuntimeLog(armPanel, armEntry.text, "arm", armEntry.source);
      renderRuntimeLog(
        handPanel,
        secondaryEntries.map((entry) => `[${entry.name}]\n${entry.text}`).join("\n\n"),
        "hand",
        secondaryEntries.map((entry) => entry.source).join("|"),
      );
      return;
    }
    const armEntry = logEntry(targets[0]);
    const handEntry = logEntry(targets[1]);
    renderRuntimeLog(armPanel, armEntry.text, "arm", armEntry.source);
    renderRuntimeLog(handPanel, handEntry.text, "hand", handEntry.source);
  } catch (error) {
    if (!runtimeLogReadFailed) toast(`日志读取失败：${error.message}`, false);
    runtimeLogReadFailed = true;
  }
}

async function openTeleopSettings() {
  settingsCache = await api("/api/yaml?target=teleop_split");
  setText("#settings-title", "摇操参数设置");
  setText("#settings-path", settingsCache.path || "--");
  $(".settings-note").textContent = "保存会写入源目录 YAML。已经运行中的 ROS launch 不会自动重载参数，需要先停止对应程序，再重新启动。";
  $("#settings-form").innerHTML = (settingsCache.profiles || []).map((profile) => `
    <section class="settings-section">
      <h3>${escapeHtml(profile.name)}</h3>
      <div class="settings-fields">
        ${(profile.params || []).map((param) => `
          <label>${escapeHtml(param.key)}
            <input data-profile="${escapeHtml(profile.name)}" data-key="${escapeHtml(param.key)}" value="${escapeHtml(param.value)}">
          </label>
        `).join("")}
      </div>
    </section>
  `).join("");
  $("#settings-modal").classList.remove("hidden");
}

async function openImpedanceSettings() {
  settingsCache = await api("/api/yaml?target=impedance_profiles");
  const settings = currentImpedanceSettings();
  setText("#settings-title", "阻抗参数设置");
  setText("#settings-path", settingsCache.path || "--");
  $(".settings-note").textContent = "保存会写入源目录 simple_dual_split_launch.yaml。Franka Stack 和 Policy Stack 启动时会显式传入这个源文件路径；已运行的 ROS launch 不会热更新，需要先停止再重新启动。";
  $("#settings-form").innerHTML = `
    ${impedanceSettingsMarkup(optionsCache?.impedance || {}, settings)}
    ${impedanceProfileSettingsMarkup(settingsCache, optionsCache?.impedance || {})}
  `;
  $("#settings-modal").classList.remove("hidden");
}

function openHandSettings() {
  const settings = currentHandSettings();
  setText("#settings-title", "灵巧手参数设置");
  setText("#settings-path", settings.wujiConfig || optionsCache?.hand?.config_file || "--");
  $(".settings-note").textContent = "控制使用官方 wujihandros2 ROS2 driver：滑条和开/关按钮会发布 sensor_msgs/JointState 到左右手 joint_commands topic。这里保存的是 FKViewer 页面参数；更换姿态 YAML 后会重新读取左右手 20 维开/关数据。";
  $("#settings-form").innerHTML = handSettingsMarkup(optionsCache?.hand || {}, settings);
  $("#settings-modal").classList.remove("hidden");
}

async function openSettings() {
  if (moduleName === "impedance") {
    await openImpedanceSettings();
    return;
  }
  if (moduleName === "hand") {
    openHandSettings();
    return;
  }
  await openTeleopSettings();
}

window.openSettings = openSettings;

function closeSettings() {
  $("#settings-modal")?.classList.add("hidden");
}

async function saveSettings() {
  if (moduleName === "hand") {
    const settings = currentHandSettings();
    window.localStorage.setItem("fkviewer.hand.settings", JSON.stringify(settings));
    closeSettings();
    await hydrateSelectedHandConfig();
    renderHandConfig(optionsCache.hand);
    toast("灵巧手设置已保存");
    return;
  }
  if (moduleName === "impedance") {
    const settings = currentImpedanceSettings();
    window.localStorage.setItem("fkviewer.impedance.settings", JSON.stringify(settings));
    const profiles = {};
    document.querySelectorAll("#settings-form [data-profile][data-key]").forEach((input) => {
      const profile = input.dataset.profile;
      const key = input.dataset.key;
      if (!profile || !key) return;
      profiles[profile] = profiles[profile] || {};
      profiles[profile][key] = input.value;
    });
    const result = await api("/api/yaml", {
      method: "POST",
      body: JSON.stringify({ target: "impedance_profiles", profiles }),
    });
    toast(`阻抗设置已保存: ${result.changed} 项，重启对应程序后生效`);
    closeSettings();
    await refresh();
    return;
  }
  const profiles = {};
  document.querySelectorAll("#settings-form input[data-profile][data-key]").forEach((input) => {
    const profile = input.dataset.profile;
    const key = input.dataset.key;
    if (!profile || !key) return;
    profiles[profile] = profiles[profile] || {};
    profiles[profile][key] = input.value;
  });
  const result = await api("/api/yaml", {
    method: "POST",
    body: JSON.stringify({ target: "teleop_split", profiles }),
  });
  toast(`设置已保存: ${result.changed} 项，重启对应程序后生效`);
  closeSettings();
  await loadOptions();
}

function describeAction(payload) {
  if (payload.action === "launch") return `启动${processDisplayName(payload.name)}`;
  if (payload.action === "stop") return `停止${processDisplayName(payload.name)}`;
  const opLabels = {
    start_impedance: "启动阻抗控制",
    stop_impedance: "停止阻抗控制",
    clear_error: "清除机械臂错误",
    update_params: "应用阻抗参数",
    enable: "使能灵巧手",
    disable: "关闭灵巧手使能",
    reset_error: "清除灵巧手错误",
    pose: "发送灵巧手姿态",
  };
  const target = payload.arm ? `${payload.arm === "left" ? "左" : "右"}臂` : payload.side ? `${payload.side === "left" ? "左" : "右"}手` : "";
  return `${target}${opLabels[payload.op] || payload.op || "执行操作"}`;
}

async function postAction(payload, button = null, { confirmed = false } = {}) {
  const actionText = describeAction(payload);
  if (statusCache?.live_control && !confirmed) {
    const accepted = await requestConfirmation(
      payload.action === "stop" ? "确认停止程序" : "确认真机操作",
      `${actionText}将立即作用于当前系统。请确认设备周围安全且状态符合预期。`,
      payload.action === "stop" ? "确认停止" : "确认执行",
    );
    if (!accepted) {
      setOperation("idle", "已取消", `${actionText}未执行`);
      return null;
    }
  }
  setButtonBusy(button, true, payload.action === "stop" ? "正在停止..." : "正在执行...");
  setOperation("working", "执行中", `${actionText}，请稍候...`, false);
  try {
    const result = await api("/api/action", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    let message;
    if (result.dry_run) {
      message = `${actionText}仿真完成，未向真机下发操作`;
    } else if (payload.action === "stop") {
      message = result.returncode != null
        ? `${processDisplayName(payload.name)}已经结束，退出码 ${result.returncode}`
        : `${processDisplayName(payload.name)}已收到停止信号，正在等待进程结束`;
    } else {
      message = `${actionText}已完成${result.pid ? `，PID ${result.pid}` : ""}`;
    }
    setOperation(payload.action === "stop" && result.returncode == null && !result.dry_run ? "warning" : "success", result.dry_run ? "仿真完成" : "操作完成", message);
    toast(message, payload.action === "stop" && !result.dry_run ? "warn" : true);
    await refresh();
    return result;
  } catch (error) {
    const message = `${actionText}失败：${error.message}`;
    setOperation("error", "操作失败", message);
    toast(message, false);
    return null;
  } finally {
    setButtonBusy(button, false);
    syncProcessButtons();
  }
}

function applyWorkflowStatus(status) {
  statusCache = status;
  renderStatus();
  renderEvents(status.events || []);
  syncProcessButtons();
}

async function waitForInferenceReady(timeoutMs = 90000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const status = await api("/api/status");
    applyWorkflowStatus(status);
    const ready = inferencePrerequisites(status);
    if (ready.all) return status;
    const waiting = [!ready.left && "左臂 HTTP", !ready.right && "右臂 HTTP", !ready.wuji && "Wuji"]
      .filter(Boolean).join("、");
    setOperation("working", "等待就绪", `推理阻抗已启动，正在等待：${waiting}`, false);
    await new Promise((resolve) => window.setTimeout(resolve, 1500));
  }
  throw new Error("等待双臂 HTTP 与 Wuji 服务就绪超时，请查看推理阻抗日志");
}

async function waitForProcessStopped(name, timeoutMs = 15000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const status = await api("/api/status");
    applyWorkflowStatus(status);
    if (!status.managed_processes?.[name]?.running) return true;
    setOperation("working", "正在停止", `等待${processDisplayName(name)}安全退出...`, false);
    await new Promise((resolve) => window.setTimeout(resolve, 800));
  }
  throw new Error(`${processDisplayName(name)}未在规定时间内退出`);
}

async function startInferenceWorkflow(button) {
  if (inferenceWorkflowBusy) return;
  inferenceWorkflowBusy = true;
  setButtonBusy(button, true, "正在顺序启动...");
  try {
    let confirmed = false;
    if (statusCache?.live_control) {
      confirmed = await requestConfirmation(
        "确认启动推理流程",
        "系统将先启动推理阻抗，等待双臂 HTTP 与 Wuji 就绪，然后运行 client_dual.py。请确认设备周围安全。",
        "确认顺序启动",
      );
      if (!confirmed) {
        setOperation("idle", "已取消", "推理流程未启动");
        return;
      }
    }
    let policyResult = null;
    if (!isProcessRunning("impedance_policy_stack")) {
      policyResult = await postAction({
        action: "launch",
        name: "impedance_policy_stack",
        selections: moduleSelections(),
      }, null, { confirmed });
      if (!policyResult) return;
    }
    if (!policyResult?.dry_run) await waitForInferenceReady();
    if (!isProcessRunning("easydp_client") || policyResult?.dry_run) {
      await postAction({
        action: "launch",
        name: "easydp_client",
        selections: moduleSelections(),
      }, null, { confirmed });
    }
  } catch (error) {
    setOperation("error", "推理启动失败", error.message);
    toast(`推理启动失败：${error.message}`, false);
  } finally {
    inferenceWorkflowBusy = false;
    setButtonBusy(button, false);
    syncInferenceWorkflow();
  }
}

async function stopInferenceWorkflow(button) {
  if (inferenceWorkflowBusy) return;
  const managed = statusCache?.managed_processes || {};
  if (isProcessRunning("easydp_client") && !managed.easydp_client?.running) {
    toast("检测到外部启动的推理客户端，FKViewer 无法安全停止它；请先在原终端停止客户端", false);
    return;
  }
  inferenceWorkflowBusy = true;
  setButtonBusy(button, true, "正在顺序停止...");
  try {
    let confirmed = false;
    if (statusCache?.live_control) {
      confirmed = await requestConfirmation(
        "确认停止推理流程",
        "系统将先停止双臂推理客户端，确认退出后再停止推理阻抗。",
        "确认顺序停止",
      );
      if (!confirmed) {
        setOperation("idle", "已取消", "推理流程保持运行");
        return;
      }
    }
    let clientResult = null;
    if (managed.easydp_client?.running) {
      clientResult = await postAction({ action: "stop", name: "easydp_client", selections: moduleSelections() }, null, { confirmed });
      if (!clientResult) return;
      if (!clientResult.dry_run) await waitForProcessStopped("easydp_client");
    }
    const latestManaged = statusCache?.managed_processes || managed;
    if (latestManaged.impedance_policy_stack?.running) {
      await postAction({ action: "stop", name: "impedance_policy_stack", selections: moduleSelections() }, null, { confirmed });
    }
  } catch (error) {
    setOperation("error", "推理停止失败", error.message);
    toast(`推理停止失败：${error.message}`, false);
  } finally {
    inferenceWorkflowBusy = false;
    setButtonBusy(button, false);
    syncInferenceWorkflow();
  }
}

async function runArmActions(op, body, button) {
  const arms = selectedArms();
  const actionText = `${arms.length === 2 ? "双臂" : arms[0] === "left" ? "左臂" : "右臂"}${describeAction({ action: "arm", arm: arms[0], op }).replace(/^(左|右)臂/, "")}`;
  let confirmed = false;
  if (statusCache?.live_control) {
    confirmed = await requestConfirmation("确认真机操作", `${actionText}将立即下发。请确认设备周围安全且状态符合预期。`, "确认执行");
    if (!confirmed) {
      setOperation("idle", "已取消", `${actionText}未执行`);
      return;
    }
  }
  setButtonBusy(button, true);
  const results = [];
  for (const arm of arms) {
    results.push(await postAction({ action: "arm", arm, op, body, selections: moduleSelections() }, null, { confirmed }));
  }
  setButtonBusy(button, false);
  syncProcessButtons();
  return results;
}

function selectedArms() {
  const arm = currentImpedanceSettings().arm || "both";
  return arm === "both" ? ["left", "right"] : [arm];
}

function impedanceParams() {
  const settings = currentImpedanceSettings();
  return {
    translational_stiffness: Number(settings.transK),
    translational_damping: Number(settings.transD),
    rotational_stiffness: Number(settings.rotK),
    rotational_damping: Number(settings.rotD),
  };
}

async function sendHandPose(side, values, label = "pose", quiet = false, confirmed = false) {
  const sideLabel = side === "left" ? "左手" : "右手";
  if (!quiet && statusCache?.live_control && !confirmed) {
    const accepted = await requestConfirmation("确认灵巧手动作", `即将向${sideLabel}发送“${label}”姿态。请确认手部周围无障碍物。`, "确认发送");
    if (!accepted) {
      setOperation("idle", "已取消", `${sideLabel}姿态未发送`);
      return null;
    }
  }
  if (!quiet) setOperation("working", "发送中", `正在向${sideLabel}发送${label}...`, false);
  try {
    const result = await api("/api/action", {
      method: "POST",
      body: JSON.stringify({
        action: "hand",
        side,
        op: "pose",
        body: { positions: values },
        selections: moduleSelections(),
      }),
    });
    if (!quiet) {
      const message = result.dry_run ? `${sideLabel}${label}姿态仿真完成，未下发真机` : `${sideLabel}${label}姿态已发送`;
      setOperation("success", result.dry_run ? "仿真完成" : "发送完成", message);
      toast(message);
      await refresh();
    }
    return result;
  } catch (error) {
    if (!quiet) {
      const message = `${sideLabel}姿态发送失败：${error.message.slice(0, 220)}`;
      setOperation("error", "发送失败", message);
      toast(message, false);
    }
    throw error;
  }
}

async function sendBothHands(poseName) {
  const labels = { released: "打开双手", closed: "关闭双手" };
  const leftValues = handPose("left", poseName);
  const rightValues = handPose("right", poseName);
  if (!setHandValues("left", leftValues) || !setHandValues("right", rightValues)) return;
  if (statusCache?.live_control) {
    const accepted = await requestConfirmation("确认双手动作", `即将执行“${labels[poseName] || poseName}”。请确认双手周围无障碍物。`, "确认执行");
    if (!accepted) {
      setOperation("idle", "已取消", `${labels[poseName] || poseName}未执行`);
      return;
    }
  }
  setOperation("working", "发送中", `正在执行${labels[poseName] || poseName}...`, false);
  try {
    const results = await Promise.allSettled([
      sendHandPose("left", handValues.left, labels[poseName] || poseName, true),
      sendHandPose("right", handValues.right, labels[poseName] || poseName, true),
    ]);
    const failed = results.find((result) => result.status === "rejected");
    if (failed) throw failed.reason;
    const dryRun = results.some((item) => item.status === "fulfilled" && item.value?.dry_run);
    const message = dryRun ? `${labels[poseName] || poseName}仿真完成，未下发真机` : `${labels[poseName] || poseName}已完成`;
    setOperation("success", dryRun ? "仿真完成" : "操作完成", message);
    toast(message);
    await refresh();
  } catch (error) {
    const message = `双手动作失败：${error.message.slice(0, 220)}`;
    setOperation("error", "操作失败", message);
    toast(message, false);
  }
}

function scheduleHandSend(side) {
  const settings = currentHandSettings();
  const delay = Math.max(20, Number(settings.throttleMs) || 120);
  window.clearTimeout(handSendTimers[side]);
  handSendTimers[side] = window.setTimeout(() => {
    handSendTimers[side] = null;
    sendHandNow(side, true);
  }, delay);
}

async function sendHandNow(side, quiet = false) {
  if (handSending[side]) {
    handPending[side] = true;
    return;
  }
  handSending[side] = true;
  try {
    await sendHandPose(side, handValues[side], "当前姿态", quiet);
  } catch {
    // sendHandPose already reports user-facing errors for non-quiet calls.
  } finally {
    handSending[side] = false;
    if (handPending[side]) {
      handPending[side] = false;
      scheduleHandSend(side);
    }
  }
}

function setHandValues(side, values) {
  if (!Array.isArray(values) || values.length !== HAND_JOINT_COUNT) {
    toast(`${side} 没有可用的 20 维姿态`, false);
    return false;
  }
  handValues[side] = values.map((value) => Number(value) || 0);
  refreshHandSliders(side);
  return true;
}

function handleHandJointInput(target) {
  const wrap = target.closest("[data-hand-joint]");
  if (!wrap) return false;
  const [side, indexRaw] = wrap.dataset.handJoint.split(":");
  const index = Number(indexRaw);
  if (!["left", "right"].includes(side) || !Number.isInteger(index)) return false;
  const value = Number(target.value);
  handValues[side][index] = Number.isFinite(value) ? value : 0;
  const number = wrap.querySelector(".joint-number");
  const range = wrap.querySelector(".joint-range");
  if (number && target !== number) number.value = Number(handValues[side][index]).toFixed(3);
  if (range && target !== range) range.value = String(handValues[side][index]);
  scheduleHandSend(side);
  return true;
}

document.addEventListener("click", async (event) => {
  const button = event.target.closest("button");
  if (!button) return;
  if (button.dataset.logFollow) {
    const key = button.dataset.logFollow;
    setLogFollow(key, !runtimeLogFollow[key], { scroll: !runtimeLogFollow[key] });
    return;
  }
  if (button.matches("[data-confirm-cancel]")) {
    resolveConfirmation(false);
    return;
  }
  if (button.matches("[data-confirm-submit]")) {
    resolveConfirmation(true);
    return;
  }
  if (button.matches("[data-dismiss-operation]")) {
    setOperation("idle", "就绪", "请选择需要执行的操作", false);
    return;
  }
  if (button.matches("[data-refresh]")) {
    setButtonBusy(button, true, "刷新中...");
    await refresh();
    setButtonBusy(button, false);
    return;
  }
  if (button.matches("[data-settings]")) {
    openSettings().catch((error) => toast(`设置加载失败: ${error.message}`, false));
    return;
  }
  if (button.matches("[data-close-settings]")) {
    closeSettings();
    return;
  }
  if (button.matches("[data-save-settings]")) {
    setButtonBusy(button, true, "保存中...");
    try {
      await saveSettings();
      setOperation("success", "设置已保存", "新设置将在相关程序下次启动时生效");
    } catch (error) {
      setOperation("error", "保存失败", error.message);
      toast(`设置保存失败：${error.message}`, false);
    } finally {
      setButtonBusy(button, false);
    }
    return;
  }
  if (button.matches("[data-inference-start-all]")) {
    await startInferenceWorkflow(button);
    return;
  }
  if (button.matches("[data-inference-stop-all]")) {
    await stopInferenceWorkflow(button);
    return;
  }
  if (button.matches("[data-inference-reset]")) {
    if (isProcessRunning("easydp_client")) {
      const message = "请先停止双臂推理客户端，再恢复机械臂位置";
      setOperation("warning", "无法复位", message);
      toast(message, "warn");
      return;
    }
    if (!inferencePrerequisites().all) {
      const message = "请先启动推理阻抗，并等待左臂 HTTP、右臂 HTTP 与 Wuji 服务全部就绪";
      setOperation("warning", "前置服务未就绪", message);
      toast(message, "warn");
      return;
    }
    await postAction({ action: "launch", name: "easydp_reset", selections: {} }, button);
    return;
  }
  if (button.dataset.launch) {
    if (["easydp_client", "unified_inference_client"].includes(button.dataset.launch) && !inferencePrerequisites().all) {
      const message = "无法启动双臂推理：左臂 HTTP、右臂 HTTP 与 Wuji 服务尚未全部就绪";
      setOperation("error", "前置检查失败", message);
      toast(message, false);
      return;
    }
    await postAction({ action: "launch", name: button.dataset.launch, selections: moduleSelections() }, button);
    return;
  }
  if (button.dataset.stop) {
    if (moduleName === "inference" && button.dataset.stop === "impedance_policy_stack" && isProcessRunning("easydp_client")) {
      const message = "请先停止双臂推理客户端，再停止推理阻抗";
      setOperation("warning", "停止顺序错误", message);
      toast(message, "warn");
      return;
    }
    if (
      moduleName === "unified"
      && button.dataset.stop === "unified_control_stack"
      && (isProcessRunning("unified_inference_client") || isProcessRunning("unified_quest_layer"))
    ) {
      const message = "请先停止推理客户端和 Quest 接管与录制层，再停止统一控制栈";
      setOperation("warning", "停止顺序错误", message);
      toast(message, "warn");
      return;
    }
    await postAction({ action: "stop", name: button.dataset.stop, selections: moduleSelections() }, button);
    return;
  }
  if (button.dataset.impOp) {
    await runArmActions(button.dataset.impOp, undefined, button);
    return;
  }
  if (button.matches("[data-imp-params]")) {
    await runArmActions("update_params", impedanceParams(), button);
    return;
  }
  if (button.dataset.handPose) {
    const side = button.dataset.handSide || "right";
    const values = handPose(side, button.dataset.handPose);
    if (setHandValues(side, values)) {
      setButtonBusy(button, true);
      await sendHandPose(side, handValues[side], button.dataset.handPose).catch(() => {});
      setButtonBusy(button, false);
    }
    return;
  }
  if (button.dataset.handBothPose) {
    setButtonBusy(button, true);
    await sendBothHands(button.dataset.handBothPose).catch(() => {});
    setButtonBusy(button, false);
    return;
  }
  if (button.matches("[data-hand-copy]")) {
    const side = button.dataset.handSide || "right";
    copyText(poseArrayText(handValues[side]))
      .then(() => toast(`${side} 20 维关节数据已复制`))
      .catch((error) => toast(`复制失败: ${error.message}`, false));
    return;
  }
  if (button.matches("[data-hand-import]")) {
    const side = button.dataset.handSide || "right";
    const textarea = document.getElementById(`${side}-pose-import`);
    try {
      const values = parsePoseArrayText(textarea?.value || "");
      if (setHandValues(side, values)) sendHandPose(side, handValues[side], "导入姿态").catch(() => {});
    } catch (error) {
      toast(`导入失败: ${error.message}`, false);
    }
    return;
  }
  if (button.dataset.handOp) {
    await postAction({
      action: "hand",
      side: button.dataset.handSide || "right",
      op: button.dataset.handOp,
      selections: moduleSelections(),
    }, button);
  }
});

document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  if (!$("#confirm-modal")?.classList.contains("hidden")) {
    resolveConfirmation(false);
    return;
  }
  if (!$("#settings-modal")?.classList.contains("hidden")) closeSettings();
});

document.querySelectorAll(".modal").forEach((modal) => {
  modal.addEventListener("click", (event) => {
    if (event.target !== modal) return;
    if (modal.id === "confirm-modal") resolveConfirmation(false);
    if (modal.id === "settings-modal") closeSettings();
  });
});

document.addEventListener("input", (event) => {
  const target = event.target;
  if (!(target instanceof HTMLInputElement)) return;
  handleHandJointInput(target);
});

init().catch((error) => {
  toast(`初始化失败: ${error.message}`, false);
  setOperation("error", "初始化失败", error.message);
  const actionArea = $("#action-area");
  if (actionArea) {
    actionArea.innerHTML = `
      <div class="proc-row">
        <div class="proc-name">初始化失败</div>
        <div class="proc-line">${escapeHtml(error.message)}</div>
      </div>
    `;
  }
});
