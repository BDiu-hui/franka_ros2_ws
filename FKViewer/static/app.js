const $ = (selector) => document.querySelector(selector);
const MODULE_TITLES = {
  impedance: "阻抗控制",
  hand: "灵巧手控制",
  inference: "推理控制",
  teleop: "摇操控制",
  unified: "统一控制",
};
const MODULE_SUBTITLES = {
  impedance: "Franka 控制栈状态与参数操作",
  hand: "Wuji 灵巧手驱动、姿态与桥接服务",
  inference: "固定双臂推理、位置恢复与进程日志",
  teleop: "Quest 双臂摇操、分进程启动与日志",
  unified: "Y 键切换 EasyDP 推理与 Quest 摇操控制权",
};
let activeModule = new URLSearchParams(window.location.search).get("module") || "impedance";
let refreshPending = false;
let connectionFailed = false;
if (!MODULE_TITLES[activeModule]) activeModule = "impedance";

function setText(selector, text) {
  const el = $(selector);
  if (el) el.textContent = text;
}

function notify(message, type = "ok", timeout = 4200) {
  const stack = $("#toast-stack");
  if (!stack) return;
  const item = document.createElement("div");
  item.className = `toast ${type}`;
  item.textContent = message;
  item.addEventListener("click", () => item.remove());
  stack.append(item);
  while (stack.children.length > 4) stack.firstElementChild?.remove();
  window.setTimeout(() => item.remove(), timeout);
}

async function api(path) {
  const response = await fetch(path);
  const contentType = response.headers.get("content-type") || "";
  if (!contentType.includes("application/json")) throw new Error(`服务器返回了无效响应 (HTTP ${response.status})`);
  const data = await response.json();
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function setFrameLoading(loading, text = "正在加载模块...") {
  const target = $("#frame-loading");
  if (!target) return;
  target.classList.toggle("is-hidden", !loading);
  const label = target.querySelector("span:last-child");
  if (label) label.textContent = text;
}

function loadModule(module) {
  if (!MODULE_TITLES[module]) return;
  activeModule = module;
  const hasSettings = ["teleop", "impedance", "hand"].includes(module);
  const frame = $("#module-frame");
  setFrameLoading(true, `正在加载${MODULE_TITLES[module]}...`);
  setText("#app-status-message", `正在切换到${MODULE_TITLES[module]}`);
  if (frame) frame.src = `/window?module=${encodeURIComponent(module)}&embedded=1`;
  document.querySelectorAll("[data-load-module]").forEach((button) => {
    const active = button.dataset.loadModule === module;
    button.classList.toggle("active", active);
    button.setAttribute("aria-current", active ? "page" : "false");
  });
  const url = new URL(window.location.href);
  url.searchParams.set("module", module);
  window.history.replaceState({}, "", url);
  document.title = `FKViewer - ${MODULE_TITLES[module]}`;
  setText("#active-module-title", MODULE_TITLES[module]);
  setText("#active-module-subtitle", MODULE_SUBTITLES[module]);
  setText("#active-module-status", MODULE_TITLES[module]);
  for (const selector of ["#frame-settings-button", "#frame-settings-inline"]) {
    $(selector)?.classList.toggle("hidden", !hasSettings);
  }
  const settingsLabels = { impedance: "阻抗设置", hand: "灵巧手设置", teleop: "摇操设置" };
  setText("#frame-settings-inline", settingsLabels[module] || "设置");
}

function setSidebarCollapsed(hidden) {
  document.body.classList.toggle("sidebar-collapsed", hidden);
  setText("[data-toggle-sidebar]", hidden ? "显示侧栏" : "隐藏侧栏");
  window.localStorage.setItem("fkviewer.sidebar.collapsed", hidden ? "1" : "0");
}

function toggleSidebar() {
  setSidebarCollapsed(!document.body.classList.contains("sidebar-collapsed"));
}

function initSidebarResize() {
  const shell = $(".embedded-shell");
  const handle = $("#sidebar-resizer");
  if (!shell || !handle) return;
  const stored = Number(window.localStorage.getItem("fkviewer.sidebar.width"));
  if (Number.isFinite(stored) && stored >= 180 && stored <= 420) {
    shell.style.setProperty("--sidebar-width", `${stored}px`);
  }
  const resize = (event) => {
    const rect = shell.getBoundingClientRect();
    const width = Math.min(420, Math.max(180, event.clientX - rect.left));
    shell.style.setProperty("--sidebar-width", `${width}px`);
    window.localStorage.setItem("fkviewer.sidebar.width", String(Math.round(width)));
  };
  handle.addEventListener("pointerdown", (event) => {
    if (window.innerWidth <= 980) return;
    handle.setPointerCapture(event.pointerId);
    document.body.classList.add("is-resizing");
    resize(event);
  });
  handle.addEventListener("pointermove", (event) => {
    if (handle.hasPointerCapture(event.pointerId)) resize(event);
  });
  const finish = (event) => {
    if (handle.hasPointerCapture(event.pointerId)) handle.releasePointerCapture(event.pointerId);
    document.body.classList.remove("is-resizing");
  };
  handle.addEventListener("pointerup", finish);
  handle.addEventListener("pointercancel", finish);
  handle.addEventListener("keydown", (event) => {
    if (!["ArrowLeft", "ArrowRight"].includes(event.key)) return;
    event.preventDefault();
    const current = parseInt(getComputedStyle(shell).getPropertyValue("--sidebar-width"), 10) || 224;
    const width = Math.min(420, Math.max(180, current + (event.key === "ArrowRight" ? 16 : -16)));
    shell.style.setProperty("--sidebar-width", `${width}px`);
    window.localStorage.setItem("fkviewer.sidebar.width", String(width));
  });
}

function openFrameSettings() {
  const child = $("#module-frame")?.contentWindow;
  if (!["teleop", "impedance", "hand"].includes(activeModule) || !child?.openSettings) {
    notify("当前模块尚未加载完成，请稍后重试", "warn");
    return;
  }
  child.openSettings().catch((error) => notify(`设置加载失败：${error.message}`, "err"));
}

async function refresh({ manual = false } = {}) {
  if (refreshPending) return;
  refreshPending = true;
  try {
    const status = await api("/api/status");
    const live = $("#live-chip");
    live.className = status.live_control ? "chip err" : "chip warn";
    live.innerHTML = `<span class="dot"></span>${status.live_control ? "真机模式" : "仿真模式"}`;
    const refreshChip = $("#refresh-chip");
    refreshChip.className = "chip ok";
    refreshChip.innerHTML = `<span class="dot"></span>${status.now.split(" ").pop()}`;
    const connection = $("#connection-status");
    connection.className = "statusbar-item ok";
    connection.innerHTML = '<span class="dot"></span>服务器已连接';
    setText("#app-status-message", `${status.live_control ? "真机控制已启用" : "当前为仿真模式"}，状态自动刷新中`);
    if (connectionFailed) notify("已恢复与 FKViewer 服务的连接", "ok");
    connectionFailed = false;
    if (manual) notify("状态已刷新", "ok", 2200);
  } catch (error) {
    const connection = $("#connection-status");
    if (connection) {
      connection.className = "statusbar-item err";
      connection.innerHTML = '<span class="dot"></span>服务器连接失败';
    }
    const refreshChip = $("#refresh-chip");
    if (refreshChip) {
      refreshChip.className = "chip err";
      refreshChip.innerHTML = '<span class="dot"></span>连接失败';
    }
    setText("#app-status-message", error.message);
    if (!connectionFailed || manual) notify(`无法刷新状态：${error.message}`, "err", 6500);
    connectionFailed = true;
  } finally {
    refreshPending = false;
  }
}

document.addEventListener("click", (event) => {
  const button = event.target.closest("button");
  if (!button) return;
  if (button.dataset.loadModule) loadModule(button.dataset.loadModule);
  if (button.matches("[data-toggle-sidebar]")) toggleSidebar();
  if (button.matches("[data-frame-settings]")) openFrameSettings();
  if (button.matches("[data-refresh]")) refresh({ manual: true });
});

$("#module-frame")?.addEventListener("load", () => {
  setFrameLoading(false);
  setText("#app-status-message", `${MODULE_TITLES[activeModule]}已加载`);
});

setSidebarCollapsed(window.localStorage.getItem("fkviewer.sidebar.collapsed") === "1");
initSidebarResize();
loadModule(activeModule);
refresh();
window.setInterval(() => refresh(), 2500);
