const express = require("express");
const cors = require("cors");
const http = require("http");
const { Server } = require("socket.io");
const { spawn } = require("child_process");
const path = require("path");

const app = express();
app.use(cors());
app.use(express.json({ limit: "1mb" }));

const server = http.createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

const BRIDGE_PORT = Number(process.env.BRIDGE_PORT || 8000);
const MATLAB_CMD = process.env.MATLAB_CMD || "matlab";
const PROJECT_ROOT = path.resolve(__dirname, "..", "..");
const MATLAB_SCRIPT_DIR = path.join(PROJECT_ROOT, "Cooperative Communication and DRL NAV");
const DEFAULT_AGENT_FILE = process.env.MATLAB_AGENT_FILE || "";
const WEB_BASE_FOR_MATLAB = process.env.WEB_BASE_FOR_MATLAB || `http://127.0.0.1:${BRIDGE_PORT}`;

let activeProcess = null;
let missionState = { running: false, state: "idle", error: "" };
let lastFrame = null;
let stopRequested = false;
let frameCount = 0;

const safeMatlabString = (value) => String(value ?? "").replace(/'/g, "''");
const parseHospitalId = (value) => {
  const text = String(value ?? "").trim().toUpperCase();
  const digits = text.startsWith("H") ? text.slice(1) : text;
  const parsed = Number(digits);
  return Number.isFinite(parsed) ? parsed : NaN;
};

const broadcastStatus = () => {
  io.emit("mission-status", missionState);
};

const buildMatlabBatchCommand = (payload) => {
  const src = parseHospitalId(payload.src);
  const dst = parseHospitalId(payload.dst);
  const cit = Number(payload.CIT_total);
  const battery = Number(payload.batteryPercent);
  const speed = Number(payload.uavSpeedMS);
  const organ = safeMatlabString(payload.organ || "Unknown");
  const agentFile = payload.agentFile || DEFAULT_AGENT_FILE;

  if (!Number.isFinite(src) || !Number.isFinite(dst)) {
    throw new Error("Invalid src/dst. Expected Hx format like H9 or numeric values.");
  }
  if (!Number.isFinite(cit) || cit <= 0) {
    throw new Error("Invalid CIT_total.");
  }
  if (!Number.isFinite(battery) || battery <= 0) {
    throw new Error("Invalid batteryPercent.");
  }
  if (!Number.isFinite(speed) || speed <= 0) {
    throw new Error("Invalid uavSpeedMS.");
  }
  if (!agentFile) {
    throw new Error("Agent file not configured. Set MATLAB_AGENT_FILE or pass agentFile in /mission/start.");
  }

  const matlabDir = safeMatlabString(MATLAB_SCRIPT_DIR.replace(/\\/g, "/"));
  const matlabAgent = safeMatlabString(path.resolve(agentFile).replace(/\\/g, "/"));
  const webBase = safeMatlabString(WEB_BASE_FOR_MATLAB);

  return [
    `addpath('${matlabDir}')`,
    `m=struct('src',${src},'dst',${dst},'organ','${organ}','CIT_total',${cit},'batteryPercent',${battery},'uavSpeedMS',${speed},'webEnable',true,'webBase','${webBase}','enableVisual',false)`,
    `react('${matlabAgent}',m)`
  ].join(";");
};

app.get("/health", (_req, res) => {
  res.json({ ok: true, ...missionState });
});

app.post("/mission/start", (req, res) => {
  if (activeProcess) {
    return res.status(409).json({ error: "Mission already running." });
  }

  let batchCommand;
  try {
    batchCommand = buildMatlabBatchCommand(req.body || {});
  } catch (error) {
    return res.status(400).json({ error: error.message });
  }

  missionState = { running: true, state: "starting", error: "" };
  lastFrame = null;
  stopRequested = false;
  frameCount = 0;
  broadcastStatus();

  activeProcess = spawn(MATLAB_CMD, ["-batch", batchCommand], {
    cwd: PROJECT_ROOT,
    shell: false,
    stdio: ["ignore", "pipe", "pipe"]
  });

  activeProcess.stdout.on("data", (chunk) => {
    process.stdout.write(`[MATLAB] ${chunk}`);
  });
  activeProcess.stderr.on("data", (chunk) => {
    process.stderr.write(`[MATLAB-ERR] ${chunk}`);
  });

  activeProcess.on("exit", (code) => {
    if (stopRequested) {
      missionState = { running: false, state: "abandoned", error: "" };
    } else {
      missionState = {
        running: false,
        state: code === 0 ? "ended" : "failed",
        error: code === 0 ? "" : `MATLAB exited with code ${code}`
      };
    }
    activeProcess = null;
    stopRequested = false;
    broadcastStatus();
  });

  return res.json({ ok: true, status: "starting" });
});

app.post("/frame", (req, res) => {
  frameCount += 1;
  const payload = { ...(req.body || {}), step: req.body?.step ?? frameCount };
  missionState = { ...missionState, running: true, state: "running" };
  lastFrame = payload;
  io.emit("drl-frame", payload);
  broadcastStatus();
  res.json({ ok: true });
});

app.post("/mission/stop", (_req, res) => {
  if (!activeProcess) {
    return res.status(409).json({ error: "No active mission to stop." });
  }

  stopRequested = true;
  const finalLat = lastFrame?.lat ?? null;
  const finalLon = lastFrame?.lon ?? null;

  const payload = {
    event: "mission_end",
    finalReason: "Mission Abandoned",
    step: lastFrame?.step ?? null,
    finalLat,
    finalLon
  };

  if (process.platform === "win32") {
    spawn("taskkill", ["/PID", String(activeProcess.pid), "/T", "/F"], { shell: true });
  } else {
    activeProcess.kill("SIGTERM");
  }
  activeProcess = null;

  missionState = { running: false, state: "abandoned", error: "" };
  io.emit("mission-end", payload);
  broadcastStatus();
  return res.json({ ok: true, status: "abandoned", finalLat, finalLon });
});

app.post("/mission/end", (req, res) => {
  if (stopRequested) {
    return res.json({ ok: true, ignored: true });
  }
  missionState = { running: false, state: "ended", error: "" };
  io.emit("mission-end", { ...(req.body || {}), step: req.body?.step ?? frameCount });
  broadcastStatus();
  res.json({ ok: true });
});

server.listen(BRIDGE_PORT, () => {
  console.log(`Bridge server running on http://localhost:${BRIDGE_PORT}`);
  console.log(`MATLAB script dir: ${MATLAB_SCRIPT_DIR}`);
  console.log(`MATLAB agent default: ${DEFAULT_AGENT_FILE || "(not set)"}`);
});
