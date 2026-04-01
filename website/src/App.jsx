import { useEffect, useMemo, useRef, useState } from "react";
import { io } from "socket.io-client";
import * as XLSX from "xlsx";
import LiveMissionMap from "./LiveMissionMap";

const relayLegend = [
  { label: "Active Relay", className: "dot-active" },
  { label: "Failed Relay", className: "dot-failed" },
  { label: "Idle Relay", className: "dot-idle" },
  { label: "UAV Path", className: "dot-path" }
];

const bloodGroups = ["O+", "O-", "A+", "A-", "B+", "B-", "AB+", "AB-"];
const organOptions = ["Heart", "Lung", "Liver", "Kidney"];
const validRoles = ["user", "src", "dest", "admin"];

const hlaMarkers = [
  { id: "m1", label: "HLA-A A02" },
  { id: "m2", label: "HLA-A A24" },
  { id: "m3", label: "HLA-A A26" },
  { id: "m4", label: "HLA-B B07" },
  { id: "m5", label: "HLA-B B15" },
  { id: "m6", label: "HLA-B B40" },
  { id: "m7", label: "HLA-DR DR04" },
  { id: "m8", label: "HLA-DR DR11" },
  { id: "m9", label: "HLA-DR DR13" }
];
const totalMarkers = 6;

const hospitalOptions = [
  { id: "H1", name: "MIOT Hospitals", latitude: 13.0212213, longitude: 80.1839951 },
  { id: "H2", name: "Government Royapettah Hospital", latitude: 13.0554981, longitude: 80.2648512 },
  { id: "H3", name: "Rajiv Gandhi Govt General Hospital", latitude: 13.0815311, longitude: 80.2776431 },
  { id: "H4", name: "Sri Ramachandra Medical Centre", latitude: 13.0392523, longitude: 80.1434904 },
  { id: "H5", name: "Frontline Hospital", latitude: 13.1022933, longitude: 80.1903163 },
  { id: "H6", name: "Velammal Medical College", latitude: 13.0792928, longitude: 80.1144087 },
  { id: "H7", name: "Dr Kamakshi Memorial", latitude: 12.9518467, longitude: 80.2094059 },
  { id: "H8", name: "Gleneagles Health City", latitude: 12.898058, longitude: 80.2062485 },
  { id: "H9", name: "Chettinad Hospital", latitude: 12.7968561, longitude: 80.2184882 },
  { id: "H10", name: "Sree Balaji Medical College", latitude: 12.9554166, longitude: 80.1378104 },
  { id: "H11", name: "SRM Medical College", latitude: 12.8210435, longitude: 80.048141 },
  { id: "H12", name: "Hindu Mission Hospital", latitude: 12.9238515, longitude: 80.1141072 },
  { id: "H13", name: "Saveetha Medical College", latitude: 13.0264032, longitude: 80.0139753 },
  { id: "H14", name: "Tagore Medical College", latitude: 12.8603845, longitude: 80.1361455 },
  { id: "H15", name: "Karpagam Hospital", latitude: 13.2006399, longitude: 79.8908348 }
];

const formatHospitalLabel = (hospital) => `${hospital.id} - ${hospital.name}`;
const normalizeText = (value) => String(value ?? "").trim().toLowerCase();
const normalizeCode = (value) => String(value ?? "").trim().toUpperCase();
const findHospitalById = (id) => hospitalOptions.find((hospital) => hospital.id === id) ?? null;
const missionStorageKey = "mission_state_v2";

const getAgeGroup = (age) => {
  if (age <= 17) {
    return "0-17";
  }
  if (age <= 35) {
    return "18-35";
  }
  if (age <= 55) {
    return "36-55";
  }
  return "56+";
};

const emptyMission = {
  sourceHospital: null,
  destinationHospital: null,
  sourceUpdates: [],
  destinationUpdates: [],
  requestMeta: null,
  hla: {
    srcSelected: [],
    destSelected: [],
    srcSubmitted: false,
    destSubmitted: false,
    totalMatches: 0,
    compatibilityScore: null,
    compatible: false
  },
  adminMission: {
    citSec: "",
    batteryPercent: 100,
    uavSpeed: 25,
    submitted: false
  },
  execution: {
    state: "idle",
    message: "",
    finalLat: null,
    finalLon: null
  }
};

const normalizeMission = (raw) => {
  const base = {
    ...emptyMission,
    ...(raw || {}),
    hla: {
      ...emptyMission.hla,
      ...((raw && raw.hla) || {})
    },
    adminMission: {
      ...emptyMission.adminMission,
      ...((raw && raw.adminMission) || {})
    },
    execution: {
      ...emptyMission.execution,
      ...((raw && raw.execution) || {})
    },
    sourceUpdates: Array.isArray(raw?.sourceUpdates) ? raw.sourceUpdates : [],
    destinationUpdates: Array.isArray(raw?.destinationUpdates) ? raw.destinationUpdates : []
  };

  if (base.requestMeta) {
    base.requestMeta = {
      ...base.requestMeta,
      donorPatient: {
        patientName: base.requestMeta?.donorPatient?.patientName || "Unknown Donor",
        age: base.requestMeta?.donorPatient?.age ?? "-",
        bloodGroup: base.requestMeta?.donorPatient?.bloodGroup || "-"
      },
      recipientPatient: {
        patientName: base.requestMeta?.recipientPatient?.patientName || "Unknown Recipient",
        age: base.requestMeta?.recipientPatient?.age ?? "-",
        bloodGroup: base.requestMeta?.recipientPatient?.bloodGroup || "-"
      }
    };
  }

  return base;
};

export default function App() {
  const bridgeUrl = useMemo(() => import.meta.env.VITE_BRIDGE_URL || "http://localhost:8000", []);
  const [loggedInRole, setLoggedInRole] = useState("");
  const [loginForm, setLoginForm] = useState({ username: "", password: "" });
  const [loginError, setLoginError] = useState("");

  const [availabilityRows, setAvailabilityRows] = useState([]);
  const [availabilityError, setAvailabilityError] = useState("");
  const [isAvailabilityLoading, setIsAvailabilityLoading] = useState(true);

  const [userForm, setUserForm] = useState({
    patientName: "",
    age: "",
    hospitalId: hospitalOptions[0].id,
    bloodGroup: "O+",
    organRequired: "Heart"
  });
  const [userMessage, setUserMessage] = useState({ type: "", text: "" });
  const [pendingRequest, setPendingRequest] = useState(null);
  const [liveFrame, setLiveFrame] = useState(null);
  const [flightTrail, setFlightTrail] = useState([]);
  const [missionRuntime, setMissionRuntime] = useState({ running: false, state: "idle", error: "" });
  const lastLoggedStepRef = useRef(0);
  const liveFrameRef = useRef(null);

  const [roleMessage, setRoleMessage] = useState("");
  const [adminMessage, setAdminMessage] = useState("");
  const readMissionFromStorage = () => {
    try {
      const raw = localStorage.getItem(missionStorageKey);
      return raw ? normalizeMission(JSON.parse(raw)) : null;
    } catch {
      return null;
    }
  };

  const [mission, setMission] = useState(() => {
    const stored = readMissionFromStorage();
    return stored ?? normalizeMission(emptyMission);
  });

  useEffect(() => {
    localStorage.setItem(missionStorageKey, JSON.stringify(mission));
  }, [mission]);

  useEffect(() => {
    const socket = io(bridgeUrl, { transports: ["websocket"] });

    socket.on("connect_error", () => {
      setMissionRuntime((prev) => ({ ...prev, error: "Bridge connection failed." }));
    });

    socket.on("drl-frame", (frame) => {
      setLiveFrame(frame);
      liveFrameRef.current = frame;
      if (frame?.lat != null && frame?.lon != null) {
        setFlightTrail((prev) => {
          const next = [...prev, [Number(frame.lat), Number(frame.lon)]];
          if (next.length > 2000) {
            return next.slice(next.length - 2000);
          }
          return next;
        });
      }
      if (frame?.step != null) {
        const step = Number(frame.step) || 0;
        if (step > lastLoggedStepRef.current) {
          lastLoggedStepRef.current = step;
          const msg = `Step ${step} | Relay ${frame?.relay ?? "-"} | Mode ${frame?.mode ?? "-"} | SINR ${
            frame?.sinr != null ? Number(frame.sinr).toFixed(1) : "-"
          } dB`;
          setMission((prev) => ({
            ...prev,
            sourceUpdates: [msg, ...prev.sourceUpdates].slice(0, 300),
            destinationUpdates: [msg, ...prev.destinationUpdates].slice(0, 300)
          }));
        }
      }
      setMissionRuntime((prev) => ({ ...prev, running: true, state: "running", error: "" }));
    });

    socket.on("mission-status", (status) => {
      setMissionRuntime((prev) => ({
        ...prev,
        running: Boolean(status?.running),
        state: String(status?.state || prev.state),
        error: String(status?.error || "")
      }));
    });

    socket.on("mission-end", (payload) => {
      setMissionRuntime((prev) => ({ ...prev, running: false, state: "ended" }));
      const reason = String(payload?.finalReason || "Mission Ended");
      const lat = payload?.finalLat ?? liveFrameRef.current?.lat ?? null;
      const lon = payload?.finalLon ?? liveFrameRef.current?.lon ?? null;
      const hasCoord = lat != null && lon != null;
      const locationText = hasCoord ? `(${Number(lat).toFixed(6)}, ${Number(lon).toFixed(6)})` : "unknown location";

      const isAbandoned = reason.toLowerCase().includes("abandoned");
      const isSuccess = reason.toLowerCase().includes("simulation complete");
      const message = isAbandoned
        ? `Mission abandoned. UAV stopped at ${locationText}.`
        : isSuccess
          ? `Mission succeeded. Drone landed at ${locationText}.`
          : `Drone landed at ${locationText}. Reason: ${reason}`;
      const timestamp = new Date().toLocaleTimeString();

      setMission((prev) => ({
        ...prev,
        sourceUpdates: [`[${timestamp}] ${message}`, ...prev.sourceUpdates],
        destinationUpdates: [`[${timestamp}] ${message}`, ...prev.destinationUpdates],
        execution: {
          state: isAbandoned ? "abandoned" : isSuccess ? "succeeded" : "landed",
          message,
          finalLat: lat ?? null,
          finalLon: lon ?? null
        }
      }));
    });

    return () => socket.disconnect();
  }, [bridgeUrl]);

  useEffect(() => {
    const onStorage = (event) => {
      if (event.key === missionStorageKey && event.newValue) {
        try {
          setMission(normalizeMission(JSON.parse(event.newValue)));
        } catch {
          // ignore invalid storage payload
        }
      }
    };
    window.addEventListener("storage", onStorage);
    return () => window.removeEventListener("storage", onStorage);
  }, []);

  useEffect(() => {
    const loadAvailabilityFile = async () => {
      try {
        setIsAvailabilityLoading(true);
        const response = await fetch("/data/organ_available.xlsx");
        if (!response.ok) {
          throw new Error("Excel file not found");
        }
        const fileBuffer = await response.arrayBuffer();
        const workbook = XLSX.read(fileBuffer, { type: "array" });
        const firstSheet = workbook.Sheets[workbook.SheetNames[0]];
        const rows = XLSX.utils.sheet_to_json(firstSheet, { defval: "" });
        setAvailabilityRows(rows);
        setAvailabilityError("");
      } catch {
        setAvailabilityRows([]);
        setAvailabilityError("Could not load organ availability data.");
      } finally {
        setIsAvailabilityLoading(false);
      }
    };
    loadAvailabilityFile();
  }, []);

  const handleLoginFieldChange = (event) => {
    const { name, value } = event.target;
    setLoginForm((prev) => ({ ...prev, [name]: value }));
  };

  const handleLogin = (event) => {
    event.preventDefault();
    const username = normalizeText(loginForm.username);
    const password = String(loginForm.password ?? "");
    const isValid = validRoles.includes(username) && password === "123";

    if (!isValid) {
      setLoginError("Invalid credentials. Use user/src/dest/admin with password 123.");
      return;
    }

    setLoginError("");
    setLoggedInRole(username);
    setRoleMessage("");
    setAdminMessage("");
    setLoginForm({ username: "", password: "" });
  };

  const handleLogout = () => {
    setLoggedInRole("");
  };

  const handleUserFieldChange = (event) => {
    const { name, value } = event.target;
    setUserForm((prev) => ({ ...prev, [name]: value }));
  };

  const pickBestByRemainingTime = (rows) =>
    rows.reduce((best, current) => {
      const currentRemaining = Number(current.Remaining_Time) || 0;
      const bestRemaining = Number(best.Remaining_Time) || 0;
      return currentRemaining > bestRemaining ? current : best;
    });

  const handleUserSubmit = (event) => {
    event.preventDefault();
    const parsedAge = Number(userForm.age);
    if (!Number.isFinite(parsedAge) || parsedAge < 0 || parsedAge > 120) {
      setPendingRequest(null);
      setUserMessage({ type: "error", text: "Enter a valid age between 0 and 120." });
      return;
    }

    if (isAvailabilityLoading) {
      setPendingRequest(null);
      setUserMessage({ type: "info", text: "Organ availability data is still loading. Try again in a moment." });
      return;
    }

    if (availabilityRows.length === 0) {
      setPendingRequest(null);
      setUserMessage({
        type: "error",
        text: availabilityError || "Organ availability data is missing. Add Excel to website/public/data."
      });
      return;
    }

    const requestedHospital = findHospitalById(userForm.hospitalId);
    const requestAgeGroup = getAgeGroup(parsedAge);
    const sameCoreMatch = (row) =>
      normalizeCode(row.Blood_Group) === normalizeCode(userForm.bloodGroup) &&
      normalizeText(row.Organ) === normalizeText(userForm.organRequired) &&
      getAgeGroup(Number(row.Age)) === requestAgeGroup;

    const sameHospitalMatches = availabilityRows.filter(
      (row) => normalizeCode(row.Hospital_ID) === normalizeCode(userForm.hospitalId) && sameCoreMatch(row)
    );

    let chosenRow = null;
    if (sameHospitalMatches.length > 0) {
      chosenRow = pickBestByRemainingTime(sameHospitalMatches);
    } else {
      const otherHospitalMatches = availabilityRows.filter(
        (row) => normalizeCode(row.Hospital_ID) !== normalizeCode(userForm.hospitalId) && sameCoreMatch(row)
      );
      if (otherHospitalMatches.length > 0) {
        chosenRow = pickBestByRemainingTime(otherHospitalMatches);
      }
    }

    if (!chosenRow) {
      setPendingRequest(null);
      setUserMessage({ type: "error", text: "No matching organ availability found in any hospital." });
      return;
    }

    const availableHospital = findHospitalById(normalizeCode(chosenRow.Hospital_ID)) ?? {
      id: normalizeCode(chosenRow.Hospital_ID),
      name: String(chosenRow.Hospital_Name || "Matched Hospital"),
      latitude: null,
      longitude: null
    };

    setPendingRequest({
      patientName: userForm.patientName.trim(),
      age: parsedAge,
      bloodGroup: userForm.bloodGroup,
      organRequired: userForm.organRequired,
      requestHospital: requestedHospital,
      availableHospital,
      donorPatient: {
        patientName: String(chosenRow.Patient_Name || "Matched Donor"),
        age: Number(chosenRow.Age) || "-",
        bloodGroup: String(chosenRow.Blood_Group || "-")
      }
    });

    setUserMessage({
      type: "success",
      text: `Organ is available at ${formatHospitalLabel(availableHospital)}.`
    });
  };

  const handleRequestOrgan = () => {
    if (!pendingRequest) {
      return;
    }

    const timestamp = new Date().toLocaleTimeString();
    const source = pendingRequest.availableHospital;
    const destination = pendingRequest.requestHospital;

    setMission({
      sourceHospital: source,
      destinationHospital: destination,
      sourceUpdates: [
        `[${timestamp}] Organ allocated at ${formatHospitalLabel(source)}.`,
        `[${timestamp}] HLA review pending from SRC and DEST.`
      ],
      destinationUpdates: [
        `[${timestamp}] Delivery request received for ${formatHospitalLabel(destination)}.`,
        `[${timestamp}] Waiting for compatibility validation.`
      ],
      requestMeta: {
        organRequired: pendingRequest.organRequired,
        createdAt: timestamp,
        donorPatient: pendingRequest.donorPatient,
        recipientPatient: {
          patientName: pendingRequest.patientName || "Recipient",
          age: pendingRequest.age,
          bloodGroup: pendingRequest.bloodGroup
        }
      },
      hla: {
        srcSelected: [],
        destSelected: [],
        srcSubmitted: false,
        destSubmitted: false,
        totalMatches: 0,
        compatibilityScore: null,
        compatible: false
      },
      adminMission: {
        citSec: "",
        batteryPercent: 100,
        uavSpeed: 25,
        submitted: false
      },
      execution: {
        state: "idle",
        message: "",
        finalLat: null,
        finalLon: null
      }
    });

    setUserMessage({ type: "success", text: "Request sent to SRC/DEST. Admin dashboard updated." });
    setPendingRequest(null);
    setUserForm({
      patientName: "",
      age: "",
      hospitalId: hospitalOptions[0].id,
      bloodGroup: "O+",
      organRequired: "Heart"
    });
  };

  const toggleHlaMarker = (role, markerId) => {
    const latestMission = readMissionFromStorage() ?? mission;
    setMission(() => {
      const prev = latestMission;
      const selected = role === "src" ? prev.hla.srcSelected : prev.hla.destSelected;
      const nextSelected = selected.includes(markerId)
        ? selected.filter((id) => id !== markerId)
        : [...selected, markerId];
      return {
        ...prev,
        hla: {
          ...prev.hla,
          srcSelected: role === "src" ? nextSelected : prev.hla.srcSelected,
          destSelected: role === "dest" ? nextSelected : prev.hla.destSelected
        }
      };
    });
  };

  const maybeFinalizeCompatibility = (draftMission) => {
    const bothSubmitted = draftMission.hla.srcSubmitted && draftMission.hla.destSubmitted;
    if (!bothSubmitted) {
      return draftMission;
    }

    const srcSet = new Set(draftMission.hla.srcSelected);
    const destSet = new Set(draftMission.hla.destSelected);
    const totalMatches = hlaMarkers.filter((marker) => srcSet.has(marker.id) && destSet.has(marker.id)).length;
    const score = totalMatches / totalMarkers;
    const compatible = score > 0.5;
    const timestamp = new Date().toLocaleTimeString();

    return {
      ...draftMission,
      sourceUpdates: [
        `[${timestamp}] Compatibility calculated: ${totalMatches}/${totalMarkers} (${score.toFixed(2)}).`,
        ...draftMission.sourceUpdates
      ],
      destinationUpdates: [
        `[${timestamp}] ${compatible ? "Compatible. Mission Active." : "Incompatible. Mission blocked."}`,
        ...draftMission.destinationUpdates
      ],
      hla: {
        ...draftMission.hla,
        totalMatches,
        compatibilityScore: score,
        compatible
      }
    };
  };

  const submitHlaResults = (role) => {
    const latestMission = readMissionFromStorage() ?? mission;
    if (!latestMission.requestMeta) {
      setRoleMessage("No request available yet.");
      return;
    }

    setMission(() => {
      const prev = latestMission;
      const updated = {
        ...prev,
        hla: {
          ...prev.hla,
          srcSubmitted: role === "src" ? true : prev.hla.srcSubmitted,
          destSubmitted: role === "dest" ? true : prev.hla.destSubmitted
        }
      };
      return maybeFinalizeCompatibility(updated);
    });

    setRoleMessage(`${role.toUpperCase()} results submitted.`);
  };

  const handleAdminMissionFieldChange = (event) => {
    const { name, value } = event.target;
    setMission((prev) => ({
      ...prev,
      adminMission: {
        ...prev.adminMission,
        [name]: value
      }
    }));
  };

  const handleAdminMissionSubmit = async (event) => {
    event.preventDefault();
    const cit = Number(mission.adminMission.citSec);
    if (!Number.isFinite(cit) || cit <= 0) {
      setAdminMessage("Enter a valid CIT value in seconds.");
      return;
    }

    const payload = {
      src: mission.sourceHospital?.id,
      dst: mission.destinationHospital?.id,
      organ: mission.requestMeta?.organRequired || "Unknown",
      CIT_total: cit,
      batteryPercent: Number(mission.adminMission.batteryPercent) || 100,
      uavSpeedMS: Number(mission.adminMission.uavSpeed) || 25
    };

    try {
      setLiveFrame(null);
      setFlightTrail([]);
      lastLoggedStepRef.current = 0;
      const response = await fetch(`${bridgeUrl}/mission/start`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });
      const body = await response.json().catch(() => ({}));

      if (!response.ok) {
        setAdminMessage(body?.error || "Failed to start mission.");
        return;
      }

      const timestamp = new Date().toLocaleTimeString();
      setMission((prev) => ({
        ...prev,
        sourceUpdates: [`[${timestamp}] Mission parameters configured by admin.`, ...prev.sourceUpdates],
        destinationUpdates: [`[${timestamp}] UAV dispatch started from control center.`, ...prev.destinationUpdates],
        adminMission: {
          ...prev.adminMission,
          submitted: true
        },
        execution: {
          state: "running",
          message: "",
          finalLat: null,
          finalLon: null
        }
      }));
      setMissionRuntime((prev) => ({ ...prev, running: true, state: "starting", error: "" }));
      setAdminMessage("Start mission signal sent. Waiting for live frames.");
    } catch {
      setAdminMessage("Bridge server not reachable. Start backend first.");
    }
  };

  if (!loggedInRole) {
    return (
      <main className="auth-screen">
        <section className="auth-card">
          <p className="eyebrow">Secure Access</p>
          <h1>Organ Relay Portal Login</h1>
          <form className="auth-form" onSubmit={handleLogin}>
            <label htmlFor="username">Username</label>
            <input
              id="username"
              name="username"
              type="text"
              value={loginForm.username}
              onChange={handleLoginFieldChange}
              placeholder="user | src | dest | admin"
              required
            />
            <label htmlFor="password">Password</label>
            <input
              id="password"
              name="password"
              type="password"
              value={loginForm.password}
              onChange={handleLoginFieldChange}
              placeholder="123"
              required
            />
            <button type="submit" className="submit-btn">
              Login
            </button>
          </form>
          <p className="auth-help">Valid users: user, src, dest, admin. Password for all: 123</p>
          {loginError && <p className="auth-error">{loginError}</p>}
        </section>
      </main>
    );
  }

  if (loggedInRole === "user") {
    return (
      <main className="role-screen">
        <section className="panel role-panel">
          <header className="panel-head">
            <div>
              <p className="eyebrow">User Portal</p>
              <h1>Organ Requirement</h1>
            </div>
            <button type="button" className="logout-btn" onClick={handleLogout}>
              Logout
            </button>
          </header>

          <form className="organ-form" onSubmit={handleUserSubmit}>
            <label htmlFor="patientName">Patient Name</label>
            <input
              id="patientName"
              name="patientName"
              type="text"
              value={userForm.patientName}
              onChange={handleUserFieldChange}
              placeholder="Enter patient name"
              required
            />

            <label htmlFor="age">Age</label>
            <input
              id="age"
              name="age"
              type="number"
              min="0"
              max="120"
              value={userForm.age}
              onChange={handleUserFieldChange}
              placeholder="Enter age"
              required
            />

            <label htmlFor="hospitalId">Hospital</label>
            <select id="hospitalId" name="hospitalId" value={userForm.hospitalId} onChange={handleUserFieldChange}>
              {hospitalOptions.map((hospital) => (
                <option key={hospital.id} value={hospital.id}>
                  {formatHospitalLabel(hospital)}
                </option>
              ))}
            </select>

            <label htmlFor="bloodGroup">Blood Group</label>
            <select id="bloodGroup" name="bloodGroup" value={userForm.bloodGroup} onChange={handleUserFieldChange}>
              {bloodGroups.map((group) => (
                <option key={group} value={group}>
                  {group}
                </option>
              ))}
            </select>

            <label htmlFor="organRequired">Organ Required</label>
            <select
              id="organRequired"
              name="organRequired"
              value={userForm.organRequired}
              onChange={handleUserFieldChange}
            >
              {organOptions.map((organ) => (
                <option key={organ} value={organ}>
                  {organ}
                </option>
              ))}
            </select>

            <button type="submit" className="submit-btn">
              Submit
            </button>
          </form>

          {userMessage.text && <article className={`match-status status-${userMessage.type}`}>{userMessage.text}</article>}

          {pendingRequest && (
            <button type="button" className="submit-btn request-btn" onClick={handleRequestOrgan}>
              Request Organ
            </button>
          )}
        </section>
      </main>
    );
  }

  if (loggedInRole === "src" || loggedInRole === "dest") {
    const roleKey = loggedInRole;
    const roleSubmitted = roleKey === "src" ? mission.hla.srcSubmitted : mission.hla.destSubmitted;
    const selectedMarkers = roleKey === "src" ? mission.hla.srcSelected : mission.hla.destSelected;

    return (
      <main className="role-screen">
        <section className="panel role-panel">
          <header className="panel-head">
            <div>
              <p className="eyebrow">{roleKey.toUpperCase()} Portal</p>
              <h1>HLA Validation</h1>
            </div>
            <button type="button" className="logout-btn" onClick={handleLogout}>
              Logout
            </button>
          </header>

          {!mission.requestMeta ? (
            <p className="auth-help">No request received yet.</p>
          ) : (
            <>
              <div className="patient-grid">
                <article className="patient-card">
                  <h3>Matching Patient (Donor)</h3>
                  <p>Name: {mission.requestMeta.donorPatient.patientName}</p>
                  <p>Age: {mission.requestMeta.donorPatient.age}</p>
                  <p>Blood Group: {mission.requestMeta.donorPatient.bloodGroup}</p>
                </article>
                <article className="patient-card">
                  <h3>Recipient Patient</h3>
                  <p>Name: {mission.requestMeta.recipientPatient.patientName}</p>
                  <p>Age: {mission.requestMeta.recipientPatient.age}</p>
                  <p>Blood Group: {mission.requestMeta.recipientPatient.bloodGroup}</p>
                </article>
              </div>

              <div className="hla-box">
                <h3>HLA Typing (Multiple Selection)</h3>
                <p className="auth-help">Select markers you confirm as matched.</p>
                <div className="hla-list">
                  {hlaMarkers.map((marker) => (
                    <label key={marker.id} className="hla-row">
                      <input
                        type="checkbox"
                        checked={selectedMarkers.includes(marker.id)}
                        onChange={() => toggleHlaMarker(roleKey, marker.id)}
                      />
                      <span>{marker.label}</span>
                    </label>
                  ))}
                </div>
                <button type="button" className="submit-btn" onClick={() => submitHlaResults(roleKey)}>
                  Submit Results
                </button>
                {roleSubmitted && <p className="auth-help">Results submitted for {roleKey.toUpperCase()}.</p>}
              </div>

              {roleMessage && <article className="match-status status-info">{roleMessage}</article>}
              {mission.hla.srcSubmitted && mission.hla.destSubmitted && (
                <article className={`match-status ${mission.hla.compatible ? "status-success" : "status-error"}`}>
                  Total matches = {mission.hla.totalMatches} out of {totalMarkers} | Compatibility score ={" "}
                  {mission.hla.compatibilityScore?.toFixed(2)}
                </article>
              )}
              {mission.execution.message && <article className="match-status status-info">{mission.execution.message}</article>}
            </>
          )}
        </section>
      </main>
    );
  }

  const hlaReady = mission.hla.srcSubmitted && mission.hla.destSubmitted;

  return (
    <main className="screen">
      <div className="hud-glow hud-glow-left" />
      <div className="hud-glow hud-glow-right" />

      <section className="panel control-center">
        <header className="panel-head">
          <div>
            <p className="eyebrow">Top</p>
            <h1>Control Center</h1>
          </div>
          <div className="head-actions">
            <div className="status-pill">
              <span className="pulse" />
              Mission Control
            </div>
            <button type="button" className="logout-btn" onClick={handleLogout}>
              Logout
            </button>
          </div>
        </header>

        <div className="mission-control-wrap">
          <article className="mission-state">
            {mission.execution.state === "abandoned"
              ? "Mission Abandoned"
              : mission.execution.state === "succeeded"
                ? "Mission Succeeded"
              : mission.execution.state === "landed"
                ? "Mission Ended | Drone Landed"
                : !mission.requestMeta
              ? "No Missions Active."
              : !hlaReady
                ? "Mission Pending | Waiting for SRC and DEST HLA submissions."
                : mission.hla.compatible
                  ? `Compatible (${mission.hla.totalMatches}/${totalMarkers}) | ${
                      missionRuntime.running ? "Mission Running" : "Mission Ready"
                    }`
                  : `Incompatible (${mission.hla.totalMatches}/${totalMarkers}) | Mission Blocked`}
          </article>

          {mission.execution.message && <article className="match-status status-info">{mission.execution.message}</article>}
          {(mission.execution.finalLat != null && mission.execution.finalLon != null) && (
            <article className="match-status status-info">
              Final UAV Location: ({Number(mission.execution.finalLat).toFixed(6)}, {Number(mission.execution.finalLon).toFixed(6)})
            </article>
          )}

          {hlaReady && (
            <article className={`match-status ${mission.hla.compatible ? "status-success" : "status-error"}`}>
              Total matches = {mission.hla.totalMatches} out of {totalMarkers}
              <br />
              HLA Match Score = {mission.hla.totalMatches} / {totalMarkers} = {mission.hla.compatibilityScore?.toFixed(2)}
            </article>
          )}

          {mission.hla.compatible && (
            <form className="admin-mission-form" onSubmit={handleAdminMissionSubmit}>
              <h2>Mission Control Inputs</h2>
              <label>Src Hospital</label>
              <input value={mission.sourceHospital ? formatHospitalLabel(mission.sourceHospital) : ""} readOnly />

              <label>Dest Hospital</label>
              <input value={mission.destinationHospital ? formatHospitalLabel(mission.destinationHospital) : ""} readOnly />

              <label>Organ</label>
              <input value={mission.requestMeta?.organRequired || ""} readOnly />

              <label htmlFor="citSec">CIT (s)</label>
              <input
                id="citSec"
                name="citSec"
                type="number"
                min="1"
                value={mission.adminMission.citSec}
                onChange={handleAdminMissionFieldChange}
                required
              />

              <label htmlFor="batteryPercent">Battery Percentage</label>
              <input
                id="batteryPercent"
                name="batteryPercent"
                type="number"
                min="1"
                max="100"
                value={mission.adminMission.batteryPercent}
                onChange={handleAdminMissionFieldChange}
              />

              <label htmlFor="uavSpeed">UAV Speed (m/s)</label>
              <input
                id="uavSpeed"
                name="uavSpeed"
                type="number"
                min="1"
                value={mission.adminMission.uavSpeed}
                onChange={handleAdminMissionFieldChange}
              />
              <button type="submit" className="submit-btn">
                Start Mission
              </button>
              {adminMessage && <p className="auth-help">{adminMessage}</p>}
            </form>
          )}

          <div className="control-grid">
            <article className="map-frame">
              <LiveMissionMap
                liveFrame={liveFrame}
                trail={flightTrail}
                sourceHospital={mission.sourceHospital}
                destinationHospital={mission.destinationHospital}
              />
              <div className="map-overlay map-overlay-live">
                <span>{missionRuntime.running ? "Live MATLAB Stream" : "Mission Map"}</span>
                <small>
                  {liveFrame
                    ? `t=${Number(liveFrame.timeElapsed).toFixed(1)}s | lat=${Number(liveFrame.lat).toFixed(5)}, lon=${Number(
                        liveFrame.lon
                      ).toFixed(5)}`
                    : "Start mission to receive real-time frames"}
                </small>
              </div>
            </article>

            <aside className="telemetry">
              <h2>Live Telemetry</h2>
              <ul>
                <li>
                  <span>Relay</span>
                  <strong>{liveFrame?.relay ?? "-"}</strong>
                </li>
                <li>
                  <span>Mode</span>
                  <strong>{liveFrame?.mode ?? "-"}</strong>
                </li>
                <li>
                  <span>SINR</span>
                  <strong>{liveFrame?.sinr != null ? `${Number(liveFrame.sinr).toFixed(1)} dB` : "-"}</strong>
                </li>
                <li>
                  <span>Battery</span>
                  <strong>
                    {liveFrame?.batteryPercent != null
                      ? `${Number(liveFrame.batteryPercent).toFixed(1)}%`
                      : `${mission.adminMission.batteryPercent}%`}
                  </strong>
                </li>
                <li>
                  <span>CIT Remaining</span>
                  <strong>{liveFrame?.citRemaining != null ? `${Number(liveFrame.citRemaining).toFixed(1)} s` : "-"}</strong>
                </li>
              </ul>
              <div className="legend">
                {relayLegend.map((item) => (
                  <p key={item.label}>
                    <i className={item.className} />
                    {item.label}
                  </p>
                ))}
              </div>
            </aside>
          </div>
        </div>
      </section>

      <section className="panel source-panel">
        <header className="panel-head">
          <div>
            <p className="eyebrow">Bottom Left</p>
            <h2>Source Hospital</h2>
          </div>
        </header>
        <div className="card-grid">
          {[
            { label: "Hospital ID", value: mission.sourceHospital ? mission.sourceHospital.id : "Not selected" },
            {
              label: "Hospital Name",
              value: mission.sourceHospital ? mission.sourceHospital.name : "Waiting for request"
            },
            {
              label: "Latitude",
              value: mission.sourceHospital?.latitude != null ? mission.sourceHospital.latitude.toFixed(7) : "-"
            },
            {
              label: "Longitude",
              value: mission.sourceHospital?.longitude != null ? mission.sourceHospital.longitude.toFixed(7) : "-"
            }
          ].map((item) => (
            <article className="stat-card" key={item.label}>
              <p>{item.label}</p>
              <strong>{item.value}</strong>
            </article>
          ))}
        </div>
        <div className="updates-block">
          <h3>Updates</h3>
          {mission.sourceUpdates.length === 0 ? (
            <p className="updates-empty">No updates yet.</p>
          ) : (
            <ul className="updates-list">
              {mission.sourceUpdates.map((entry, index) => (
                <li key={`${entry}-${index}`}>{entry}</li>
              ))}
            </ul>
          )}
        </div>
      </section>

      <section className="panel destination-panel">
        <header className="panel-head">
          <div>
            <p className="eyebrow">Bottom Right</p>
            <h2>Destination Hospital</h2>
          </div>
        </header>
        <div className="card-grid">
          {[
            {
              label: "Hospital ID",
              value: mission.destinationHospital ? mission.destinationHospital.id : "No destination yet"
            },
            {
              label: "Hospital Name",
              value: mission.destinationHospital ? mission.destinationHospital.name : "Waiting for request"
            },
            {
              label: "Latitude",
              value:
                mission.destinationHospital?.latitude != null ? mission.destinationHospital.latitude.toFixed(7) : "-"
            },
            {
              label: "Longitude",
              value:
                mission.destinationHospital?.longitude != null ? mission.destinationHospital.longitude.toFixed(7) : "-"
            }
          ].map((item) => (
            <article className="stat-card" key={item.label}>
              <p>{item.label}</p>
              <strong>{item.value}</strong>
            </article>
          ))}
        </div>
        <div className="updates-block">
          <h3>Updates</h3>
          {mission.destinationUpdates.length === 0 ? (
            <p className="updates-empty">No updates yet.</p>
          ) : (
            <ul className="updates-list">
              {mission.destinationUpdates.map((entry, index) => (
                <li key={`${entry}-${index}`}>{entry}</li>
              ))}
            </ul>
          )}
        </div>
      </section>
    </main>
  );
}
