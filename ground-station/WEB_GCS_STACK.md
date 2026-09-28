# Ground Station Web UI — Recommended Technology Stack & Architecture

This document specifies the recommended frontend architecture, libraries, and integration patterns for building the WebSerial + Gamepad Ground Control Station (GCS) to accompany the flight firmware.

---

## 1. Core Framework & Build

| Component | Choice | Reason |
| :--- | :--- | :--- |
| **Framework** | **Svelte 5** | Fine-grained reactivity (Runes) without Virtual DOM overhead. High-frequency telemetry (30–100 Hz) updates only specific DOM nodes/canvas transforms without causing frame drops. |
| **Bundler** | **Vite (SPA)** | Fast HMR, direct support for WebAssembly (`wasm-pack`), client-side only (hardware Web APIs require no SSR). |
| **Language** | **TypeScript** | Strict typing for telemetry packets, command schemas, and serial frames. |
| **Styling & UI** | **Tailwind CSS + Bits UI / Lucide-Svelte** | Lightweight HUD overlays, calibration panels, and clean telemetry badges. |

---

## 2. Library Recommendations by Domain

### A. Gamepad & RC Control

#### Evaluated: `@lizardbyte/gamepad-helper`
- **What it is:** A utility by the Sunshine/LizardByte team designed to detect controller hardware (Xbox, PlayStation, Switch, generic) and render visual controller artwork matching button/stick positions.
- **Verdict:** **Recommended for the UI & Controller Settings view, but supplement with custom RC math for flight control.**
  - **Pros:** 
    - Identifies controller type and maps vendor-specific button layouts automatically.
    - Built-in visualizer with clean controller art to show connected controller status.
    - Detects browser/OS gamepad compatibility quirks (`getCompatibilityIssues()`).
  - **Limitations for RC Flight:**
    - It is a general gaming library; it does **not** handle RC flight mechanics: Mode 2 / Mode 1 channel assignments, deadband filtering, exponential curves (`expo`), or channel scaling (e.g. mapping sticks to 1000–2000 µs PWM or normalized $[-1.0, 1.0]$ floats).
- **Recommended Setup:**
  - Use `@lizardbyte/gamepad-helper` in a **Controller Setup / Diagnostics Modal** so the pilot can see their controller type, test buttons, and verify connections.
  - Implement a dedicated lightweight **`RcChannelMapper`** in TypeScript to process raw `gamepad.axes` during flight:
    - Deadzone thresholding (e.g. ignore values $< \pm 0.04$)
    - Expo curve: $f(x) = (1 - e)x + e \cdot x^3$
    - Channel mapping: Throttle, Yaw, Pitch, Roll, Arm Switch, Flight Mode Switch.

---

### B. Telemetry Ingestion & WebSerial

| Library / Tool | Role | Why |
| :--- | :--- | :--- |
| **Native WebSerial API** | Hardware serial connection | Standard browser API (`navigator.serial`) available in Chromium browsers. Operates at up to 921,600 baud with zero external dependencies. |
| **Web Streams API** | Stream buffering & framing | Pipe `port.readable` through a custom `TransformStream` to reassemble framed binary packets from serial chunk fragments. |
| **WebAssembly via `wasm-pack`** *(Recommended)* | Shared Packet Deserialization | Compile a tiny Rust WASM package from `flight-core` using `postcard`. Guarantees the web UI and ESP32 firmware share identical packet schemas with zero protocol drift. |

---

### C. Artificial Horizon & HUD

| Component | Recommended Tool | Approach |
| :--- | :--- | :--- |
| **Horizon Ball (Pitch & Roll)** | **HTML5 Canvas 2D** | Draw sky/ground division and pitch ladder using standard 2D context transforms (`ctx.translate`, `ctx.rotate`). Eliminates third-party engine overhead and easily runs at 60–144 FPS. |
| **HUD Overlays (Ladders, Tapes)** | **Declarative SVG (Svelte)** | Airspeed tapes, altitude ladders, reticles, and heading scales drawn as SVG elements. Crisp at any DPI screen, styled via Tailwind CSS, updated reactively by Svelte props. |
| **Optional 3D Attitude Model** | **Threlte (`@threlte/core`)** | Only needed if you want a full 3D drone mesh mirroring vehicle attitude in real time. |

*Note: Avoid **p5.js** for this use case. p5.js adds ~1 MB of unnecessary runtime and requires instance-mode boilerplate to work inside Svelte components.*

---

### D. GPS Map

| Library | Role | Details |
| :--- | :--- | :--- |
| **MapLibre GL JS** (`maplibre-gl`) | Hardware-accelerated map | Open-source fork of Mapbox GL. Supports vector tiles, hardware-accelerated smooth rotation (`map.setBearing(heading)` for "course-up" drone navigation), and smooth marker interpolation. |
| **Leaflet** *(Alternative)* | Simple raster tile map | Lighter, but lacks native smooth vector map rotation. Better suited only if relying strictly on pre-downloaded offline raster tile packs (PNG/MBTiles). |

---

## 3. Recommended Project Structure

Place the web application adjacent to the Rust crates within this repository:

```text
flight/
├── Cargo.toml                    # Rust monorepo root
├── flight-core/                  # Shared Rust telemetry schemas (Command, Attitude, Postcard)
├── flight-esp32s3/               # Drone flight controller firmware
├── flight-sitl/                  # SITL harness
├── ground-station/               # ESP32-S3 hardware bridge firmware (ESP-NOW <-> USB)
│   └── WEB_GCS_STACK.md          # This documentation
│
└── ground-station-ui/            # Web application (Svelte 5 + Vite)
    ├── package.json
    ├── vite.config.ts
    ├── src/
    │   ├── lib/
    │   │   ├── comms/            # WebSerial connection & packet parser
    │   │   ├── controller/       # Gamepad polling, @lizardbyte visualizer, RC curves
    │   │   ├── components/
    │   │   │   ├── Horizon.svelte    # Canvas 2D artificial horizon
    │   │   │   ├── HudOverlay.svelte # SVG airspeed/altitude tapes & reticle
    │   │   │   ├── GpsMap.svelte     # MapLibre GL GPS tracking view
    │   │   │   └── GamepadModal.svelte # Controller config & visualizer
    │   │   └── state/
    │   │       └── telemetry.svelte.ts # Svelte 5 rune state for live telemetry
    │   ├── App.svelte
    │   └── main.ts
```

---

## 4. Quick-Start Commands

To initialize the frontend workspace inside `flight/`:

```bash
# From workspace root
npm create vite@latest ground-station-ui -- --template svelte-ts
cd ground-station-ui

# Core dependencies
npm install maplibre-gl @lizardbyte/gamepad-helper lucide-svelte
npm install -D @types/maplibre-gl tailwindcss @tailwindcss/vite
```
