<script lang="ts">
    import { onMount } from "svelte";
    import Horizon from "./lib/Horizon.svelte";
    import GpsMap from "./lib/GpsMap.svelte";
    import ControllerSettingsModal from "./lib/ControllerSettingsModal.svelte";
    import initWasm, { decode_packet, encode_packet } from "./lib/wasm/web_decoder";
    import type { Message } from "./lib/types/Message";
    import {
        stickToRate,
        loadRateConfig,
        saveRateConfig,
        type RateConfig
    } from "./lib/controller";

    let gp = $state<Gamepad | null>(null);
    let port = $state<SerialPort | null>(null);

    // Rate curve & controller configuration (loaded from localStorage)
    let rateConfig = $state<RateConfig>(loadRateConfig());
    let isControllerModalOpen = $state(false);

    // Persist controller settings on change
    $effect(() => {
        saveRateConfig(rateConfig);
    });

    // Throttle state (0 to 100%)
    let throttle = $state(0);

    // Left stick inputs:
    // Axis 0 = X: Left (<0) is Roll Left, Right (>0) is Roll Right
    // Axis 1 = Y: Invert Y default: Up (<0) is Nose Down, Down (>0) is Nose Up
    let rawRollStick = $derived(gp ? (gp.axes[0] ?? 0) * (rateConfig.invertRoll ? -1 : 1) : 0);
    let rawPitchStick = $derived(gp ? (gp.axes[1] ?? 0) * (rateConfig.invertPitch ? 1 : -1) : 0);

    // Connection & Handshake Status
    let isConnected = $state(false);
    let isVerified = $state(false);
    let droneLinked = $state(false);
    let droneRssi = $state<number | null>(null);
    let firmwareVersion = $state<number | null>(null);
    let connectionError = $state<string | null>(null);
    let heartbeatTimeout: number | null = null;

    let pollHandle: number | null = null;
    let yaw = $state(192);
    let coords = $state<[number, number]>([1.3521, 103.8198]);
    let speed = $state(45);
    let altitude = $state(120);

    // Telemetry state
    let telemetryRoll = $state(0);
    let telemetryPitch = $state(0);

    let roll = $derived.by(() => {
        if (gp && gp.axes[0] !== undefined) {
            const rate = stickToRate(rawRollStick, rateConfig);
            return (rate / rateConfig.maxRate) * 45;
        }
        return telemetryRoll;
    });

    let pitch = $derived.by(() => {
        if (gp && gp.axes[1] !== undefined) {
            const rate = stickToRate(rawPitchStick, rateConfig);
            return (rate / rateConfig.maxRate) * 30;
        }
        return telemetryPitch;
    });

    onMount(() => {
        initWasm().catch(console.error);
    });

    function addGamepad(e: GamepadEvent) {
        gp = e.gamepad;
        pollGamepad();
    }

    function removeGamepad() {
        gp = null;
        if (pollHandle !== null) {
            cancelAnimationFrame(pollHandle);
        }
    }

    function pollGamepad() {
        if (gp === null) return;

        const currentGp = navigator.getGamepads()[gp.index];
        if (currentGp) {
            gp = currentGp;

            // Left stick drives roll & pitch with curves applied
            const rollRate = stickToRate(rawRollStick, rateConfig);
            const pitchRate = stickToRate(rawPitchStick, rateConfig);

            // Throttle button controls:
            // Button 1 = Circle (PS4) -> Throttle UP
            // Button 0 = Cross / X (PS4) -> Throttle DOWN
            const circlePressed = currentGp.buttons[1]?.pressed ?? false;
            const xPressed = currentGp.buttons[0]?.pressed ?? false;

            let throttleChanged = false;
            if (circlePressed && !xPressed) {
                throttle = Math.min(100, throttle + 0.6);
                throttleChanged = true;
            } else if (xPressed && !circlePressed) {
                throttle = Math.max(0, throttle - 0.6);
                throttleChanged = true;
            }

            // Stream RollRate command over serial to the drone
            if (port && port.writable) {
                sendMessage({ RollRate: [rollRate, pitchRate, 0] }).catch(() => {});
                if (throttleChanged) {
                    sendMessage({ Throttle: Math.round(throttle) }).catch(() => {});
                }
            }

            // Right stick (axes[2], axes[3]) does nothing
        }

        pollHandle = requestAnimationFrame(pollGamepad);
    }

    function resetHeartbeatWatchdog() {
        if (heartbeatTimeout !== null) {
            clearTimeout(heartbeatTimeout);
        }
        heartbeatTimeout = window.setTimeout(() => {
            if (isConnected) {
                isVerified = false;
                connectionError = "Bridge heartbeat lost (no signal from ESP32).";
            }
        }, 2500);
    }

    async function sendMessage(msg: Message) {
        if (!port || !port.writable) return;
        const writer = port.writable.getWriter();
        try {
            const bytes = encode_packet(msg);
            await writer.write(bytes);
        } finally {
            writer.releaseLock();
        }
    }

    async function connectSerial() {
        connectionError = null;
        try {
            // 1. Request port filtered strictly to Espressif USB vendor ID (0x303a)
            port = await navigator.serial.requestPort({
                filters: [{ usbVendorId: 0x303a }]
            });

            await port.open({ baudRate: 115200 });
            isConnected = true;
            isVerified = false;

            // 2. Start reading frames (listens for 1 Hz heartbeat from ESP32)
            readSerialLoop();
            resetHeartbeatWatchdog();
        } catch (err: any) {
            console.error("Serial connect error:", err);
            if (err.name !== "NotFoundError") {
                connectionError = err.message || "Failed to open serial port.";
            }
            isConnected = false;
        }
    }

    async function disconnectSerial() {
        if (heartbeatTimeout !== null) {
            clearTimeout(heartbeatTimeout);
            heartbeatTimeout = null;
        }
        isConnected = false;
        isVerified = false;
        droneLinked = false;
        droneRssi = null;
        if (port) {
            try {
                await port.close();
            } catch (e) {
                console.error("Error closing port:", e);
            }
            port = null;
        }
    }

    async function readSerialLoop() {
        if (!port || !port.readable) return;
        const reader = port.readable.getReader();
        let buffer: number[] = [];

        try {
            while (isConnected) {
                const { value, done } = await reader.read();
                if (done) break;

                // Accumulate bytes until 0x00 (COBS delimiter)
                for (const byte of value) {
                    if (byte === 0x00) {
                        if (buffer.length > 0) {
                            try {
                                const msg = decode_packet(new Uint8Array(buffer));
                                handleMessage(msg);
                            } catch (e) {
                                console.warn("Failed to decode packet:", e);
                            }
                            buffer = [];
                        }
                    } else {
                        buffer.push(byte);
                    }
                }
            }
        } catch (err) {
            console.error("Serial read loop error:", err);
        } finally {
            reader.releaseLock();
        }
    }

    function handleMessage(msg: Message) {
        if (typeof msg === "object") {
            if ("Pong" in msg) {
                isVerified = true;
                connectionError = null;
                firmwareVersion = msg.Pong.version;
                droneLinked = msg.Pong.drone_linked;
                droneRssi = msg.Pong.rssi;
                resetHeartbeatWatchdog();
            } else if ("Telemetry" in msg) {
                altitude = msg.Telemetry.altitude;
                telemetryRoll = msg.Telemetry.attitude[0];
                telemetryPitch = msg.Telemetry.attitude[1];
                yaw = msg.Telemetry.attitude[2];
                coords = [msg.Telemetry.coords[0], msg.Telemetry.coords[1]];
            }
        }
    }
</script>

<svelte:window
    ongamepadconnected={addGamepad}
    ongamepaddisconnected={removeGamepad}
/>

<div class="flex flex-col h-screen w-full bg-slate-950 p-2 gap-2 select-none">
    <!-- Main instruments grid -->
    <div class="grid h-1/2 w-full grid-cols-2 gap-2">
        <div class="min-h-0 min-w-0 relative">
            <Horizon {pitch} {roll} {yaw} {speed} {altitude} />
            <!-- Floating Throttle HUD Badge -->
            <div class="absolute top-2 left-2 z-10 px-2.5 py-1 rounded bg-black/75 border border-slate-700 font-mono text-xs backdrop-blur flex items-center gap-2 pointer-events-none">
                <span class="text-slate-400 font-medium">THR</span>
                <div class="w-16 h-2 bg-slate-900 rounded-full overflow-hidden border border-slate-700">
                    <div
                        class="h-full bg-linear-to-r from-cyan-400 to-emerald-400"
                        style="width: {throttle}%"
                    ></div>
                </div>
                <span class="text-cyan-400 font-bold">{Math.round(throttle)}%</span>
            </div>
        </div>

        <div class="min-h-0 min-w-0">
            <GpsMap {coords} {yaw} />
        </div>
    </div>

    <!-- Ground Station Control & Telemetry Bar -->
    <footer class="flex items-center justify-between px-4 py-2 bg-slate-900 border border-slate-800 rounded-md font-mono text-xs text-slate-300">
        <!-- Connection actions & USB Status -->
        <div class="flex items-center gap-3">
            {#if !isConnected}
                <button
                    onclick={connectSerial}
                    class="px-3 py-1.5 rounded bg-blue-600 hover:bg-blue-500 active:bg-blue-700 text-white font-semibold transition"
                >
                    Connect Serial (ESP32)
                </button>
            {:else}
                <button
                    onclick={disconnectSerial}
                    class="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 active:bg-slate-600 text-slate-300 border border-slate-700 transition"
                >
                    Disconnect
                </button>
            {/if}

            <!-- Hardware Bridge Status -->
            <div class="flex items-center gap-1.5">
                <span class="h-2 w-2 rounded-full {isConnected ? (isVerified ? 'bg-emerald-400' : 'bg-amber-400 animate-pulse') : 'bg-slate-600'}"></span>
                <span>
                    {#if !isConnected}
                        USB Disconnected
                    {:else if isVerified}
                        Bridge v{firmwareVersion} (ESP32)
                    {:else}
                        Verifying Bridge...
                    {/if}
                </span>
            </div>

            <!-- Drone Radio Link Status -->
            {#if isConnected && isVerified}
                <div class="flex items-center gap-1.5 border-l border-slate-800 pl-3">
                    <span class="h-2 w-2 rounded-full {droneLinked ? 'bg-emerald-400' : 'bg-rose-500 animate-pulse'}"></span>
                    <span>
                        {#if droneLinked}
                            Drone Linked {droneRssi !== null ? `(${droneRssi} dBm)` : ''}
                        {:else}
                            Drone Link: Searching...
                        {/if}
                    </span>
                </div>
            {/if}

            <!-- Throttle Level Indicator -->
            <div class="flex items-center gap-2 border-l border-slate-800 pl-3">
                <span class="text-slate-400 font-semibold">THR</span>
                <div class="w-20 h-2.5 bg-slate-950 rounded-full overflow-hidden border border-slate-700">
                    <div
                        class="h-full bg-linear-to-r from-cyan-400 to-emerald-400 transition-all duration-75"
                        style="width: {throttle}%"
                    ></div>
                </div>
                <span class="text-cyan-400 font-bold w-10 text-right">{Math.round(throttle)}%</span>
                <span class="text-[10px] text-slate-500 hidden xl:inline">
                    {gp?.buttons[1]?.pressed ? '▲ [○]' : (gp?.buttons[0]?.pressed ? '▼ [✕]' : '[○] / [✕]')}
                </span>
            </div>
        </div>

        <!-- Center / Right Action Controls -->
        <div class="flex items-center gap-3">
            <button
                type="button"
                onclick={() => (isControllerModalOpen = true)}
                class="px-2.5 py-1.5 rounded bg-slate-800 hover:bg-slate-700 active:bg-slate-600 text-cyan-400 border border-slate-700 flex items-center gap-1.5 transition font-semibold"
                title="Configure controller curves and axis inversion"
            >
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                    <rect x="2" y="6" width="20" height="12" rx="4" />
                    <line x1="6" y1="12" x2="10" y2="12" />
                    <line x1="8" y1="10" x2="8" y2="14" />
                    <circle cx="15" cy="13" r="1" />
                    <circle cx="18" cy="11" r="1" />
                </svg>
                <span>Controller Settings</span>
            </button>

            <!-- Diagnostics / Error Display -->
            <div>
                {#if connectionError}
                    <span class="text-rose-400 text-[11px]">{connectionError}</span>
                {:else if gp}
                    <span class="text-slate-400 text-[11px]">Gamepad: {gp.id.slice(0, 20)}</span>
                {:else}
                    <span class="text-slate-500 text-[11px]">Ready</span>
                {/if}
            </div>
        </div>
    </footer>

    <!-- Controller Settings Modal with Live Response Graph -->
    <ControllerSettingsModal
        bind:isOpen={isControllerModalOpen}
        bind:config={rateConfig}
        currentStick={rawRollStick}
    />
</div>
