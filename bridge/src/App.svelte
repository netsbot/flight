<script lang="ts">
    import Horizon from "./lib/Horizon.svelte";
    import GpsMap from "./lib/GpsMap.svelte";

    let gp = $state<Gamepad | null>(null);
    let pollHandle: number | null = null;
    let yaw = $state(192);
    let coords = $state<[number, number]>([1.3521, 103.8198]);
    let speed = $state(45);
    let altitude = $state(120);

    let roll = $derived(gp && gp.axes[2] !== undefined ? gp.axes[2] * 45 : 0);
    let pitch = $derived(gp && gp.axes[3] !== undefined ? -gp.axes[3] * 30 : 0);

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
            if (Math.abs(currentGp.axes[0] ?? 0) > 0.05) {
                yaw = (yaw + currentGp.axes[0] * 1.5 + 360) % 360;
            }
            // Left stick Y (throttle) drives airspeed for tape demo
            const throttle = currentGp.axes[1] ?? 0;
            if (Math.abs(throttle) > 0.05) {
                speed = Math.max(0, Math.min(150, speed - throttle * 0.6));
            }
            // Pitch integrates altitude for tape demo
            altitude = Math.max(0, altitude + (pitch / 30) * 0.25);
        }

        pollHandle = requestAnimationFrame(pollGamepad);
    }
</script>

<svelte:window
    ongamepadconnected={addGamepad}
    ongamepaddisconnected={removeGamepad}
/>
<div class="grid h-1/2 w-full grid-cols-2 gap-2">
<div class="min-h-0 min-w-0">
<Horizon {pitch} {roll} {yaw} {speed} {altitude} />
</div>

<div class="min-h-0 min-w-0">
<GpsMap coords={coords} {yaw} />
</div>
</div>
