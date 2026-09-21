<script lang="ts">
    let gp: Gamepad | null = $state(null);
    let pollHandle: number | null = null;

    function addGamepad(e: GamepadEvent) {
        gp = e.gamepad;

        pollGamepad()
    }

    function removeGamepad() {
        gp = null;
        
        if (pollHandle !== null) {
            cancelAnimationFrame(pollHandle);
        }
    }

    function pollGamepad() {
        if (gp === null) {
            return;
        }

        gp = navigator.getGamepads()[gp.index];
        pollHandle = requestAnimationFrame(pollGamepad);
    }
</script>

<svelte:window ongamepadconnected={addGamepad} ongamepaddisconnected={removeGamepad} />

{#if gp !== null}
	<p>{gp.axes}</p>
{:else}
	<p>please connect a gamepad</p>
{/if}

