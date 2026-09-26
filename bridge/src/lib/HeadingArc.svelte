<script lang="ts">
  import { onMount } from 'svelte';

  let {
    roll = 0
  } = $props<{
    roll?: number;
  }>();

  let container: HTMLDivElement;
  let canvas: HTMLCanvasElement;
  let width = $state(300);
  let height = $state(200);

  const COLOR_LINE = '#ffffff';

  // --- Heading Scale (track-up only): fixed arc + nose, rotating world ticks ---
  function drawRollScale(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    arcRadius: number,
    scale: number
  ) {
    ctx.save();
    ctx.translate(cx, cy);

    // Fixed outer nose (always at top, never moves)
    ctx.fillStyle = COLOR_LINE;
    ctx.beginPath();
    ctx.moveTo(0, -arcRadius);
    ctx.lineTo(-6 * scale, -(arcRadius + 12 * scale));
    ctx.lineTo(6 * scale, -(arcRadius + 12 * scale));
    ctx.closePath();
    ctx.fill();

    // Fixed arc rail (never rotates)
    ctx.strokeStyle = COLOR_LINE;
    ctx.lineWidth = Math.max(1.5, 2 * scale);
    ctx.beginPath();
    const rad60 = (60 * Math.PI) / 180;
    ctx.arc(0, 0, arcRadius, -Math.PI / 2 - rad60, -Math.PI / 2 + rad60);
    ctx.stroke();

    // World-fixed headings slide along the fixed arc as yaw changes
    const tickLen = 8 * scale;

    for (let abs = 0; abs < 360; abs += 10) {
      // Screen offset of this world heading relative to nose (-180..180)
      const offset = ((((abs - roll) % 360) + 540) % 360) - 180;
      if (Math.abs(offset) > 60) continue;

      const rad = ((offset - 90) * Math.PI) / 180;

      // Inner tick mark (rotates with world)
      ctx.beginPath();
      ctx.strokeStyle = COLOR_LINE;
      ctx.lineWidth = Math.max(1.5, 2 * scale);
      ctx.moveTo(Math.cos(rad) * arcRadius, Math.sin(rad) * arcRadius);
      ctx.lineTo(Math.cos(rad) * (arcRadius - tickLen), Math.sin(rad) * (arcRadius - tickLen));
      ctx.stroke();

      // Inner label (rotates with world, skip under nose)
      if (Math.abs(offset) > 4) {
        const labelR = arcRadius - tickLen - 11 * scale;
        ctx.fillStyle = COLOR_LINE;
        ctx.font = `bold ${Math.max(9, Math.round(10 * scale))}px monospace`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(
          abs.toString().padStart(3, '0'),
          Math.cos(rad) * labelR,
          Math.sin(rad) * labelR
        );
      }
    }

    ctx.restore();
  }

  function render() {
    if (!canvas || width <= 0 || height <= 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.floor(width * dpr);
    const targetH = Math.floor(height * dpr);

    if (canvas.width !== targetW || canvas.height !== targetH) {
      canvas.width = targetW;
      canvas.height = targetH;
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, width, height);

    const scale = Math.min(Math.max(width / 440, 0.6), 1.4);
    const cx = width / 2;
    const cy = height / 2;
    const arcRadius = Math.max(60, Math.min(cx * 0.7, cy * 0.85));

    drawRollScale(ctx, cx, cy, arcRadius, scale);

    ctx.restore();
  }

  $effect(() => {
    render();
  });

  onMount(() => {
    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        width = entry.contentRect.width;
        height = entry.contentRect.height;
      }
      render();
    });

    if (container) ro.observe(container);

    return () => ro.disconnect();
  });
</script>

<div
  bind:this={container}
  class="w-full h-full overflow-hidden pointer-events-none select-none"
>
  <canvas bind:this={canvas} class="block w-full h-full"></canvas>
</div>
