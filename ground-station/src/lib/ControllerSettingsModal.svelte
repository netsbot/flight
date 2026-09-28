<script lang="ts">
  import { onMount } from 'svelte';
  import {
    type RateConfig,
    evaluateCurve,
    stickToRate,
    DEFAULT_RATE_CONFIG
  } from './controller';

  let {
    isOpen = $bindable(false),
    config = $bindable<RateConfig>({ ...DEFAULT_RATE_CONFIG }),
    currentStick = 0 // live stick position from gamepad (-1.0 to 1.0)
  } = $props<{
    isOpen?: boolean;
    config?: RateConfig;
    currentStick?: number;
  }>();

  let canvas = $state<HTMLCanvasElement>();

  function setPreset(preset: 'cinematic' | 'freestyle' | 'racing') {
    if (preset === 'cinematic') {
      config.curveType = 'expo';
      config.expo = 0.5;
      config.superRate = 0.0;
      config.maxRate = 400;
      config.deadband = 0.05;
    } else if (preset === 'freestyle') {
      config.curveType = 'superexpo';
      config.expo = 0.4;
      config.superRate = 0.65;
      config.maxRate = 670;
      config.deadband = 0.05;
    } else if (preset === 'racing') {
      config.curveType = 'superexpo';
      config.expo = 0.25;
      config.superRate = 0.75;
      config.maxRate = 900;
      config.deadband = 0.04;
    }
  }

  function drawGraph() {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;

    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    const pad = 24;
    const plotW = w - pad * 2;
    const plotH = h - pad * 2;
    const cx = pad + plotW / 2;
    const cy = pad + plotH / 2;

    // Background Grid
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;

    // Center Axes
    ctx.beginPath();
    ctx.moveTo(pad, cy);
    ctx.lineTo(pad + plotW, cy);
    ctx.moveTo(cx, pad);
    ctx.lineTo(cx, pad + plotH);
    ctx.stroke();

    // 50% lines
    ctx.setLineDash([2, 4]);
    ctx.strokeStyle = '#334155';
    ctx.beginPath();
    ctx.moveTo(cx - plotW * 0.25, pad);
    ctx.lineTo(cx - plotW * 0.25, pad + plotH);
    ctx.moveTo(cx + plotW * 0.25, pad);
    ctx.lineTo(cx + plotW * 0.25, pad + plotH);
    ctx.moveTo(pad, cy - plotH * 0.25);
    ctx.lineTo(pad + plotW, cy - plotH * 0.25);
    ctx.moveTo(pad, cy + plotH * 0.25);
    ctx.lineTo(pad + plotW, cy + plotH * 0.25);
    ctx.stroke();
    ctx.setLineDash([]);

    // Deadband Shaded Zone
    if (config.deadband > 0) {
      const deadbandPx = (config.deadband * (plotW / 2));
      ctx.fillStyle = 'rgba(239, 68, 68, 0.08)';
      ctx.fillRect(cx - deadbandPx, pad, deadbandPx * 2, plotH);
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.25)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - deadbandPx, pad);
      ctx.lineTo(cx - deadbandPx, pad + plotH);
      ctx.moveTo(cx + deadbandPx, pad);
      ctx.lineTo(cx + deadbandPx, pad + plotH);
      ctx.stroke();
    }

    // Border box
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.strokeRect(pad, pad, plotW, plotH);

    // Labels on Axis
    ctx.fillStyle = '#64748b';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText('-1.0', pad, cy + 3);
    ctx.fillText('0', cx + 6, cy + 3);
    ctx.fillText('+1.0', pad + plotW, cy + 3);

    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillText(`+${config.maxRate}°`, pad - 3, pad);
    ctx.fillText(`-${config.maxRate}°`, pad - 3, pad + plotH);

    // Plot Curve
    ctx.beginPath();
    ctx.strokeStyle = config.curveType === 'superexpo' ? '#06b6d4' : '#10b981';
    ctx.lineWidth = 2.5;

    const samples = 120;
    for (let i = 0; i <= samples; i++) {
      const t = (i / samples) * 2 - 1; // -1.0 to 1.0 (raw stick)
      const rate = stickToRate(t, config);
      const px = cx + (t * (plotW / 2));
      const py = cy - ((rate / config.maxRate) * (plotH / 2));

      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Live Left Stick Indicator Dot
    const liveRate = stickToRate(currentStick, config);
    const livePx = cx + (currentStick * (plotW / 2));
    const livePy = cy - ((liveRate / config.maxRate) * (plotH / 2));

    // Crosshairs to live position
    ctx.strokeStyle = 'rgba(250, 204, 21, 0.4)';
    ctx.setLineDash([3, 3]);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(livePx, pad);
    ctx.lineTo(livePx, pad + plotH);
    ctx.moveTo(pad, livePy);
    ctx.lineTo(pad + plotW, livePy);
    ctx.stroke();
    ctx.setLineDash([]);

    // Glowing Dot
    ctx.fillStyle = '#facc15';
    ctx.beginPath();
    ctx.arc(livePx, livePy, 5, 0, Math.PI * 2);
    ctx.fill();

    // Value readout badge
    ctx.fillStyle = '#facc15';
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = livePx > cx ? 'right' : 'left';
    ctx.fillText(`${liveRate.toFixed(0)}°/s`, livePx + (livePx > cx ? -8 : 8), livePy - 6);

    ctx.restore();
  }

  $effect(() => {
    if (isOpen) {
      config;
      currentStick;
      drawGraph();
    }
  });

  onMount(() => {
    drawGraph();
  });
</script>

{#if isOpen}
  <div class="fixed inset-0 z-50 flex items-center justify-center bg-black/75 backdrop-blur-sm p-4 animate-in fade-in duration-150">
    <div class="relative w-full max-w-3xl rounded-xl border border-slate-700 bg-slate-900 shadow-2xl overflow-hidden text-slate-200">
      
      <!-- Header -->
      <div class="flex items-center justify-between border-b border-slate-800 px-5 py-3 bg-slate-950/60">
        <div class="flex items-center gap-2">
          <span class="text-sm font-bold tracking-wide uppercase text-cyan-400">Controller Settings</span>
          <span class="text-[10px] font-mono px-2 py-0.5 rounded bg-slate-800 text-slate-400 border border-slate-700">
            {config.curveType === 'superexpo' ? 'Betaflight SuperExpo' : 'Standard Cubic Expo'}
          </span>
        </div>
        <button
          onclick={() => (isOpen = false)}
          class="rounded p-1 text-slate-400 hover:bg-slate-800 hover:text-white transition"
          aria-label="Close modal"
        >
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        </button>
      </div>

      <!-- Body: Split Graph & Controls -->
      <div class="grid grid-cols-1 md:grid-cols-2 gap-4 p-5">
        
        <!-- Left: Canvas Response Graph -->
        <div class="flex flex-col gap-2">
          <div class="flex items-center justify-between text-xs font-mono text-slate-400">
            <span>LEFT STICK RESPONSE</span>
            <span class="text-amber-400 font-semibold">Deflection: {currentStick.toFixed(2)}</span>
          </div>

          <div class="relative w-full h-64 rounded-lg bg-slate-950 border border-slate-800 overflow-hidden shadow-inner">
            <canvas bind:this={canvas} class="w-full h-full block"></canvas>
          </div>

          <!-- Formula Preview -->
          <div class="text-[10px] font-mono p-2 rounded bg-slate-950 border border-slate-800 text-slate-400">
            {#if config.curveType === 'expo'}
              <code>f(x) = (1 - {config.expo.toFixed(2)})x + {config.expo.toFixed(2)}x³</code>
            {:else}
              <code>f(x) = [(1 - {config.expo.toFixed(2)})x + {config.expo.toFixed(2)}x³] / [1 - {config.superRate.toFixed(2)}·|x|]</code>
            {/if}
          </div>
        </div>

        <!-- Right: Curve & Invert Controls -->
        <div class="flex flex-col gap-3">
          
          <!-- Curve Type Selection -->
          <div>
            <span class="text-xs font-medium text-slate-400 mb-1.5 block">Rate Curve Model</span>
            <div class="grid grid-cols-2 gap-2">
              <button
                type="button"
                onclick={() => (config.curveType = 'expo')}
                class="py-1.5 px-3 rounded text-xs font-semibold border transition {config.curveType === 'expo'
                  ? 'bg-emerald-600 border-emerald-500 text-white'
                  : 'bg-slate-800 border-slate-700 text-slate-300 hover:bg-slate-700'}"
              >
                Standard Expo
              </button>
              <button
                type="button"
                onclick={() => (config.curveType = 'superexpo')}
                class="py-1.5 px-3 rounded text-xs font-semibold border transition {config.curveType === 'superexpo'
                  ? 'bg-cyan-600 border-cyan-500 text-white'
                  : 'bg-slate-800 border-slate-700 text-slate-300 hover:bg-slate-700'}"
              >
                SuperExpo (FPV)
              </button>
            </div>
          </div>

          <!-- Axis Inversion Toggles -->
          <div class="p-2.5 rounded-lg bg-slate-950 border border-slate-800 space-y-2">
            <span class="text-[11px] font-mono text-slate-400 block font-semibold">AXIS INVERSION (LEFT STICK)</span>
            <div class="flex items-center justify-between text-xs">
              <label class="flex items-center gap-2 cursor-pointer select-none">
                <input
                  type="checkbox"
                  bind:checked={config.invertRoll}
                  class="accent-cyan-500 rounded cursor-pointer"
                />
                <span>Invert Roll (Left Stick X)</span>
              </label>
              <span class="text-[10px] font-mono text-slate-500">{config.invertRoll ? 'INVERTED' : 'NORMAL'}</span>
            </div>
            <div class="flex items-center justify-between text-xs">
              <label class="flex items-center gap-2 cursor-pointer select-none">
                <input
                  type="checkbox"
                  bind:checked={config.invertPitch}
                  class="accent-cyan-500 rounded cursor-pointer"
                />
                <span>Invert Pitch (Left Stick Y)</span>
              </label>
              <span class="text-[10px] font-mono text-slate-500">{config.invertPitch ? 'INVERTED' : 'NORMAL'}</span>
            </div>
          </div>

          <!-- Sliders -->
          <div class="space-y-2.5 font-mono text-xs">
            <!-- Expo k -->
            <div>
              <div class="flex justify-between mb-0.5 text-slate-300">
                <span>Expo (Center Softness, k):</span>
                <span class="text-cyan-400">{config.expo.toFixed(2)}</span>
              </div>
              <input
                type="range"
                min="0.0"
                max="1.0"
                step="0.02"
                bind:value={config.expo}
                class="w-full accent-cyan-500 cursor-pointer"
              />
            </div>

            <!-- SuperRate s -->
            <div class="{config.curveType !== 'superexpo' ? 'opacity-40 pointer-events-none' : ''}">
              <div class="flex justify-between mb-0.5 text-slate-300">
                <span>Super Rate (Endpoint Boost, s):</span>
                <span class="text-cyan-400">{config.superRate.toFixed(2)}</span>
              </div>
              <input
                type="range"
                min="0.0"
                max="0.85"
                step="0.02"
                bind:value={config.superRate}
                class="w-full accent-cyan-500 cursor-pointer"
              />
            </div>

            <!-- Max Rate -->
            <div>
              <div class="flex justify-between mb-0.5 text-slate-300">
                <span>Max Angular Rate:</span>
                <span class="text-emerald-400">{config.maxRate} °/s</span>
              </div>
              <input
                type="range"
                min="200"
                max="1200"
                step="20"
                bind:value={config.maxRate}
                class="w-full accent-emerald-500 cursor-pointer"
              />
            </div>

            <!-- Deadband -->
            <div>
              <div class="flex justify-between mb-0.5 text-slate-300">
                <span>Stick Deadband:</span>
                <span class="text-amber-400">{(config.deadband * 100).toFixed(0)}%</span>
              </div>
              <input
                type="range"
                min="0.0"
                max="0.15"
                step="0.01"
                bind:value={config.deadband}
                class="w-full accent-amber-500 cursor-pointer"
              />
            </div>
          </div>

          <!-- Presets -->
          <div class="pt-2 border-t border-slate-800">
            <span class="text-[11px] font-medium text-slate-400 mb-1.5 block">Quick Presets</span>
            <div class="flex gap-2">
              <button
                type="button"
                onclick={() => setPreset('cinematic')}
                class="px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-xs font-mono border border-slate-700 transition"
              >
                Cinematic
              </button>
              <button
                type="button"
                onclick={() => setPreset('freestyle')}
                class="px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-xs font-mono border border-slate-700 transition"
              >
                Freestyle
              </button>
              <button
                type="button"
                onclick={() => setPreset('racing')}
                class="px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-xs font-mono border border-slate-700 transition"
              >
                Racing
              </button>
            </div>
          </div>

        </div>

      </div>

      <!-- Footer -->
      <div class="flex justify-end gap-2 border-t border-slate-800 px-5 py-3 bg-slate-950/60">
        <button
          type="button"
          onclick={() => (isOpen = false)}
          class="px-4 py-1.5 rounded bg-cyan-600 hover:bg-cyan-500 active:bg-cyan-700 text-white font-semibold text-xs transition"
        >
          Apply & Close
        </button>
      </div>

    </div>
  </div>
{/if}
