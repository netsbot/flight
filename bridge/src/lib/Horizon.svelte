<script lang="ts">
  import { onMount } from 'svelte';

  let {
    pitch = 0,         // degrees: +nose up, -nose down
    roll = 0,          // degrees: +right bank, -left bank
    yaw = 0,           // degrees: 0..360
    speed = 0,         // airspeed (knots or km/h)
    altitude = 0,      // altitude (m or ft)
    width: propWidth,  // optional explicit width in px
    height: propHeight // optional explicit height in px
  } = $props<{
    pitch?: number;
    roll?: number;
    yaw?: number;
    speed?: number;
    altitude?: number;
    width?: number;
    height?: number;
  }>();

  let container: HTMLDivElement;
  let canvas: HTMLCanvasElement;
  let measuredWidth = $state(400);
  let measuredHeight = $state(300);

  let width = $derived(propWidth ?? measuredWidth);
  let height = $derived(propHeight ?? measuredHeight);

  // Garmin G1000 Color Palette
  const COLOR_SKY = '#1b5faa';      // G1000 deep sky blue
  const COLOR_GROUND = '#6f421b';   // G1000 earth brown
  const COLOR_LINE = '#ffffff';
  const COLOR_AIRCRAFT = '#ffde00';  // G1000 high-vis yellow

  function norm360(deg: number): number {
    return ((deg % 360) + 360) % 360;
  }

  // --- 1. Edge-to-Edge Filled Horizon (Sky & Ground) ---
  function drawSkyAndGround(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    w: number,
    h: number,
    pitchPxPerDeg: number
  ) {
    const diagonal = Math.hypot(w, h) * 1.6;

    ctx.save();
    // Clip to full viewport
    ctx.beginPath();
    ctx.rect(0, 0, w, h);
    ctx.clip();

    ctx.translate(cx, cy);
    ctx.rotate((-roll * Math.PI) / 180);
    ctx.translate(0, pitch * pitchPxPerDeg);

    // Sky & Ground
    ctx.fillStyle = COLOR_SKY;
    ctx.fillRect(-diagonal, -diagonal, diagonal * 2, diagonal);

    ctx.fillStyle = COLOR_GROUND;
    ctx.fillRect(-diagonal, 0, diagonal * 2, diagonal);

    // White Horizon Dividing Line
    ctx.strokeStyle = COLOR_LINE;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-diagonal, 0);
    ctx.lineTo(diagonal, 0);
    ctx.stroke();

    ctx.restore();
  }

  // --- 2. G1000 Pitch Ladder (Strictly Clipped Below Arc) ---
  function drawPitchLadder(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    arcCy: number,
    arcRadius: number,
    scale: number,
    pitchPxPerDeg: number
  ) {
    ctx.save();

    // STRICT CLIP: Do not render any pitch markings past the roll arc
    const clipTop = arcCy - arcRadius + 18 * scale;
    ctx.beginPath();
    ctx.rect(0, clipTop, width, height - clipTop);
    ctx.clip();

    // Transform into pitch/roll aircraft frame
    ctx.translate(cx, cy);
    ctx.rotate((-roll * Math.PI) / 180);
    ctx.translate(0, pitch * pitchPxPerDeg);

    ctx.fillStyle = COLOR_LINE;
    ctx.font = `bold ${Math.max(10, Math.round(11 * scale))}px monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    for (let deg = -80; deg <= 80; deg += 2.5) {
      if (deg === 0) continue;

      const y = -deg * pitchPxPerDeg;
      const isMajor = Math.abs(deg % 5) < 1e-6;
      const rungHalfW = (isMajor ? 32 : 16) * scale;

      ctx.beginPath();
      ctx.strokeStyle = COLOR_LINE;
      ctx.lineWidth = 2 * scale;

      // Plain solid rung
      ctx.moveTo(-rungHalfW, y);
      ctx.lineTo(rungHalfW, y);
      ctx.stroke();

      if (isMajor) {
        const val = Math.abs(deg).toString();
        const textOffset = rungHalfW + 12 * scale;
        ctx.fillText(val, -textOffset, y);
        ctx.fillText(val, textOffset, y);
      }
    }

    ctx.restore();
  }

  // --- 3. G1000 Roll Arc ---
  function drawRollScale(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    arcRadius: number,
    scale: number
  ) {
    ctx.save();
    ctx.translate(cx, cy);

    const ticks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
    ctx.strokeStyle = COLOR_LINE;
    ctx.lineWidth = Math.max(1.5, 2 * scale);

    // Roll Arc line
    ctx.beginPath();
    const rad60 = (60 * Math.PI) / 180;
    ctx.arc(0, 0, arcRadius, -Math.PI / 2 - rad60, -Math.PI / 2 + rad60);
    ctx.stroke();

    for (const deg of ticks) {
      const rad = ((deg - 90) * Math.PI) / 180;
      const isMajor = Math.abs(deg) === 30 || Math.abs(deg) === 60 || deg === 0;
      const isDot = Math.abs(deg) === 45;
      const tickLen = (isMajor ? 12 : 6) * scale;

      ctx.beginPath();
      if (isDot) {
        const dotR = arcRadius + 6 * scale;
        ctx.arc(Math.cos(rad) * dotR, Math.sin(rad) * dotR, 2.5 * scale, 0, Math.PI * 2);
        ctx.fillStyle = COLOR_LINE;
        ctx.fill();
      } else {
        ctx.moveTo(Math.cos(rad) * arcRadius, Math.sin(rad) * arcRadius);
        ctx.lineTo(Math.cos(rad) * (arcRadius + tickLen), Math.sin(rad) * (arcRadius + tickLen));
        ctx.stroke();
      }
    }

    // Fixed outer reference triangle (top)
    ctx.fillStyle = COLOR_LINE;
    ctx.beginPath();
    ctx.moveTo(0, -arcRadius);
    ctx.lineTo(-6 * scale, -(arcRadius + 12 * scale));
    ctx.lineTo(6 * scale, -(arcRadius + 12 * scale));
    ctx.closePath();
    ctx.fill();

    // Rotating inner pointer (inside arc, moves with roll)
    ctx.rotate((-roll * Math.PI) / 180);

    // Roll pointer (inside, points outward to arc)
    ctx.beginPath();
    ctx.moveTo(0, -(arcRadius));
    ctx.lineTo(-6 * scale, -(arcRadius - 12 * scale));
    ctx.lineTo(6 * scale, -(arcRadius - 12 * scale));
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  // --- 5. G1000 Airspeed Tape (left) ---
  function drawSpeedTape(
    ctx: CanvasRenderingContext2D,
    leftX: number,
    cy: number,
    tapeH: number,
    scale: number
  ) {
    const tapeW = 52 * scale;
    const pxPerUnit = tapeH / 60; // ±30 visible
    const span = 30;

    ctx.save();

    // Background strip
    ctx.fillStyle = 'rgba(2, 6, 23, 0.85)';
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    ctx.fillRect(leftX, cy - tapeH / 2, tapeW, tapeH);
    ctx.strokeRect(leftX, cy - tapeH / 2, tapeW, tapeH);

    // Clip scrolling scale to strip
    ctx.beginPath();
    ctx.rect(leftX, cy - tapeH / 2, tapeW, tapeH);
    ctx.clip();

    ctx.font = `bold ${Math.max(9, Math.round(10 * scale))}px monospace`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const minV = Math.max(0, Math.floor((speed - span) / 5) * 5);
    const maxV = Math.ceil((speed + span) / 5) * 5;

    for (let s = minV; s <= maxV; s += 5) {
      const y = cy - (s - speed) * pxPerUnit;
      const isMajor = s % 10 === 0;

      ctx.beginPath();
      ctx.strokeStyle = COLOR_LINE;
      ctx.lineWidth = (isMajor ? 2 : 1.2) * scale;
      ctx.moveTo(leftX + tapeW, y);
      ctx.lineTo(leftX + tapeW - (isMajor ? 9 : 5) * scale, y);
      ctx.stroke();

      if (isMajor) {
        ctx.fillStyle = COLOR_LINE;
        ctx.fillText(s.toString(), leftX + tapeW - 12 * scale, y);
      }
    }

    ctx.restore();

    // Current value box + pointer
    ctx.save();
    const boxW = 46 * scale;
    const boxH = 20 * scale;
    const boxX = leftX - 6 * scale;
    ctx.fillStyle = '#020617';
    ctx.strokeStyle = COLOR_LINE;
    ctx.lineWidth = 1.2;
    ctx.fillRect(boxX, cy - boxH / 2, boxW, boxH);
    ctx.strokeRect(boxX, cy - boxH / 2, boxW, boxH);

    // Pointer notch into tape
    ctx.beginPath();
    ctx.moveTo(leftX + tapeW, cy - 5 * scale);
    ctx.lineTo(leftX + tapeW + 6 * scale, cy);
    ctx.lineTo(leftX + tapeW, cy + 5 * scale);
    ctx.closePath();
    ctx.fillStyle = '#020617';
    ctx.fill();

    ctx.fillStyle = COLOR_LINE;
    ctx.font = `bold ${Math.max(10, Math.round(11 * scale))}px monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(Math.round(speed).toString(), boxX + boxW / 2, cy);
    ctx.restore();
  }

  // --- 6. G1000 Altitude Tape (right) ---
  function drawAltitudeTape(
    ctx: CanvasRenderingContext2D,
    rightX: number,
    cy: number,
    tapeH: number,
    scale: number
  ) {
    const tapeW = 56 * scale;
    const pxPerUnit = tapeH / 120; // ±60 visible
    const span = 60;

    ctx.save();

    // Background strip
    ctx.fillStyle = 'rgba(2, 6, 23, 0.85)';
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    ctx.fillRect(rightX, cy - tapeH / 2, tapeW, tapeH);
    ctx.strokeRect(rightX, cy - tapeH / 2, tapeW, tapeH);

    // Clip scrolling scale to strip
    ctx.beginPath();
    ctx.rect(rightX, cy - tapeH / 2, tapeW, tapeH);
    ctx.clip();

    ctx.font = `bold ${Math.max(9, Math.round(10 * scale))}px monospace`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    const minV = Math.max(0, Math.floor((altitude - span) / 10) * 10);
    const maxV = Math.ceil((altitude + span) / 10) * 10;

    for (let a = minV; a <= maxV; a += 10) {
      const y = cy - (a - altitude) * pxPerUnit;
      const isMajor = a % 20 === 0;

      ctx.beginPath();
      ctx.strokeStyle = COLOR_LINE;
      ctx.lineWidth = (isMajor ? 2 : 1.2) * scale;
      ctx.moveTo(rightX, y);
      ctx.lineTo(rightX + (isMajor ? 9 : 5) * scale, y);
      ctx.stroke();

      if (isMajor) {
        ctx.fillStyle = COLOR_LINE;
        ctx.fillText(a.toString(), rightX + 12 * scale, y);
      }
    }

    ctx.restore();

    // Current value box + pointer
    ctx.save();
    const boxW = 52 * scale;
    const boxH = 20 * scale;
    const boxX = rightX + tapeW - boxW + 6 * scale;
    ctx.fillStyle = '#020617';
    ctx.strokeStyle = COLOR_LINE;
    ctx.lineWidth = 1.2;
    ctx.fillRect(boxX, cy - boxH / 2, boxW, boxH);
    ctx.strokeRect(boxX, cy - boxH / 2, boxW, boxH);

    // Pointer notch into tape
    ctx.beginPath();
    ctx.moveTo(rightX, cy - 5 * scale);
    ctx.lineTo(rightX - 6 * scale, cy);
    ctx.lineTo(rightX, cy + 5 * scale);
    ctx.closePath();
    ctx.fillStyle = '#020617';
    ctx.fill();

    ctx.fillStyle = COLOR_LINE;
    ctx.font = `bold ${Math.max(10, Math.round(11 * scale))}px monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(Math.round(altitude).toString(), boxX + boxW / 2, cy);
    ctx.restore();
  }

  // --- 4. G1000 Yellow Aircraft Reference Symbol ---
  function drawAircraftSymbol(
    ctx: CanvasRenderingContext2D,
    cx: number,
    cy: number,
    scale: number
  ) {
    ctx.save();
    ctx.translate(cx, cy);

    ctx.fillStyle = COLOR_AIRCRAFT;
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = Math.max(1.5, 2 * scale);

    const wingSpan = 70 * scale;
    const wingInner = 24 * scale;
    const wingThick = 5 * scale;

    // Left Wing Bar
    ctx.beginPath();
    ctx.rect(-wingSpan, -wingThick / 2, wingSpan - wingInner, wingThick);
    ctx.fill();
    ctx.stroke();

    // Right Wing Bar
    ctx.beginPath();
    ctx.rect(wingInner, -wingThick / 2, wingSpan - wingInner, wingThick);
    ctx.fill();
    ctx.stroke();

    // Center Inverted-V Chevron (G1000 symbol)
    const chW = 18 * scale;
    const chH = 10 * scale;
    ctx.beginPath();
    ctx.moveTo(0, -chH);
    ctx.lineTo(chW, 2 * scale);
    ctx.lineTo(chW * 0.7, 4 * scale);
    ctx.lineTo(0, -chH * 0.4);
    ctx.lineTo(-chW * 0.7, 4 * scale);
    ctx.lineTo(-chW, 2 * scale);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

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

    // Uniform geometric scaling (prevents distortion regardless of aspect ratio)
    const scale = Math.min(Math.max(width / 440, 0.6), Math.max(height / 360, 0.6));
    const cx = width / 2;
    const cy = height / 2 + 10 * scale;
    const arcCy = cy - 48 * scale;
    const arcRadius = Math.max(85, Math.min(cx * 0.7, (arcCy - 36 * scale) * 0.9));
    const pitchPxPerDeg = 6.0 * scale;

    // 1. Sky & Ground (Full Bleed)
    drawSkyAndGround(ctx, cx, cy, width, height, pitchPxPerDeg);

    // 2. Pitch Ladder (STRICTLY clipped so nothing renders past the arc)
    drawPitchLadder(ctx, cx, cy, arcCy, arcRadius, scale, pitchPxPerDeg);

    // 3. Roll Arc & Slip/Skid Brick
    drawRollScale(ctx, cx, arcCy, arcRadius, scale);

    // 4. Yellow Aircraft Reference Symbol
    drawAircraftSymbol(ctx, cx, cy, scale);

    // 5. Airspeed + Altitude tapes (G1000 sides, centered on horizon)
    const tapeH = Math.min(height * 0.62, arcRadius * 1.8);
    drawSpeedTape(ctx, 10 * scale, cy, tapeH, scale);
    drawAltitudeTape(ctx, width - 66 * scale, cy, tapeH, scale);

    ctx.restore();
  }

  $effect(() => {
    render();
  });

  onMount(() => {
    if (propWidth && propHeight) {
      render();
      return;
    }

    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        measuredWidth = entry.contentRect.width;
        measuredHeight = entry.contentRect.height;
      }
      render();
    });

    if (container) ro.observe(container);

    return () => ro.disconnect();
  });
</script>

<div
  bind:this={container}
  style="{propWidth ? `width: ${propWidth}px;` : ''} {propHeight ? `height: ${propHeight}px;` : ''}"
  class="relative {propWidth ? '' : 'w-full'} {propHeight ? '' : 'h-full'} min-h-[200px] flex items-center justify-center overflow-hidden touch-none select-none"
>
  <canvas
    bind:this={canvas}
    class="block w-full h-full"
  ></canvas>
</div>
