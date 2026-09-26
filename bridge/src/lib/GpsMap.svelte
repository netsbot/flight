<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Map, Marker, type StyleSpecification, type GeoJSONSource } from 'maplibre-gl';
  import HeadingArc from './HeadingArc.svelte';
  import 'maplibre-gl/dist/maplibre-gl.css';

  let {
    coords = [1.3521, 103.8198], // [latitude, longitude] or { lat, lon }
    yaw = 0,                     // degrees heading (0 = North)
    zoom = 16,
    showTrail = true
  } = $props<{
    coords?: [number, number] | { lat: number; lon: number };
    yaw?: number;
    zoom?: number;
    showTrail?: boolean;
  }>();

  let mapContainer: HTMLDivElement;
  let map: Map | null = null;
  let marker: Marker | null = null;
  let resizeObserver: ResizeObserver | null = null;
  let follow = $state(true);

  function recenter() {
    follow = true;
    map?.easeTo({
      center: [lon, lat],
      bearing: yaw,
      zoom,
      duration: 300
    });
  }

  // Normalizing lat/lon (aviation convention: [lat, lon])
  let lat = $derived(Array.isArray(coords) ? coords[0] : coords.lat);
  let lon = $derived(Array.isArray(coords) ? coords[1] : coords.lon);

  // Free Esri World Imagery Satellite Raster Style
  const SATELLITE_STYLE: StyleSpecification = {
    version: 8,
    sources: {
      'esri-satellite': {
        type: 'raster',
        tiles: [
          'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
        ],
        tileSize: 256,
        maxzoom: 19,
        attribution: 'Esri, Maxar, Earthstar Geographics'
      }
    },
    layers: [
      {
        id: 'esri-satellite-layer',
        type: 'raster',
        source: 'esri-satellite',
        minzoom: 0,
        maxzoom: 19
      }
    ]
  };

  const trailCoordinates: [number, number][] = [];

  onMount(() => {
    map = new Map({
      container: mapContainer,
      style: SATELLITE_STYLE,
      center: [lon, lat],
      zoom,
      bearing: yaw,
      pitch: 0,
      attributionControl: false
    });

    // Navigation arrow marker (track-up: always points up)
    const el = document.createElement('div');
    el.style.width = '32px';
    el.style.height = '32px';
    el.style.display = 'flex';
    el.style.alignItems = 'center';
    el.style.justifyContent = 'center';
    el.innerHTML = `
      <svg width="26" height="26" viewBox="0 0 24 24" fill="#ffde00" stroke="#000000" stroke-width="1.8" stroke-linejoin="round" style="filter: drop-shadow(0 2px 4px rgba(0,0,0,0.6));">
        <path d="M12 2 L19 21 L12 16.5 L5 21 Z" />
      </svg>
    `;

    marker = new Marker({
      element: el,
      rotationAlignment: 'viewport'
    })
      .setLngLat([lon, lat])
      .addTo(map);

    map.on('load', () => {
      if (!map) return;

      // Flight trail breadcrumbs
      map.addSource('flight-trail', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: trailCoordinates },
          properties: {}
        }
      });

      map.addLayer({
        id: 'flight-trail-line',
        type: 'line',
        source: 'flight-trail',
        layout: {
          'line-join': 'round',
          'line-cap': 'round'
        },
        paint: {
          'line-color': '#00e5ff',
          'line-width': 2.5,
          'line-opacity': 0.85
        }
      });
    });

    // Auto-resize when container dimensions change
    resizeObserver = new ResizeObserver(() => {
      map?.resize();
    });
    if (mapContainer) resizeObserver.observe(mapContainer);

    // User panning breaks follow; recenter button restores it
    map.on('dragstart', () => {
      follow = false;
    });
  });

  // Reactive telemetry updates (track-up only)
  $effect(() => {
    if (!map || !marker) return;

    marker.setLngLat([lon, lat]);
    marker.setRotation(0);
    if (follow) {
      map.easeTo({
        center: [lon, lat],
        bearing: yaw,
        duration: 100
      });
    }

    // Update flight trail
    if (showTrail) {
      const lastPoint = trailCoordinates[trailCoordinates.length - 1];
      if (!lastPoint || Math.hypot(lastPoint[0] - lon, lastPoint[1] - lat) > 0.00002) {
        trailCoordinates.push([lon, lat]);

        const source = map.getSource('flight-trail') as GeoJSONSource | undefined;
        source?.setData({
          type: 'Feature',
          geometry: {
            type: 'LineString',
            coordinates: trailCoordinates
          },
          properties: {}
        });
      }
    }
  });

  onDestroy(() => {
    resizeObserver?.disconnect();
    map?.remove();
  });
</script>

<div class="relative w-full h-full min-h-35 rounded-md overflow-hidden border border-slate-700 bg-slate-950 shadow-xl select-none">
  <div bind:this={mapContainer} class="w-full h-full"></div>

  <!-- Heading indicator (fixed nose, rotating inner scale) -->
  <div class="absolute inset-0 pointer-events-none">
    <HeadingArc roll={yaw} />
  </div>

  <!-- Center on aircraft -->
  <button
    type="button"
    onclick={recenter}
    title="Center on aircraft"
    aria-label="Center on aircraft"
    class="absolute bottom-8 right-2 flex h-8 w-8 items-center justify-center rounded-md border border-slate-700 bg-slate-950/80 text-slate-200 backdrop-blur hover:bg-slate-900 {follow ? 'text-amber-400' : ''}"
  >
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round">
      <circle cx="12" cy="12" r="3" />
      <path d="M12 2v3M12 19v3M2 12h3M19 12h3" />
      <circle cx="12" cy="12" r="8" stroke-dasharray="2 2" />
    </svg>
  </button>

  <!-- Coordinates HUD Badge -->
  <div class="absolute bottom-1 left-2 font-mono text-[10px] text-slate-200 bg-slate-950/80 backdrop-blur px-1.5 py-0.5 rounded border border-slate-800 pointer-events-none flex items-center gap-1.5">
    <span class="text-amber-400 font-semibold">TRK-UP</span>
    <span>{lat.toFixed(5)}°, {lon.toFixed(5)}°</span>
  </div>
</div>
