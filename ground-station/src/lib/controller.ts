export type CurveType = 'expo' | 'superexpo';

export interface RateConfig {
  curveType: CurveType;
  expo: number;       // k in [0.0, 1.0]
  superRate: number;  // s in [0.0, 0.85]
  maxRate: number;    // deg/s peak rate at full stick deflection
  deadband: number;   // deadband zone around stick center
  invertRoll: boolean;
  invertPitch: boolean;
}

export const DEFAULT_RATE_CONFIG: RateConfig = {
  curveType: 'superexpo',
  expo: 0.40,
  superRate: 0.65,
  maxRate: 670,
  deadband: 0.05,
  invertRoll: false,
  invertPitch: true // Default Invert Y (Flight stick: Up = Nose Down, Down = Nose Up)
};

export const CONFIG_STORAGE_KEY = 'gcs_controller_settings_v1';

export function loadRateConfig(): RateConfig {
  if (typeof window === 'undefined' || !window.localStorage) {
    return { ...DEFAULT_RATE_CONFIG };
  }
  try {
    const raw = localStorage.getItem(CONFIG_STORAGE_KEY);
    if (raw) {
      return { ...DEFAULT_RATE_CONFIG, ...JSON.parse(raw) };
    }
  } catch (e) {
    console.warn('Failed to load controller config from localStorage:', e);
  }
  return { ...DEFAULT_RATE_CONFIG };
}

export function saveRateConfig(config: RateConfig): void {
  if (typeof window === 'undefined' || !window.localStorage) return;
  try {
    localStorage.setItem(CONFIG_STORAGE_KEY, JSON.stringify(config));
  } catch (e) {
    console.warn('Failed to save controller config to localStorage:', e);
  }
}

/**
 * Calculates normalized stick deflection (-1.0 to 1.0) with center calibration and deadband.
 */
export function stickDeflection(
  raw: number,
  center = 0.0,
  deadband = 0.05
): number {
  const d = raw - center;
  if (Math.abs(d) < deadband) return 0.0;
  const sign = Math.sign(d);
  return sign * Math.min(1.0, (Math.abs(d) - deadband) / (1.0 - deadband));
}

/**
 * Evaluates the mathematical rate curve factor.
 * - Standard Expo:  (1 - k)*x + k*x^3
 * - SuperExpo:      ((1 - k)*x + k*x^3) / (1 - s * abs(x))
 */
export function evaluateCurve(
  x: number,
  curveType: CurveType,
  expo: number,
  superRate: number
): number {
  const k = Math.max(0.0, Math.min(1.0, expo));
  const base = (1.0 - k) * x + k * (x * x * x);

  if (curveType === 'expo') {
    return base;
  }

  const s = Math.max(0.0, Math.min(0.85, superRate));
  const denominator = Math.max(0.02, 1.0 - s * Math.abs(x));
  return base / denominator;
}

/**
 * Maps raw stick input directly to angular rate (degrees/second) based on user's curve configuration.
 */
export function stickToRate(
  rawStick: number,
  config: RateConfig,
  center = 0.0
): number {
  const x = stickDeflection(rawStick, center, config.deadband);
  if (x === 0.0) return 0.0;

  const factor = evaluateCurve(x, config.curveType, config.expo, config.superRate);
  const maxFactor = evaluateCurve(1.0, config.curveType, config.expo, config.superRate);

  return (factor / maxFactor) * config.maxRate;
}
