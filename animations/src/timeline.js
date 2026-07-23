// Deterministic mapping from animation time -> build state.
// Pure functions only: identical (t, params) always yields identical state,
// which is what makes headless frame export frame-accurate.

const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
export const smooth = (x) => { x = clamp(x, 0, 1); return x * x * (3 - 2 * x); };

// Returns build state at absolute time `t` (seconds) for the given params.
//   layerCount   : total number of layers in the part
//   layerIndex   : index of the layer currently being formed (0-based, clamped)
//   layerProgress: 0..1 sweep progress across the current layer
//   completedH   : world height already fully solid (top of finished layers)
//   currentTopH  : world height of the leading edge (completedH + partial reveal)
//   done         : true once the whole part is finished
export function buildState(t, params) {
  const { duration, partHeight, layerHeight } = params;
  const detailLayers = Math.round(params.detailLayers);
  const detailFrac = params.detailFrac;

  const layerCount = Math.max(1, Math.round(partHeight / layerHeight));
  const dLayers = Math.min(detailLayers, layerCount);

  const tn = clamp(t / duration, 0, 1);

  // Piecewise layer schedule: the first `dLayers` layers occupy the first
  // `detailFrac` of the timeline (slow, so the process reads on camera);
  // the remaining layers fill the rest (fast montage to finish the part).
  let layerFloat;
  if (tn <= detailFrac || dLayers >= layerCount) {
    const p = detailFrac > 0 ? tn / detailFrac : 1;
    layerFloat = smooth(p) * dLayers;
  } else {
    const p = (tn - detailFrac) / (1 - detailFrac);
    layerFloat = dLayers + smooth(p) * (layerCount - dLayers);
  }
  layerFloat = clamp(layerFloat, 0, layerCount);

  const done = layerFloat >= layerCount - 1e-6;
  const layerIndex = clamp(Math.floor(layerFloat), 0, layerCount - 1);
  const layerProgress = done ? 1 : (layerFloat - layerIndex);

  const completedH = layerIndex * layerHeight;
  const currentTopH = completedH + layerProgress * layerHeight;

  return {
    layerCount, layerIndex, layerProgress,
    completedH, currentTopH,
    totalH: layerCount * layerHeight,
    done, tn,
  };
}
