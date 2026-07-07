using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Continuum Crowds tuning constants (spec §2.2). Singleton component,
  /// Burst-readable from any job. Names are preserved verbatim from the
  /// reference repo (yohash/ContinuumCrowds Constants.cs) so the two
  /// codebases remain cross-checkable — do not rename to C# conventions.
  ///
  /// Authored via <c>CCConstantsAsset</c> (ScriptableObject) and baked by
  /// <c>CCConstantsAuthoring</c>; hot-reloaded each frame in Editor builds
  /// by <c>CCConstantsHotReloadSystem</c>.
  /// </summary>
  public struct CCConstants : IComponentData
  {
    // --- Density / footprint ---
    /// <summary>Extra radial footprint extent beyond unit size (repo default 0).</summary>
    public float u_unitRadialFalloff;
    /// <summary>
    /// Density splat exponent λ (spec §6.4). Not a repo constant: the repo
    /// delegates the footprint to the consumer; the spec adopts the paper's
    /// §4.1 splat kernel, and λ parameterizes it. Default 2 ⇒ ρ̄ = 0.25.
    /// </summary>
    public float lambda;
    /// <summary>
    /// ρ̄ — the minimum contribution a unit makes to its own cell and the
    /// maximum it makes to any neighboring cell (paper §4.1). Derived:
    /// ρ̄ = 1 / 2^λ. Config invariant (asserted): f_rhoMin ≥ ρ̄, so an
    /// isolated unit always moves at topographical speed — it can never
    /// congest itself.
    /// </summary>
    public float rhoBar;

    // --- Predictive velocity (repo extension; spec §7) ---
    /// <summary>Speed above which the dynamic (predictive) footprint applies (default 0.25).</summary>
    public float v_dynamicFootprintThreshold;
    /// <summary>Seconds of velocity extrapolation (default 1.0).</summary>
    public float v_predictiveSeconds;
    /// <summary>Predictive ghost-stamp scale at t=0 (default 0.3).</summary>
    public float v_scaleMax;
    /// <summary>Predictive ghost-stamp scale at t=v_predictiveSeconds (default 0.25).</summary>
    public float v_scaleMin;
    /// <summary>
    /// Cap (in cells) on the predictive extrapolation distance (spec §6.3,
    /// spec-added, default 8). Without it, R_max = footprint +
    /// v_predictiveSeconds·f_speedMax/CellSize would be 20+ cells at default
    /// speeds and the stamping-hash buckets would grow unboundedly; the spec
    /// mandates clamping the extrapolation distance instead. Also bounds the
    /// scatter-reference ghost-chain length (CCStampOps.MaxGhosts).
    /// </summary>
    public float v_predictiveDistanceCapCells;

    // --- Speed field ---
    public float f_slopeMax;   // default  1.0
    public float f_slopeMin;   // default -1.0
    public float f_rhoMax;     // default  0.8
    public float f_rhoMin;     // default  0.3
    public float f_speedMin;   // default  0 (also the clamp floor for flow speed)
    public float f_speedMax;   // default 20

    // --- Cost weights ---
    public float C_alpha;      // path length weight (default 1)
    public float C_beta;       // time weight (default 1)
    public float C_gamma;      // discomfort weight (default 1)

    // --- Eikonal quadratic-root blend (spec §9.4, Decision D9) ---
    public float maxWeight;    // default 2.5
    public float minWeight;    // default 1.0

    /// <summary>
    /// Speed on flat ground (repo Constants.FlatSpeed helper):
    /// f_speedMax + (−f_slopeMin)/(f_slopeMax − f_slopeMin) · (f_speedMin − f_speedMax).
    /// </summary>
    public float FlatSpeed =>
      f_speedMax + (-f_slopeMin) / (f_slopeMax - f_slopeMin) * (f_speedMin - f_speedMax);

    /// <summary>
    /// Config-time invariant (spec §2.3/§6.4): f_rhoMin ≥ ρ̄ guarantees an
    /// isolated unit's own-cell density stays in the topographical regime.
    /// </summary>
    public bool IsValid =>
      f_rhoMin >= rhoBar
      && f_slopeMax > f_slopeMin
      && f_rhoMax > f_rhoMin
      && f_speedMax >= f_speedMin
      && maxWeight + minWeight > 0f
      && lambda > 0f;

    /// <summary>Defaults verified against yohash/ContinuumCrowds Constants.cs.</summary>
    public static CCConstants Defaults => new CCConstants {
      u_unitRadialFalloff = 0f,
      lambda = 2f,
      rhoBar = 0.25f,                    // 1 / 2^λ with λ = 2
      v_dynamicFootprintThreshold = 0.25f,
      v_predictiveSeconds = 1f,
      v_scaleMax = 0.3f,
      v_scaleMin = 0.25f,
      v_predictiveDistanceCapCells = 8f,
      f_slopeMax = 1f,
      f_slopeMin = -1f,
      f_rhoMax = 0.8f,
      f_rhoMin = 0.3f,
      f_speedMin = 0f,
      f_speedMax = 20f,
      C_alpha = 1f,
      C_beta = 1f,
      C_gamma = 1f,
      maxWeight = 2.5f,
      minWeight = 1f,
    };

    /// <summary>Re-derive ρ̄ from λ. Call after any change to lambda.</summary>
    public void DeriveRhoBar() => rhoBar = math.pow(0.5f, lambda);
  }
}
