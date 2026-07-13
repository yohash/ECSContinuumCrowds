using Unity.Entities;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Velocity-gradient scheme selector (Decision D10).
  ///
  /// CentralRepo (shipping): central difference with infinity fallback, then
  /// normalize — the repo's scheme. Blends across shocklines (kinks in φ
  /// behind obstacles / along medial axes), yielding the smoother
  /// trajectories that are the product's purpose. Its ±1 one-sided infinity
  /// fallback is effectively a degenerate upwind difference at obstacle
  /// boundaries, so the shipping scheme is a hybrid: central where φ is
  /// smooth, pseudo-upwind at walls. Known artifact: the fallback axis is
  /// overweighted after normalization near walls, occasionally angling units
  /// more wall-perpendicular than the true characteristic — absorbed in
  /// practice by min-distance + infinite-cost walls; first suspect if units
  /// clip geometry.
  ///
  /// UpwindPaper (reference/debug): differences φ against the upwind (lower-φ)
  /// neighbor per axis — consistent with the discrete solution's
  /// characteristics and decisively picks a side at shocklines, but produces
  /// direction quantization and hard flips between adjacent cells straddling
  /// a shockline. Kept as a toggle; diffing the two fields on the same φ is
  /// the best available shockline/φ debugging tool (CCFieldVisualizer).
  /// </summary>
  public enum GradientScheme : byte
  {
    CentralRepo = 0,
    UpwindPaper = 1,
  }

  /// <summary>
  /// Global solve configuration singleton (baked by CCWorldAuthoring's
  /// GameObject via CCSolveSettingsAuthoring, or defaulted).
  /// </summary>
  public struct CCSolveSettings : IComponentData
  {
    /// <summary>Solve tick rate (spec §12: default 10 Hz; decoupled from frame rate).</summary>
    public float SolveHz;
    public GradientScheme Scheme;
    /// <summary>Largest unit radius (world units); sizes the min-distance hash buckets (spec §13.2).</summary>
    public float MaxUnitRadius;

    public static CCSolveSettings Defaults => new CCSolveSettings {
      SolveHz = 10f,
      Scheme = GradientScheme.CentralRepo,
      MaxUnitRadius = 0.5f,
    };
  }

  /// <summary>
  /// Raised by CCSolveTickSystem on frames where a solve tick fires; consumed
  /// by the stamping/field/eikonal/velocity systems.
  /// </summary>
  public struct CCSolveTick : IComponentData
  {
    public bool SolveThisFrame;
    public double LastTickTime;
  }
}
