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
    /// <summary>Groups whose pipelines may start per solve tick (spec §12.3,
    /// default 1). SlotCount = ceil(groups/GroupsPerTick); per-group refresh
    /// rate = SolveHz / SlotCount — with 5 groups at 10 Hz and 1 group/tick,
    /// each group refreshes at 2 Hz. Raise SolveHz if that's too stale.</summary>
    public int GroupsPerTick;
    /// <summary>Domain hysteresis pad in cells (spec §8.2/§8.4, default 16).</summary>
    public float PadCells;
    /// <summary>BFS graph-distance cap (spec §8.2); ≤ 0 → W + H (effectively
    /// uncapped at baseline; exists so giant worlds can't explode a fill).</summary>
    public int HorizonCells;
    /// <summary>Sampled speed ≈ 0 for longer than this (seconds) fires a
    /// domain refresh with doubled pad (spec §8.6, default 1.5).</summary>
    public float StallSeconds;

    // --- hybrid eikonal decider (Decision D8, spec §10.3) ---
    /// <summary>Domain size at/above which FIM is preferred (given idle
    /// workers). PLACEHOLDER 32,768 (≈180², ~12.5% of 512²) to be replaced
    /// by the crossover benchmark's measured value (spec §10.3); ≤ 0
    /// disables FIM entirely (FMM always).</summary>
    public int FimThresholdCells;
    /// <summary>Minimum expected idle workers for FIM to be worth its
    /// ~1.5–2.5× work overhead (spec §10.3; placeholder 2).</summary>
    public int FimMinIdleWorkers;
    /// <summary>FIM convergence epsilon (spec §10.2: ~1e-3, cost units).</summary>
    public float FimEps;
    /// <summary>Parallel relax/compact sweep pairs pre-scheduled per solve
    /// chain; sweeps past convergence are no-ops over empty deferred lists.
    /// The serial finisher covers anything beyond.</summary>
    public int FimParallelSweeps;
    /// <summary>Hard sweep cap (termination guarantee; hit-cap telemetry
    /// signals the spec §10.4 fallback should ship).</summary>
    public int FimMaxSweeps;
    /// <summary>Iteration root handling (spec §10.4; WeightedBlend ships).</summary>
    public FimRootMode FimRootMode;

    public static CCSolveSettings Defaults => new CCSolveSettings {
      SolveHz = 10f,
      Scheme = GradientScheme.CentralRepo,
      MaxUnitRadius = 0.5f,
      GroupsPerTick = 1,
      PadCells = 16f,
      HorizonCells = 0,
      StallSeconds = 1.5f,
      FimThresholdCells = 32768,
      FimMinIdleWorkers = 2,
      FimEps = 1e-3f,
      FimParallelSweeps = 48,
      FimMaxSweeps = 4096,
      FimRootMode = FimRootMode.WeightedBlend,
    };
  }

  /// <summary>
  /// Tail handle of the shared hash→stamp chain for the current tick,
  /// consumed as the input dependency of every group's field pass. Written
  /// by CCStampingSystem; the stamped global map is shared by all groups
  /// scheduled on the same tick (spec §2.8 note).
  /// </summary>
  public struct CCStampState : IComponentData
  {
    public Unity.Jobs.JobHandle Handle;
  }

  /// <summary>
  /// Scheduler telemetry (Phase-3 validation: cache hit-rate ≫ 90% in steady
  /// state; stall/escape triggers observable). Read it in the inspector or
  /// from tests; counters are cumulative.
  /// </summary>
  public struct CCSolveTelemetry : IComponentData
  {
    public int SolvesStarted;
    public int SolvesCompleted;
    public int DomainRefreshes;
    public int CacheHits;          // tick solves that reused a valid domain
    public int EscapeRefreshes;
    public int StallRefreshes;     // spec §8.6: log these; they should be rare
    public int FmmSolves;          // hybrid decider observability (spec §10.3)
    public int FimSolves;
    public int FimHitCaps;         // >0 with WeightedBlend → ship the §10.4 fallback
  }

  /// <summary>
  /// Raised by CCSchedulerSystem on frames where ≥ 1 group starts a solve
  /// pipeline; consumed by the spatial-hash/stamping systems (the shared
  /// stamp runs once for all groups scheduled that tick).
  /// </summary>
  public struct CCSolveTick : IComponentData
  {
    public bool SolveThisFrame;
    public double LastTickTime;
    public long TickIndex;
  }
}
