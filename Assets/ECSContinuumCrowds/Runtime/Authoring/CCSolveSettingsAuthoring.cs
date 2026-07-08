using Unity.Entities;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Optional authoring for <see cref="CCSolveSettings"/>; when absent,
  /// CCSchedulerSystem bootstraps defaults (10 Hz, central-repo gradient).
  /// </summary>
  public class CCSolveSettingsAuthoring : MonoBehaviour
  {
    [Tooltip("Solve tick rate, decoupled from frame rate (spec §12: default 10 Hz).")]
    [Min(0.1f)] public float solveHz = 10f;
    [Tooltip("Velocity gradient scheme (D10): CentralRepo ships; UpwindPaper is the reference/shockline-debug path.")]
    public GradientScheme gradientScheme = GradientScheme.CentralRepo;
    [Tooltip("Largest unit radius in the world; sizes min-distance hash buckets.")]
    [Min(0.01f)] public float maxUnitRadius = 0.5f;
    [Tooltip("Groups whose pipelines may start per solve tick (spec §12.3). Per-group refresh rate = SolveHz / ceil(groups/GroupsPerTick).")]
    [Min(1)] public int groupsPerTick = 1;
    [Tooltip("Domain hysteresis pad in cells (spec §8.4).")]
    [Min(1f)] public float padCells = 16f;
    [Tooltip("BFS graph-distance cap; 0 = W+H (effectively uncapped).")]
    [Min(0)] public int horizonCells = 0;
    [Tooltip("Sampled speed ≈ 0 for longer than this fires a doubled-pad domain refresh (spec §8.6).")]
    [Min(0.1f)] public float stallSeconds = 1.5f;

    [Header("Hybrid eikonal (D8) — placeholder threshold, replace via crossover benchmark")]
    [Tooltip("Domain size at/above which FIM is preferred; ≤0 = FMM always (spec §10.3 placeholder 32768).")]
    public int fimThresholdCells = 32768;
    [Tooltip("Minimum expected idle workers for FIM to pay off (spec §10.3).")]
    [Min(0)] public int fimMinIdleWorkers = 2;
    [Tooltip("FIM convergence epsilon (spec §10.2).")]
    [Min(1e-6f)] public float fimEps = 1e-3f;
    [Tooltip("Parallel sweep pairs pre-scheduled per FIM chain (extras are no-ops).")]
    [Min(1)] public int fimParallelSweeps = 48;
    [Tooltip("Hard sweep cap — hit-cap telemetry signals the §10.4 fallback.")]
    [Min(8)] public int fimMaxSweeps = 4096;
    [Tooltip("§10.4: WeightedBlend ships; MaxRootWithBlendedPostPass is the documented fallback.")]
    public FimRootMode fimRootMode = FimRootMode.WeightedBlend;

    private class Baker : Baker<CCSolveSettingsAuthoring>
    {
      public override void Bake(CCSolveSettingsAuthoring authoring)
      {
        var entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, new CCSolveSettings {
          SolveHz = authoring.solveHz,
          Scheme = authoring.gradientScheme,
          MaxUnitRadius = authoring.maxUnitRadius,
          GroupsPerTick = authoring.groupsPerTick,
          PadCells = authoring.padCells,
          HorizonCells = authoring.horizonCells,
          StallSeconds = authoring.stallSeconds,
          FimThresholdCells = authoring.fimThresholdCells,
          FimMinIdleWorkers = authoring.fimMinIdleWorkers,
          FimEps = authoring.fimEps,
          FimParallelSweeps = authoring.fimParallelSweeps,
          FimMaxSweeps = authoring.fimMaxSweeps,
          FimRootMode = authoring.fimRootMode,
        });
      }
    }
  }
}
