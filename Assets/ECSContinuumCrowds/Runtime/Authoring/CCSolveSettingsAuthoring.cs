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
        });
      }
    }
  }
}
