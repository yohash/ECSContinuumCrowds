using Unity.Entities;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Optional authoring for <see cref="CCSolveSettings"/>; when absent,
  /// CCSolveTickSystem bootstraps defaults (10 Hz, central-repo gradient).
  /// </summary>
  public class CCSolveSettingsAuthoring : MonoBehaviour
  {
    [Tooltip("Solve tick rate, decoupled from frame rate (spec §12: default 10 Hz).")]
    [Min(0.1f)] public float solveHz = 10f;
    [Tooltip("Velocity gradient scheme (D10): CentralRepo ships; UpwindPaper is the reference/shockline-debug path.")]
    public GradientScheme gradientScheme = GradientScheme.CentralRepo;
    [Tooltip("Largest unit radius in the world; sizes min-distance hash buckets.")]
    [Min(0.01f)] public float maxUnitRadius = 0.5f;

    private class Baker : Baker<CCSolveSettingsAuthoring>
    {
      public override void Bake(CCSolveSettingsAuthoring authoring)
      {
        var entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, new CCSolveSettings {
          SolveHz = authoring.solveHz,
          Scheme = authoring.gradientScheme,
          MaxUnitRadius = authoring.maxUnitRadius,
        });
      }
    }
  }
}
