using Unity.Entities;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Dedicated system group for all Continuum Crowds systems (spec §4).
  /// Target order within this group as later phases land:
  ///   Scheduler → SpatialHash → Stamping → Domain → Field → Eikonal →
  ///   VelocityDerivation → Advection → MinDistance.
  /// </summary>
  [UpdateInGroup(typeof(SimulationSystemGroup))]
  public partial class CCSimulationSystemGroup : ComponentSystemGroup
  {
  }
}
