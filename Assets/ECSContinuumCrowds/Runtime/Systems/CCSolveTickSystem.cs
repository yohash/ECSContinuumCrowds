using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Phase-1 solve cadence (spec §12.1 subset): raises
  /// <see cref="CCSolveTick"/> at SolveHz (default 10) so the solve pipeline
  /// runs at a low rate decoupled from frame rate. The full staggering
  /// scheduler (slots, GroupsPerTick, buffer flipping) is Phase 3 (D7);
  /// Phase 1 solves every group on every tick within the frame's job chain.
  ///
  /// Also bootstraps default <see cref="CCSolveSettings"/> when no authored
  /// singleton exists.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  public partial struct CCSolveTickSystem : ISystem
  {
    public void OnUpdate(ref SystemState state)
    {
      if (!SystemAPI.HasSingleton<CCSolveSettings>()) {
        state.EntityManager.AddComponentData(
          state.EntityManager.CreateEntity(), CCSolveSettings.Defaults);
      }
      if (!SystemAPI.HasSingleton<CCSolveTick>()) {
        state.EntityManager.AddComponentData(
          state.EntityManager.CreateEntity(),
          new CCSolveTick { SolveThisFrame = false, LastTickTime = double.MinValue });
      }

      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      ref var tick = ref SystemAPI.GetSingletonRW<CCSolveTick>().ValueRW;

      double now = SystemAPI.Time.ElapsedTime;
      double period = 1.0 / math.max(settings.SolveHz, 0.01f);
      if (now - tick.LastTickTime >= period) {
        tick.SolveThisFrame = true;
        tick.LastTickTime = now;
      } else {
        tick.SolveThisFrame = false;
      }
    }
  }
}
