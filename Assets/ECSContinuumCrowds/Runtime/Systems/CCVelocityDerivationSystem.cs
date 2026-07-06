using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Velocity derivation (spec §11, Decision D10): ∇φ per the configured
  /// gradient scheme (central-repo shipping / upwind-paper reference), then
  /// v = −f(direction faces) · ∇φ̂ into the group's velocity buffer. Chained
  /// after the eikonal jobs on solve ticks. Phase 3 turns Velocity0 into a
  /// double buffer with snapshots (D7).
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCEikonalSystem))]
  [BurstCompile]
  public partial struct CCVelocityDerivationSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveTick>();
      state.RequireForUpdate<CCSolveSettings>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      if (!SystemAPI.GetSingleton<CCSolveTick>().SolveThisFrame) {
        return;
      }

      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var settings = SystemAPI.GetSingleton<CCSolveSettings>();

      var handles = new NativeList<JobHandle>(8, Allocator.Temp);
      foreach (var buffers in
        SystemAPI.Query<RefRO<GroupFieldBuffers>>()
          .WithAll<CCGroup, CCGroupInitialized>()) {
        handles.Add(new VelocityJob {
          Gi = fields.Indexer,
          Scheme = settings.Scheme,
          Phi = buffers.ValueRO.Phi,
          F = buffers.ValueRO.F,
          Velocity = buffers.ValueRO.Velocity0,
        }.Schedule(fields.Rho.Length, 256, state.Dependency));
      }
      if (handles.Length > 0) {
        state.Dependency = JobHandle.CombineDependencies(handles.AsArray());
      }
    }

    [BurstCompile]
    private struct VelocityJob : IJobParallelFor
    {
      public GridIndexer Gi;
      public GradientScheme Scheme;
      [ReadOnly] public NativeArray<float> Phi;
      [ReadOnly] public NativeArray<float4> F;
      [WriteOnly] public NativeArray<float2> Velocity;

      public void Execute(int i)
      {
        var c = Gi.Coord(i);
        var dPhi = Scheme == GradientScheme.CentralRepo
          ? CCMath.PotentialGradientCentral(Phi, Gi, c)
          : CCMath.PotentialGradientUpwind(Phi, Gi, c);
        Velocity[i] = CCMath.VelocityFromGradient(dPhi, F[i]);
      }
    }
  }
}
