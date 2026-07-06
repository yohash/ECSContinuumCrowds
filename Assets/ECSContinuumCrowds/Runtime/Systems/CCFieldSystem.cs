using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Speed/cost field pass (spec §2.4–2.5): on solve ticks, computes each
  /// group's anisotropic f and C float4 fields over the (Phase-1 full-grid)
  /// domain, reading the freshly stamped global ρ/v̄/g/∇h with the into-cell
  /// rule. Group jobs are independent (read-only shared inputs, private
  /// outputs) and run concurrently.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCScatterStampingSystem))]
  [BurstCompile]
  public partial struct CCFieldSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveTick>();
      state.RequireForUpdate<CCConstants>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      if (!SystemAPI.GetSingleton<CCSolveTick>().SolveThisFrame) {
        return;
      }

      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var constants = SystemAPI.GetSingleton<CCConstants>();

      var handles = new NativeList<JobHandle>(8, Allocator.Temp);
      foreach (var (group, buffers) in
        SystemAPI.Query<RefRO<CCGroup>, RefRO<GroupFieldBuffers>>()
          .WithAll<CCGroupInitialized>()) {
        handles.Add(new FieldJob {
          Gi = fields.Indexer,
          Rho = fields.Rho,
          VAve = fields.VAveAcc, // holds v̄ after the stamping finalize pass
          Discomfort = fields.Discomfort,
          DH = fields.DH,
          Constants = constants,
          Alpha = group.ValueRO.Alpha,
          Beta = group.ValueRO.Beta,
          Gamma = group.ValueRO.Gamma,
          F = buffers.ValueRO.F,
          C = buffers.ValueRO.C,
        }.Schedule(fields.Rho.Length, 256, state.Dependency));
      }
      if (handles.Length > 0) {
        state.Dependency = JobHandle.CombineDependencies(handles.AsArray());
      }
    }

    [BurstCompile]
    private struct FieldJob : IJobParallelFor
    {
      public GridIndexer Gi;
      [ReadOnly] public NativeArray<float> Rho;
      [ReadOnly] public NativeArray<float2> VAve;
      [ReadOnly] public NativeArray<float> Discomfort;
      [ReadOnly] public NativeArray<float2> DH;
      public CCConstants Constants;
      public float Alpha;
      public float Beta;
      public float Gamma;
      [WriteOnly] public NativeArray<float4> F;
      [WriteOnly] public NativeArray<float4> C;

      public void Execute(int i)
      {
        CCFieldOps.ComputeCell(
          i, Gi, Rho, VAve, Discomfort, DH, Constants, Alpha, Beta, Gamma,
          out var f, out var c);
        F[i] = f;
        C[i] = c;
      }
    }
  }
}
