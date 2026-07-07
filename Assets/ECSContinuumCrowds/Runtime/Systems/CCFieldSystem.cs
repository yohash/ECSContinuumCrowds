using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Speed/cost field pass (spec §2.4–2.5) over each requested group's
  /// compact domain (spec §8.3): reads the freshly stamped global ρ/v̄/g/∇h
  /// with the into-cell rule via the domain's neighbor table (zero hashing),
  /// writes domain-compact F/C.
  ///
  /// Multi-frame chaining (Decision D7): jobs are appended to the group's
  /// ChainTail with the shared stamp handle as input — NEVER to
  /// state.Dependency — so in-flight solves cannot serialize against the
  /// per-frame advection/min-distance passes. The scheduler polls the tail.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCDomainSystem))]
  [BurstCompile]
  public partial struct CCFieldSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCConstants>();
      state.RequireForUpdate<CCStampState>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var constants = SystemAPI.GetSingleton<CCConstants>();
      var stamp = SystemAPI.GetSingleton<CCStampState>();

      foreach (var (group, domain, buffers, solveState) in
        SystemAPI.Query<RefRO<CCGroup>, RefRO<DomainCache>, RefRW<GroupFieldBuffers>, RefRW<CCGroupSolveState>>()
          .WithAll<CCGroupInitialized, CCGroupSolveRequest>()) {
        int count = domain.ValueRO.Cells.Length;
        buffers.ValueRW.DomainLength = count;
        if (count == 0) {
          continue; // empty domain (e.g. goal on unwalkable ground) — velocity pass clears the lookup
        }

        solveState.ValueRW.ChainTail = new FieldJob {
          Cells = domain.ValueRO.Cells.AsArray(),
          Neighbors = domain.ValueRO.NeighborLocalIdx.AsArray(),
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
        }.Schedule(count, 128,
          JobHandle.CombineDependencies(stamp.Handle, solveState.ValueRO.ChainTail));
      }
    }

    [BurstCompile]
    private struct FieldJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeArray<float> Rho;
      [ReadOnly] public NativeArray<float2> VAve;
      [ReadOnly] public NativeArray<float> Discomfort;
      [ReadOnly] public NativeArray<float2> DH;
      public CCConstants Constants;
      public float Alpha;
      public float Beta;
      public float Gamma;
      [NativeDisableParallelForRestriction] public NativeArray<float4> F;
      [NativeDisableParallelForRestriction] public NativeArray<float4> C;

      public void Execute(int local)
      {
        CCFieldOps.ComputeCell(
          local, Cells, Neighbors, Rho, VAve, Discomfort, DH,
          Constants, Alpha, Beta, Gamma, out var f, out var c);
        F[local] = f;
        C[local] = c;
      }
    }
  }
}
