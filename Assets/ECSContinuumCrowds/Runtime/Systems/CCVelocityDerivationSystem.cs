using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Velocity derivation + snapshot publish (spec §11 + §12.2, Decisions
  /// D10 + D7): ∇φ per the configured gradient scheme over the domain's
  /// neighbor table, v = −f(direction faces)·∇φ̂ into the group's BACK
  /// velocity buffer, and the back buffer's full-grid localIdxLookup rebuilt
  /// to snapshot the domain mapping the velocities were written under.
  /// The last system in the chain: disables the solve request and hands the
  /// group to the scheduler (Phase = Solving) for polling + the flip.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCEikonalSystem))]
  [BurstCompile]
  public partial struct CCVelocityDerivationSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveSettings>();
    }

    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      var em = state.EntityManager;

      foreach (var (group, domain, buffers, solveState, entity) in
        SystemAPI.Query<RefRW<CCGroup>, RefRO<DomainCache>, RefRO<GroupFieldBuffers>, RefRW<CCGroupSolveState>>()
          .WithAll<CCGroupInitialized, CCGroupSolveRequest>()
          .WithEntityAccess()) {
        int count = buffers.ValueRO.DomainLength;
        int back = 1 - group.ValueRO.ActiveBuffer;
        var backVelocity = back == 0 ? buffers.ValueRO.Velocity0 : buffers.ValueRO.Velocity1;
        var backLookup = back == 0 ? buffers.ValueRO.LocalIdxLookup0 : buffers.ValueRO.LocalIdxLookup1;

        var tail = solveState.ValueRO.ChainTail;
        var clearLookup = new ClearLookupJob {
          Lookup = backLookup,
        }.Schedule(backLookup.Length, 8192, tail);

        if (count > 0) {
          var velocity = new VelocityJob {
            Gi = fields.Indexer,
            Scheme = settings.Scheme,
            Cells = domain.ValueRO.Cells.AsArray(),
            Neighbors = domain.ValueRO.NeighborLocalIdx.AsArray(),
            Phi = buffers.ValueRO.Phi,
            F = buffers.ValueRO.F,
            Velocity = backVelocity,
          }.Schedule(count, 128, tail);
          var scatter = new ScatterLookupJob {
            Cells = domain.ValueRO.Cells.AsArray(),
            Lookup = backLookup,
          }.Schedule(count, 1024, clearLookup);
          tail = JobHandle.CombineDependencies(velocity, scatter);
        } else {
          tail = clearLookup; // empty domain publishes an empty snapshot
        }

        solveState.ValueRW.ChainTail = tail;
        group.ValueRW.Phase = SolvePhase.Solving;
        // mirror the live tail into the cleanup component so mid-solve
        // destruction can Complete() before disposing (spec §12.4)
        var cleanup = em.GetComponentData<GroupFieldBuffersCleanup>(entity);
        cleanup.ChainTail = tail;
        em.SetComponentData(entity, cleanup);
        em.SetComponentEnabled<CCGroupSolveRequest>(entity, false);
      }
    }

    [BurstCompile]
    private struct ClearLookupJob : IJobParallelFor
    {
      [WriteOnly] public NativeArray<int> Lookup;

      public void Execute(int i) => Lookup[i] = -1;
    }

    [BurstCompile]
    private struct VelocityJob : IJobParallelFor
    {
      public GridIndexer Gi;
      public GradientScheme Scheme;
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeArray<float> Phi;
      [ReadOnly] public NativeArray<float4> F;
      [NativeDisableParallelForRestriction] public NativeArray<float2> Velocity;

      public void Execute(int local)
      {
        var coord = Gi.Coord(Cells[local]);
        var dPhi = Scheme == GradientScheme.CentralRepo
          ? CCMath.PotentialGradientCentral(Phi, local, coord, Neighbors[local], Gi)
          : CCMath.PotentialGradientUpwind(Phi, local, Neighbors[local]);
        Velocity[local] = CCMath.VelocityFromGradient(dPhi, F[local]);
      }
    }

    /// <summary>Publish the snapshot mapping: lookup[globalFlat] = local (disjoint writes).</summary>
    [BurstCompile]
    private struct ScatterLookupJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<int> Cells;
      [NativeDisableParallelForRestriction] public NativeArray<int> Lookup;

      public void Execute(int local) => Lookup[Cells[local]] = local;
    }
  }
}
