using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Gather-pattern stamping (Decision D3, spec §6): parallel job over the
  /// ACTIVE cells derived by CCSpatialHashSystem, each cell pulling
  /// static-splat + predictive-ghost contributions from units in its 9
  /// surrounding buckets. Runs once per solve tick for ALL groups scheduled
  /// that tick (spec §2.8).
  ///
  /// Chaining (Decision D7): the stamp chain is stored in CCStampState (not
  /// state.Dependency) and consumed by every scheduled group's field pass.
  /// Its input combines the hash tail with every group's ChainTail — an
  /// in-flight solve from an earlier tick may still be READING the global
  /// ρ/v̄ map, so the clear/gather must not start until those readers land
  /// (conservative write-after-read guard; runs on workers, never stalls
  /// the main thread).
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCSpatialHashSystem))]
  [BurstCompile]
  public partial struct CCStampingSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCStampingHash>();
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
      var hash = SystemAPI.GetSingleton<CCStampingHash>();
      ref var stamp = ref SystemAPI.GetSingletonRW<CCStampState>().ValueRW;

      // WAR guard: in-flight chains read the global map
      var dep = stamp.Handle;
      foreach (var solveState in SystemAPI.Query<RefRO<CCGroupSolveState>>().WithAll<CCGroup>()) {
        dep = JobHandle.CombineDependencies(dep, solveState.ValueRO.ChainTail);
      }

      var clear = new ClearFieldsJob {
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(fields.Rho.Length, 8192, dep);

      var gather = new GatherStampJob {
        Gi = fields.Indexer,
        Map = hash.Map,
        BucketCells = hash.BucketCells,
        BucketDims = hash.BucketDims,
        Constants = constants,
        CellSize = fields.CellSize,
        ActiveCells = hash.ActiveCells.AsDeferredJobArray(),
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(hash.ActiveCells, 64, clear);

      stamp.Handle = new FinalizeAverageVelocityJob {
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(fields.Rho.Length, 8192, gather);
    }

    [BurstCompile]
    private struct ClearFieldsJob : IJobParallelFor
    {
      [WriteOnly] public NativeArray<float> Rho;
      [WriteOnly] public NativeArray<float2> VAveAcc;

      public void Execute(int i)
      {
        Rho[i] = 0f;
        VAveAcc[i] = float2.zero;
      }
    }

    /// <summary>
    /// Spec §6.2 Job B: one item per ACTIVE cell (deferred — the list is
    /// produced by the hash system's job earlier in this same chain). All
    /// accumulation logic lives in the test-shared CCStampOps.GatherCell.
    /// </summary>
    [BurstCompile]
    private struct GatherStampJob : IJobParallelForDefer
    {
      public GridIndexer Gi;
      [ReadOnly] public NativeParallelMultiHashMap<int, UnitStampData> Map;
      public int BucketCells;
      public int2 BucketDims;
      public CCConstants Constants;
      public float CellSize;
      [ReadOnly] public NativeArray<int> ActiveCells;
      // each active cell index is unique, so per-cell writes are disjoint
      [NativeDisableParallelForRestriction] public NativeArray<float> Rho;
      [NativeDisableParallelForRestriction] public NativeArray<float2> VAveAcc;

      public void Execute(int index)
      {
        int flat = ActiveCells[index];
        CCStampOps.GatherCell(
          Gi.Coord(flat), Map, BucketCells, BucketDims, Constants, CellSize,
          out var rho, out var momentum);
        Rho[flat] = rho;
        VAveAcc[flat] = momentum;
      }
    }

    /// <summary>Spec §6.2 Job C: v̄ = acc/ρ; repo semantics divide only when ρ ≠ 0.</summary>
    [BurstCompile]
    private struct FinalizeAverageVelocityJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<float> Rho;
      public NativeArray<float2> VAveAcc;

      public void Execute(int i)
      {
        float r = Rho[i];
        if (r != 0f) {
          VAveAcc[i] /= r;
        }
      }
    }
  }
}
