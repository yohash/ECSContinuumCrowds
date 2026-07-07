using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Gather-pattern stamping (Decision D3, spec §6 — replaces the Phase-1
  /// naive scatter, whose semantics live on as the test oracle in
  /// CCStampOps.Scatter*): a parallel job over the ACTIVE cells derived by
  /// CCSpatialHashSystem, each cell pulling static-splat + predictive-ghost
  /// contributions from units in its 9 surrounding buckets. Every cell is
  /// written by exactly one thread — no write races, no atomics, no
  /// reduction pass.
  ///
  ///   ρ[c]    = Σᵢ (wᵢ(c) + wpᵢ(c)) · massᵢ
  ///   v̄acc[c] = Σᵢ (wᵢ(c) + wpᵢ(c)) · massᵢ · velocityᵢ → v̄ = acc/ρ
  ///
  /// ⚠ DIVERGENCE (repo extension, kept): mass scaling (the paper has none).
  ///
  /// Field clear: the full grid is cleared each stamp tick (~3 MB of writes
  /// at 512²) — spec §6.2 explicitly prefers this simple form first; the
  /// tracked previous-active-set clear is a measured Phase-4 option.
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

      var clear = new ClearFieldsJob {
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(fields.Rho.Length, 8192, state.Dependency);

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

      state.Dependency = new FinalizeAverageVelocityJob {
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
