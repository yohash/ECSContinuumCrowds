using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Density/velocity stamping, Phase-1 form (spec §15 P1): a NAIVE,
  /// single-threaded scatter over units — correctness-only, kept trivially
  /// auditable so the Phase-2 gather stamping (Decision D3: spatial hash →
  /// parallel gather over active cells) can be validated against it.
  ///
  /// ρ[c]    = Σᵢ wᵢ(c)·massᵢ
  /// v̄acc[c] = Σᵢ wᵢ(c)·massᵢ·velocityᵢ   → finalized to v̄ = acc/ρ
  ///
  /// ⚠ DIVERGENCE (repo extension, kept): the paper has no mass term; the
  /// repo scales density by unit mass — heterogeneous units for free.
  /// Predictive velocity (Decision D4) lands in Phase 2.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCGroupInitSystem))]
  [BurstCompile]
  public partial struct CCScatterStampingSystem : ISystem
  {
    private EntityQuery _unitQuery;

    public void OnCreate(ref SystemState state)
    {
      _unitQuery = SystemAPI.QueryBuilder()
        .WithAll<UnitTag, CCUnit, UnitVelocity, LocalTransform>().Build();
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
      int unitCount = _unitQuery.CalculateEntityCount();

      // frame-scoped scratch: the whole stamp chain completes this frame
      // (advection chains on it), so WorldUpdateAllocator is safe (§14.4)
      var stamps = new NativeArray<UnitStamp>(
        math.max(unitCount, 1), state.WorldUpdateAllocator);

      var collect = new CollectUnitStampsJob {
        Origin = fields.Origin,
        CellSize = fields.CellSize,
        Stamps = stamps,
      }.ScheduleParallel(_unitQuery, state.Dependency);

      var clear = new ClearFieldsJob {
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(fields.Rho.Length, 8192, state.Dependency);

      var scatter = new ScatterStampJob {
        Gi = fields.Indexer,
        Lambda = constants.lambda,
        Stamps = stamps,
        Count = unitCount,
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(JobHandle.CombineDependencies(collect, clear));

      state.Dependency = new FinalizeAverageVelocityJob {
        Rho = fields.Rho,
        VAveAcc = fields.VAveAcc,
      }.Schedule(fields.Rho.Length, 8192, scatter);
    }

    /// <summary>Snapshot payload; grid-space position, world-units/sec velocity.</summary>
    public struct UnitStamp
    {
      public float2 Position;
      public float2 Velocity;
      public float Mass;
    }

    [BurstCompile]
    private partial struct CollectUnitStampsJob : IJobEntity
    {
      public float2 Origin;
      public float CellSize;
      [NativeDisableParallelForRestriction] public NativeArray<UnitStamp> Stamps;

      private void Execute(
        [EntityIndexInQuery] int index,
        in LocalTransform transform,
        in CCUnit unit,
        in UnitVelocity velocity)
      {
        Stamps[index] = new UnitStamp {
          Position = CCMath.WorldToGrid(transform.Position, Origin, CellSize),
          Velocity = velocity.Value,
          Mass = unit.Mass,
        };
      }
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
    /// Single-threaded scatter (Phase 1 only): every unit deposits its 2×2
    /// splat (spec §6.4) into ρ and the momentum accumulator.
    /// </summary>
    [BurstCompile]
    private struct ScatterStampJob : IJob
    {
      public GridIndexer Gi;
      public float Lambda;
      [ReadOnly] public NativeArray<UnitStamp> Stamps;
      public int Count;
      public NativeArray<float> Rho;
      public NativeArray<float2> VAveAcc;

      public void Execute()
      {
        for (int u = 0; u < Count; u++) {
          var s = Stamps[u];
          var baseCell = CCMath.SplatBaseCell(s.Position);
          var w = CCMath.SplatWeights(s.Position, Lambda); // (A,B,C,D) = LL,LR,UR,UL
          Deposit(baseCell, w.x, s);
          Deposit(baseCell + new int2(1, 0), w.y, s);
          Deposit(baseCell + new int2(1, 1), w.z, s);
          Deposit(baseCell + new int2(0, 1), w.w, s);
        }
      }

      private void Deposit(int2 cell, float w, in UnitStamp s)
      {
        if (w <= 0f || !Gi.InBounds(cell)) {
          return;
        }
        int i = Gi.Flat(cell);
        float wm = w * s.Mass; // ⚠ DIVERGENCE (repo, kept): mass scaling
        Rho[i] += wm;
        VAveAcc[i] += wm * s.Velocity;
      }
    }

    /// <summary>v̄ = acc/ρ; repo computeAverageVelocityField divides only when ρ ≠ 0.</summary>
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
