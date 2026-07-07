using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Domain derivation (spec §8, Decision D5): for each group the scheduler
  /// flagged this frame, launches the flood-fill as a single Burst IJob (BFS
  /// over walkability is memory-bound and fast; parallelizing BFS is not
  /// worth its complexity — spec §8.2). The job runs off the critical path;
  /// the scheduler polls its handle and promotes the group to
  /// SolveRequested when it lands. Full re-derivation every refresh
  /// (spec §8.4: incremental frontier expansion adds bookkeeping for
  /// negligible savings at baseline).
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCStampingSystem))]
  [BurstCompile]
  public partial struct CCDomainSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveSettings>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      int horizon = settings.HorizonCells > 0
        ? settings.HorizonCells
        : fields.W + fields.H; // spec §8.2 default: effectively uncapped

      var em = state.EntityManager;
      foreach (var (group, domain, solveState, entity) in
        SystemAPI.Query<RefRW<CCGroup>, RefRO<DomainCache>, RefRW<CCGroupSolveState>>()
          .WithAll<CCGroupInitialized>()
          .WithEntityAccess()) {
        if (group.ValueRO.Phase != SolvePhase.DomainRefreshRequested) {
          continue;
        }

        var d = domain.ValueRO;
        // chain after the group's previous (completed) chain; the fill only
        // touches this group's own containers + the read-only walkable mask
        var tail = new FloodFillJob {
          Gi = fields.Indexer,
          Walkable = fields.Walkable,
          GoalCells = d.GoalCellList,
          PaddedMin = d.PendingPaddedMin,
          PaddedMax = d.PendingPaddedMax,
          HorizonCells = horizon,
          Cells = d.Cells,
          GlobalToLocal = d.GlobalToLocal,
          NeighborLocalIdx = d.NeighborLocalIdx,
        }.Schedule(solveState.ValueRO.ChainTail);
        solveState.ValueRW.ChainTail = tail;
        group.ValueRW.Phase = SolvePhase.DomainRefreshing;
        // mirror into cleanup: mid-refresh destruction completes then disposes
        var cleanup = em.GetComponentData<GroupFieldBuffersCleanup>(entity);
        cleanup.ChainTail = tail;
        em.SetComponentData(entity, cleanup);
      }
    }

    /// <summary>Thin wrapper: all fill logic lives in the test-shared CCDomainOps.</summary>
    [BurstCompile]
    private struct FloodFillJob : Unity.Jobs.IJob
    {
      public GridIndexer Gi;
      [ReadOnly] public NativeArray<byte> Walkable;
      [ReadOnly] public NativeList<int2> GoalCells;
      public int2 PaddedMin;
      public int2 PaddedMax;
      public int HorizonCells;
      public NativeList<int> Cells;
      public NativeParallelHashMap<int, int> GlobalToLocal;
      public NativeList<int4> NeighborLocalIdx;

      public void Execute()
      {
        CCDomainOps.FloodFill(
          Gi, Walkable, GoalCells.AsArray(), PaddedMin, PaddedMax, HorizonCells,
          Cells, GlobalToLocal, NeighborLocalIdx);
      }
    }
  }
}
