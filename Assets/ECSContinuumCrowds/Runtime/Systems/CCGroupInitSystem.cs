using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Group lifecycle (spec §3.4): allocates each new group's domain cache +
  /// compact solve buffers (grown 1.5× by the scheduler as domains demand —
  /// spec §8.5), converts the authored world-space goal rect into GoalCell
  /// entries + the goal set/list, assigns the stagger slot (spec §12.3), and
  /// attaches the cleanup component so destruction cannot leak containers —
  /// including mid-solve (the cleanup carries the chain tail and completes
  /// it before disposal). Disposes buffers of destroyed groups.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateBefore(typeof(CCSchedulerSystem))]
  public partial struct CCGroupInitSystem : ISystem
  {
    private int _nextSlot;

    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
    }

    public void OnUpdate(ref SystemState state)
    {
      var em = state.EntityManager;

      // dispose buffers of destroyed groups (cleanup-component pattern);
      // Dispose() completes the mirrored chain tail first (spec §12.4)
      var orphanQuery = SystemAPI.QueryBuilder()
        .WithAll<GroupFieldBuffersCleanup>().WithNone<CCGroup>().Build();
      if (!orphanQuery.IsEmptyIgnoreFilter) {
        state.CompleteDependency();
        var orphans = orphanQuery.ToEntityArray(Allocator.Temp);
        foreach (var e in orphans) {
          em.GetComponentData<GroupFieldBuffersCleanup>(e).Dispose();
          em.RemoveComponent<GroupFieldBuffersCleanup>(e);
        }
      }

      var newQuery = SystemAPI.QueryBuilder()
        .WithAll<CCGroup, CCGroupGoalRect>().WithNone<CCGroupInitialized>().Build();
      if (newQuery.IsEmptyIgnoreFilter) {
        return;
      }

      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var gi = fields.Indexer;
      const int initialCapacity = 1024;

      var entities = newQuery.ToEntityArray(Allocator.Temp);
      foreach (var e in entities) {
        // authored world-space goal rect -> clamped goal cells
        var rect = em.GetComponentData<CCGroupGoalRect>(e);
        var lo = (int2)math.floor((rect.MinXZ - fields.Origin) / fields.CellSize);
        var hi = (int2)math.floor((rect.MaxXZ - fields.Origin) / fields.CellSize);
        lo = math.clamp(lo, int2.zero, new int2(gi.W - 1, gi.H - 1));
        hi = math.clamp(hi, int2.zero, new int2(gi.W - 1, gi.H - 1));

        var goals = em.HasBuffer<GoalCell>(e) ? em.GetBuffer<GoalCell>(e) : em.AddBuffer<GoalCell>(e);
        goals.Clear();
        for (int y = lo.y; y <= hi.y; y++) {
          for (int x = lo.x; x <= hi.x; x++) {
            goals.Add(new GoalCell { Cell = new int2(x, y) });
          }
        }
        int goalCount = goals.Length;

        var domain = new DomainCache {
          Cells = new NativeList<int>(initialCapacity, Allocator.Persistent),
          // full-grid capacity so the fill job never needs a resize (spec §8.5)
          GlobalToLocal = new NativeParallelHashMap<int, int>(gi.CellCount, Allocator.Persistent),
          NeighborLocalIdx = new NativeList<int4>(initialCapacity, Allocator.Persistent),
          GoalSet = new NativeParallelHashMap<int, byte>(
            math.max(64, goalCount * 2), Allocator.Persistent),
          GoalCellList = new NativeList<int2>(math.max(64, goalCount), Allocator.Persistent),
          DirtyFlags = new NativeArray<byte>(2, Allocator.Persistent),
          Valid = false,
        };
        goals = em.GetBuffer<GoalCell>(e); // re-fetch (container ops above are safe, but be explicit)
        for (int i = 0; i < goals.Length; i++) {
          domain.GoalSet.TryAdd(gi.Flat(goals[i].Cell), 1);
          domain.GoalCellList.Add(goals[i].Cell);
        }

        var buffers = new GroupFieldBuffers {
          F = new NativeArray<float4>(initialCapacity, Allocator.Persistent),
          C = new NativeArray<float4>(initialCapacity, Allocator.Persistent),
          Phi = new NativeArray<float>(initialCapacity, Allocator.Persistent),
          CellState = new NativeArray<byte>(initialCapacity, Allocator.Persistent),
          HeapCells = new NativeArray<int>(initialCapacity, Allocator.Persistent),
          HeapKeys = new NativeArray<float>(initialCapacity, Allocator.Persistent),
          HeapPos = new NativeArray<int>(initialCapacity, Allocator.Persistent),
          FimActiveClean = new NativeList<int>(initialCapacity, Allocator.Persistent),
          FimActiveRaw = new NativeList<int>(initialCapacity * 5, Allocator.Persistent),
          FimStatus = new NativeArray<int>(4, Allocator.Persistent),
          Velocity0 = new NativeArray<float2>(initialCapacity, Allocator.Persistent),
          Velocity1 = new NativeArray<float2>(initialCapacity, Allocator.Persistent),
          LocalIdxLookup0 = new NativeArray<int>(gi.CellCount, Allocator.Persistent),
          LocalIdxLookup1 = new NativeArray<int>(gi.CellCount, Allocator.Persistent),
        };
        // both snapshots start empty: advection samples zero velocity until
        // the first solve flips a real snapshot in
        for (int i = 0; i < gi.CellCount; i++) {
          buffers.LocalIdxLookup0[i] = -1;
          buffers.LocalIdxLookup1[i] = -1;
        }

        var group = em.GetComponentData<CCGroup>(e);
        group.Phase = SolvePhase.Idle;
        group.ScheduleSlot = _nextSlot++;
        group.ActiveBuffer = 0;
        em.SetComponentData(e, group);

        em.AddComponentData(e, domain);
        em.AddComponentData(e, buffers);
        em.AddComponentData(e, new CCGroupSolveState());
        em.AddComponent<CCGroupSolveRequest>(e);
        em.SetComponentEnabled<CCGroupSolveRequest>(e, false);
        em.AddComponentData(e, new GroupFieldBuffersCleanup {
          Buffers = buffers,
          Domain = domain,
        });
        em.AddComponent<CCGroupInitialized>(e);
      }
    }

    public void OnDestroy(ref SystemState state)
    {
      // world teardown: dispose every group's containers
      var query = SystemAPI.QueryBuilder().WithAll<GroupFieldBuffersCleanup>().Build();
      var cleanups = query.ToComponentDataArray<GroupFieldBuffersCleanup>(Allocator.Temp);
      foreach (var c in cleanups) {
        c.Dispose();
      }
    }
  }
}
