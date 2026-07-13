using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Group lifecycle (spec §3.4): for each new group, allocates the
  /// persistent per-group solve buffers (Phase 1: full-grid sized — compact
  /// flood-fill domains arrive in Phase 3), converts the authored world-space
  /// goal rect into <see cref="GoalCell"/> entries, and attaches the cleanup
  /// component so destruction cannot leak native containers. Disposes
  /// buffers of destroyed groups.
  ///
  /// Structural changes happen here on the main thread at init/destroy only
  /// — never on the per-frame hot path.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCSolveTickSystem))]
  public partial struct CCGroupInitSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
    }

    public void OnUpdate(ref SystemState state)
    {
      var em = state.EntityManager;

      // dispose buffers of destroyed groups (cleanup-component pattern).
      // Phase-3 note: once solves span frames, disposal must wait on the
      // group's in-flight JobHandle (Dispose(JobHandle)); in Phase 1 the
      // chain completes within the frame, so direct disposal is safe.
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

      // initialize new groups
      var newQuery = SystemAPI.QueryBuilder()
        .WithAll<CCGroup, CCGroupGoalRect>().WithNone<CCGroupInitialized>().Build();
      if (newQuery.IsEmptyIgnoreFilter) {
        return;
      }

      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var gi = fields.Indexer;
      int cells = gi.CellCount;

      var entities = newQuery.ToEntityArray(Allocator.Temp);
      foreach (var e in entities) {
        var buffers = new GroupFieldBuffers {
          F = new NativeArray<float4>(cells, Allocator.Persistent),
          C = new NativeArray<float4>(cells, Allocator.Persistent),
          Phi = new NativeArray<float>(cells, Allocator.Persistent),
          Velocity0 = new NativeArray<float2>(cells, Allocator.Persistent),
          CellState = new NativeArray<byte>(cells, Allocator.Persistent),
          HeapCells = new NativeArray<int>(cells, Allocator.Persistent),
          HeapKeys = new NativeArray<float>(cells, Allocator.Persistent),
          HeapPos = new NativeArray<int>(cells, Allocator.Persistent),
        };
        em.AddComponentData(e, buffers);
        em.AddComponentData(e, new GroupFieldBuffersCleanup {
          F = buffers.F,
          C = buffers.C,
          Phi = buffers.Phi,
          Velocity0 = buffers.Velocity0,
          CellState = buffers.CellState,
          HeapCells = buffers.HeapCells,
          HeapKeys = buffers.HeapKeys,
          HeapPos = buffers.HeapPos,
        });

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

        em.AddComponent<CCGroupInitialized>(e);
      }
    }

    public void OnDestroy(ref SystemState state)
    {
      // world teardown: dispose every group's buffers
      var query = SystemAPI.QueryBuilder().WithAll<GroupFieldBuffersCleanup>().Build();
      var cleanups = query.ToComponentDataArray<GroupFieldBuffersCleanup>(Allocator.Temp);
      foreach (var c in cleanups) {
        c.Dispose();
      }
    }
  }
}
