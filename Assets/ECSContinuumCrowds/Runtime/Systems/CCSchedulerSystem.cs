using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// The solve scheduler (spec §12, Decision D7) — replaces the Phase-1
  /// CCSolveTickSystem. Every frame it:
  ///
  /// 1. POLLS each group's in-flight chain (JobHandle.IsCompleted — never a
  ///    blocking wait): a finished SOLVE gets Complete() (free), the buffer
  ///    FLIP (ActiveBuffer swap; race-free because advection only reads the
  ///    front pair and the solve only wrote the back pair), and
  ///    LastSolveTime; a finished DOMAIN REFRESH gets its compact buffers
  ///    grown (1.5×, spec §8.5) and is promoted straight to SolveRequested
  ///    (the fields→eikonal→velocity chain is assembled by the downstream
  ///    systems THIS frame, against a fresh stamp).
  ///
  /// 2. On solve TICKS (SolveHz), stagger-selects groups
  ///    (slot == tick % SlotCount, spec §12.3) that are Idle, evaluates the
  ///    domain-cache triggers (spec §8.4: goal changed / centroid drift >
  ///    pad/2 / radius growth / unit escaped / walkability version / stall),
  ///    and either requests the solve directly (cache hit — expected ≫ 90%)
  ///    or hands a padded-AABB refresh to CCDomainSystem (stall → doubled
  ///    pad, spec §8.6).
  ///
  /// 3. Raises CCSolveTick.SolveThisFrame whenever ≥ 1 group starts a solve,
  ///    which drives the shared hash→stamp pass (once per tick for ALL
  ///    groups scheduled that tick — spec §2.8).
  ///
  /// Accepted relaxations (spec §12.5 — document, don't "fix"): groups on
  /// different stagger slots solve against different stamp snapshots; a
  /// regrouped unit samples its new group's possibly-stale field for ≤ one
  /// refresh interval; advected positions lag stamped density by ≤ one
  /// interval. This is exactly the paper's low-rate regime.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCUnitSpawnSystem))]
  public partial struct CCSchedulerSystem : ISystem
  {
    private EntityQuery _groupQuery;
    private EntityQuery _unitQuery;

    public void OnCreate(ref SystemState state)
    {
      _groupQuery = SystemAPI.QueryBuilder()
        .WithAll<CCGroup, DomainCache, GroupFieldBuffers, CCGroupSolveState, CCGroupInitialized>()
        .Build();
      _unitQuery = SystemAPI.QueryBuilder()
        .WithAll<UnitTag, CCUnit, LocalTransform>().Build();
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCConstants>();
    }

    public void OnUpdate(ref SystemState state)
    {
      var em = state.EntityManager;

      // bootstrap singletons
      if (!SystemAPI.HasSingleton<CCSolveSettings>()) {
        em.AddComponentData(em.CreateEntity(), CCSolveSettings.Defaults);
      }
      if (!SystemAPI.HasSingleton<CCSolveTick>()) {
        var e = em.CreateEntity();
        em.AddComponentData(e, new CCSolveTick { LastTickTime = double.MinValue });
        em.AddComponentData(e, new CCStampState());
        em.AddComponentData(e, new CCSolveTelemetry());
      }

      // main-thread reads below touch containers written by last frame's
      // advection (DirtyFlags) — complete those (they finished long ago;
      // Complete() on done handles is free)
      em.CompleteDependencyBeforeRW<DomainCache>();

      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      ref var telemetry = ref SystemAPI.GetSingletonRW<CCSolveTelemetry>().ValueRW;
      ref var tick = ref SystemAPI.GetSingletonRW<CCSolveTick>().ValueRW;
      double now = SystemAPI.Time.ElapsedTime;
      bool anySolveStarting = false;

      // ------------------------------------------------------------
      // 1. poll in-flight chains
      // ------------------------------------------------------------
      var entities = _groupQuery.ToEntityArray(Allocator.Temp);
      foreach (var e in entities) {
        var group = em.GetComponentData<CCGroup>(e);
        var solveState = em.GetComponentData<CCGroupSolveState>(e);

        if (group.Phase == SolvePhase.Solving && solveState.ChainTail.IsCompleted) {
          solveState.ChainTail.Complete();
          group.ActiveBuffer = 1 - group.ActiveBuffer; // the flip (spec §12.2)
          group.LastSolveTime = now;
          group.Phase = SolvePhase.Idle;
          telemetry.SolvesCompleted++;
          em.SetComponentData(e, group);
          em.SetComponentData(e, solveState);
        } else if (group.Phase == SolvePhase.DomainRefreshing && solveState.ChainTail.IsCompleted) {
          solveState.ChainTail.Complete();
          var domain = em.GetComponentData<DomainCache>(e);
          var buffers = em.GetComponentData<GroupFieldBuffers>(e);
          EnsureScratchCapacity(em, e, ref buffers, domain.Cells.Length);
          domain.Valid = true;
          domain.WalkabilityVersion = SystemAPI.GetSingleton<CCWalkabilityVersion>().Version;
          em.SetComponentData(e, domain);
          telemetry.DomainRefreshes++;
          // fresh domain solves immediately, against a fresh stamp
          group.Phase = SolvePhase.SolveRequested;
          em.SetComponentData(e, group);
          em.SetComponentData(e, solveState);
          em.SetComponentEnabled<CCGroupSolveRequest>(e, true);
          anySolveStarting = true;
        }
      }

      // ------------------------------------------------------------
      // 2. solve tick + stagger selection + trigger evaluation
      // ------------------------------------------------------------
      double period = 1.0 / math.max(settings.SolveHz, 0.01f);
      bool tickFired = now - tick.LastTickTime >= period;
      if (tickFired && entities.Length > 0) {
        tick.LastTickTime = now;
        tick.TickIndex++;

        int groupsPerTick = math.max(settings.GroupsPerTick, 1);
        int slotCount = math.max(1, (entities.Length + groupsPerTick - 1) / groupsPerTick);
        int activeSlot = (int)(tick.TickIndex % slotCount);

        // per-group unit extents for trigger tests + fill bounds (one cheap
        // Burst pass over the unit snapshot; ~10k units ≈ microseconds)
        var extents = ComputeGroupExtents(ref state, fields);

        foreach (var e in entities) {
          var group = em.GetComponentData<CCGroup>(e);
          if (group.Phase != SolvePhase.Idle || group.ScheduleSlot % slotCount != activeSlot) {
            continue;
          }
          var domain = em.GetComponentData<DomainCache>(e);
          var goals = em.GetBuffer<GoalCell>(e);

          // goal centroid/count (goal buffers are small)
          float2 goalCentroid = float2.zero;
          var goalMin = new int2(int.MaxValue);
          var goalMax = new int2(int.MinValue);
          for (int i = 0; i < goals.Length; i++) {
            goalCentroid += goals[i].Cell;
            goalMin = math.min(goalMin, goals[i].Cell);
            goalMax = math.max(goalMax, goals[i].Cell);
          }
          if (goals.Length > 0) {
            goalCentroid /= goals.Length;
          } else {
            // degenerate group: no goal cells — anchor bounds on the units
            goalMin = (int2)math.floor(goalCentroid);
            goalMax = goalMin;
          }

          bool hasUnits = extents.TryGetValue(group.GroupId, out var extent);
          float2 centroid = hasUnits ? extent.Sum / extent.Count : goalCentroid;
          float2 unitMin = hasUnits ? extent.Min : goalCentroid;
          float2 unitMax = hasUnits ? extent.Max : goalCentroid;
          float radius = hasUnits
            ? math.max(math.distance(centroid, unitMin), math.distance(centroid, unitMax))
            : 0f;

          var reason = CCDomainOps.EvaluateTriggers(
            domain.Valid,
            domain.CachedGoalCentroid, domain.CachedGoalCount,
            goalCentroid, goals.Length,
            domain.CachedGroupCentroid, domain.CachedGroupRadius,
            centroid, radius,
            domain.WalkabilityVersion,
            SystemAPI.GetSingleton<CCWalkabilityVersion>().Version,
            domain.DirtyFlags[0] != 0,
            domain.DirtyFlags[1] != 0,
            domain.PadCellsUsed > 0f ? domain.PadCellsUsed : settings.PadCells);

          if (reason == DomainRefreshReason.None) {
            // cache hit — solve over the cached domain
            group.Phase = SolvePhase.SolveRequested;
            em.SetComponentData(e, group);
            em.SetComponentEnabled<CCGroupSolveRequest>(e, true);
            telemetry.CacheHits++;
            telemetry.SolvesStarted++;
            anySolveStarting = true;
            continue;
          }

          // refresh required: compute fill bounds; stall doubles the pad
          // (spec §8.6 — log it, it should be rare)
          float pad = settings.PadCells;
          if (reason == DomainRefreshReason.UnitStalled) {
            pad = math.max(domain.PadCellsUsed, settings.PadCells) * 2f;
            telemetry.StallRefreshes++;
          } else if (reason == DomainRefreshReason.UnitEscaped) {
            telemetry.EscapeRefreshes++;
          }
          CCDomainOps.PaddedBounds(
            unitMin, unitMax, goalMin, goalMax, pad,
            out var paddedMin, out var paddedMax);

          domain.PendingPaddedMin = paddedMin;
          domain.PendingPaddedMax = paddedMax;
          domain.PendingReason = reason;
          domain.PadCellsUsed = pad;
          domain.CachedGoalCentroid = goalCentroid;
          domain.CachedGoalCount = goals.Length;
          domain.CachedGroupCentroid = centroid;
          domain.CachedGroupRadius = radius;
          domain.DirtyFlags[0] = 0;
          domain.DirtyFlags[1] = 0;
          domain.Valid = false;

          // keep the goal set/list in sync when the goal moved (arrival
          // tests + BFS seeds read these)
          if (reason == DomainRefreshReason.GoalChanged
            || reason == DomainRefreshReason.NeverBuilt) {
            RebuildGoalSet(em, e, ref domain, goals, fields.Indexer);
          }

          em.SetComponentData(e, domain);
          group.Phase = SolvePhase.DomainRefreshRequested;
          em.SetComponentData(e, group);
          telemetry.SolvesStarted++;
        }
        extents.Dispose();
      }

      tick.SolveThisFrame = anySolveStarting;
    }

    private struct Extent
    {
      public float2 Min;
      public float2 Max;
      public float2 Sum;
      public int Count;
    }

    private NativeHashMap<int, Extent> ComputeGroupExtents(
      ref SystemState state, in GlobalFields fields)
    {
      var transforms = _unitQuery.ToComponentDataArray<LocalTransform>(state.WorldUpdateAllocator);
      var units = _unitQuery.ToComponentDataArray<CCUnit>(state.WorldUpdateAllocator);
      var map = new NativeHashMap<int, Extent>(16, Allocator.Temp);
      for (int i = 0; i < transforms.Length; i++) {
        var pos = CCMath.WorldToGrid(transforms[i].Position, fields.Origin, fields.CellSize);
        if (map.TryGetValue(units[i].GroupId, out var e)) {
          e.Min = math.min(e.Min, pos);
          e.Max = math.max(e.Max, pos);
          e.Sum += pos;
          e.Count++;
        } else {
          e = new Extent { Min = pos, Max = pos, Sum = pos, Count = 1 };
        }
        map[units[i].GroupId] = e;
      }
      return map;
    }

    private static void RebuildGoalSet(
      EntityManager em, Entity e, ref DomainCache domain,
      DynamicBuffer<GoalCell> goals, in GridIndexer gi)
    {
      if (domain.GoalSet.Capacity < goals.Length) {
        domain.GoalSet.Dispose();
        domain.GoalSet = new NativeParallelHashMap<int, byte>(
          math.max(64, goals.Length * 2), Allocator.Persistent);
        SyncCleanup(em, e, domain);
      }
      domain.GoalSet.Clear();
      domain.GoalCellList.Clear();
      for (int i = 0; i < goals.Length; i++) {
        if (gi.InBounds(goals[i].Cell)) {
          domain.GoalSet.TryAdd(gi.Flat(goals[i].Cell), 1);
          domain.GoalCellList.Add(goals[i].Cell);
        }
      }
    }

    /// <summary>
    /// Grow the domain-compact scratch + the BACK velocity buffer to fit the
    /// new domain (1.5× growth, spec §8.5). Only called when the group is
    /// idle (chain completed), and never touches the FRONT velocity/lookup —
    /// advection may be reading them right now.
    /// </summary>
    private static void EnsureScratchCapacity(
      EntityManager em, Entity e, ref GroupFieldBuffers buffers, int needed)
    {
      var group = em.GetComponentData<CCGroup>(e);
      int back = 1 - group.ActiveBuffer;
      var backVelocity = back == 0 ? buffers.Velocity0 : buffers.Velocity1;

      bool scratchOk = buffers.Capacity >= needed;
      bool backOk = backVelocity.Length >= needed;
      if (scratchOk && backOk) {
        return;
      }
      int newCap = math.max(needed, (int)math.ceil(buffers.Capacity * 1.5f));

      if (!scratchOk) {
        buffers.F.Dispose();
        buffers.C.Dispose();
        buffers.Phi.Dispose();
        buffers.CellState.Dispose();
        buffers.HeapCells.Dispose();
        buffers.HeapKeys.Dispose();
        buffers.HeapPos.Dispose();
        buffers.F = new NativeArray<float4>(newCap, Allocator.Persistent);
        buffers.C = new NativeArray<float4>(newCap, Allocator.Persistent);
        buffers.Phi = new NativeArray<float>(newCap, Allocator.Persistent);
        buffers.CellState = new NativeArray<byte>(newCap, Allocator.Persistent);
        buffers.HeapCells = new NativeArray<int>(newCap, Allocator.Persistent);
        buffers.HeapKeys = new NativeArray<float>(newCap, Allocator.Persistent);
        buffers.HeapPos = new NativeArray<int>(newCap, Allocator.Persistent);
      }
      if (!backOk) {
        backVelocity.Dispose();
        var grown = new NativeArray<float2>(newCap, Allocator.Persistent);
        if (back == 0) buffers.Velocity0 = grown; else buffers.Velocity1 = grown;
      }
      em.SetComponentData(e, buffers);
      SyncCleanup(em, e, em.GetComponentData<DomainCache>(e), buffers);
    }

    internal static void SyncCleanup(EntityManager em, Entity e, in DomainCache domain)
      => SyncCleanup(em, e, domain, em.GetComponentData<GroupFieldBuffers>(e));

    internal static void SyncCleanup(
      EntityManager em, Entity e, in DomainCache domain, in GroupFieldBuffers buffers)
    {
      var cleanup = em.GetComponentData<GroupFieldBuffersCleanup>(e);
      cleanup.Buffers = buffers;
      cleanup.Domain = domain;
      em.SetComponentData(e, cleanup);
    }
  }
}
