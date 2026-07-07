using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>Per-group solve pipeline state (spec §3.4), driven by CCSchedulerSystem.</summary>
  public enum SolvePhase : byte
  {
    /// <summary>No pipeline in flight; eligible for its stagger slot.</summary>
    Idle = 0,
    /// <summary>Scheduler flagged an invalid domain; CCDomainSystem starts the BFS this frame.</summary>
    DomainRefreshRequested = 1,
    /// <summary>Flood-fill job in flight (polled by the scheduler).</summary>
    DomainRefreshing = 2,
    /// <summary>Fields→eikonal→velocity chain being assembled this frame.</summary>
    SolveRequested = 3,
    /// <summary>Solve chain in flight (polled; flip on completion).</summary>
    Solving = 4,
  }

  /// <summary>
  /// A solve group / request (spec §3.4): one goal region, one potential/
  /// velocity solve, any number of member units.
  /// </summary>
  public struct CCGroup : IComponentData
  {
    public int GroupId;
    /// <summary>Per-group cost weight overrides; seeded from authoring.</summary>
    public float Alpha;
    public float Beta;
    public float Gamma;

    public SolvePhase Phase;
    /// <summary>Stagger slot (spec §12.3), assigned round-robin at init.</summary>
    public int ScheduleSlot;
    /// <summary>Which velocity buffer advection reads (0/1). The solve
    /// pipeline only ever writes buffer 1−ActiveBuffer; the flip happens on
    /// the main thread after the chain's handle completes — race-free by
    /// construction (spec §3.6).</summary>
    public int ActiveBuffer;
    public double LastSolveTime;
  }

  /// <summary>
  /// Tail JobHandle of the group's in-flight pipeline chain.
  ///
  /// ⚠ NOTE (deviation from spec §12.4's letter, same intent): the spec
  /// suggests a managed Dictionary side-table because JobHandles in
  /// components are "not usefully storable across frames without care".
  /// We store the handle in this unmanaged component WITH the required care
  /// — single-writer discipline (only the pipeline systems on the frame a
  /// chain is assembled, only the scheduler thereafter), poll → Complete →
  /// flip, and chains never entangle state.Dependency (container-only jobs
  /// with explicit dependencies). This keeps the scheduler Burst-compiled
  /// with zero managed state; the spec's stated goal ("the scheduler is the
  /// one permitted non-Burst brain") is bettered, not violated.
  /// </summary>
  public struct CCGroupSolveState : IComponentData
  {
    public JobHandle ChainTail;
  }

  /// <summary>
  /// Enabled by the scheduler on the frame a group's solve chain should be
  /// assembled; the field/eikonal/velocity systems append their jobs to
  /// ChainTail for enabled groups; the velocity system disables it and sets
  /// Phase = Solving.
  /// </summary>
  public struct CCGroupSolveRequest : IComponentData, IEnableableComponent
  {
  }

  /// <summary>Goal region as grid cells (spec §3.4).</summary>
  [InternalBufferCapacity(0)]
  public struct GoalCell : IBufferElementData
  {
    public int2 Cell;
  }

  /// <summary>
  /// Authored goal region in world space (XZ min/max rect), converted to
  /// GoalCell entries once GlobalFields exists (CCGroupInitSystem).
  /// </summary>
  public struct CCGroupGoalRect : IComponentData
  {
    public float2 MinXZ;
    public float2 MaxXZ;
  }

  /// <summary>Marks a group whose runtime state (buffers, goal cells) is initialized.</summary>
  public struct CCGroupInitialized : IComponentData
  {
  }

  /// <summary>
  /// Cached solve domain (spec §3.4/§8.4, Decisions D5+D6): the flood-filled
  /// compact working set plus everything needed to decide, cheaply, whether
  /// it is still valid. Containers are Persistent, owned by the group,
  /// disposed via <see cref="GroupFieldBuffersCleanup"/>.
  /// </summary>
  public struct DomainCache : IComponentData
  {
    /// <summary>Flat global indices in the domain, BFS order.</summary>
    public NativeList<int> Cells;
    /// <summary>global flat idx → compact domain idx. Capacity = full grid
    /// (allocated once) so the fill job never needs a main-thread resize.</summary>
    public NativeParallelHashMap<int, int> GlobalToLocal;
    /// <summary>Per-cell E,N,W,S local indices, −1 absent (spec §8.3 — hot
    /// loops do zero hashing).</summary>
    public NativeList<int4> NeighborLocalIdx;
    /// <summary>Goal cells as a set for O(1) arrival tests in advection
    /// (rebuilt with the goal, independent of solver scratch/buffers).</summary>
    public NativeParallelHashMap<int, byte> GoalSet;
    /// <summary>Goal cells as a persistent list — BFS seeds and FMM goal
    /// seeding read this instead of the entity buffer so multi-frame jobs
    /// never alias chunk memory (structural changes would invalidate it).</summary>
    public NativeList<int2> GoalCellList;
    /// <summary>[0] = unit escaped front domain, [1] = unit stalled —
    /// written by advection (identical-value byte stores, benign), read and
    /// cleared by the scheduler on the group's tick (spec §8.4 hard
    /// triggers).</summary>
    public NativeArray<byte> DirtyFlags;

    public int2 CachedGoalCentroidCell;
    public float2 CachedGoalCentroid;
    public int CachedGoalCount;
    public float2 CachedGroupCentroid;
    public float CachedGroupRadius;
    public int WalkabilityVersion;
    public bool Valid;
    /// <summary>Pad used for the CURRENT fill; doubled after a stall trigger
    /// (spec §8.6), restored to settings pad on the next normal refresh.</summary>
    public float PadCellsUsed;
    /// <summary>Scheduler → domain-system handoff: fill bounds for the
    /// refresh requested this frame.</summary>
    public int2 PendingPaddedMin;
    public int2 PendingPaddedMax;
    public DomainRefreshReason PendingReason;

    public bool IsCreated => Cells.IsCreated;

    public void Dispose()
    {
      if (Cells.IsCreated) Cells.Dispose();
      if (GlobalToLocal.IsCreated) GlobalToLocal.Dispose();
      if (NeighborLocalIdx.IsCreated) NeighborLocalIdx.Dispose();
      if (GoalSet.IsCreated) GoalSet.Dispose();
      if (GoalCellList.IsCreated) GoalCellList.Dispose();
      if (DirtyFlags.IsCreated) DirtyFlags.Dispose();
    }
  }

  /// <summary>
  /// Per-group solve buffers (spec §3.4), domain-compact (spec §8.3):
  /// scratch arrays sized to the domain (capacity-grown 1.5×, reallocated
  /// only when the domain outgrows them — spec §8.5), plus the
  /// double-buffered velocity output (Decision D7).
  ///
  /// Snapshot invariant (spec §12.2): each velocity buffer carries its own
  /// full-grid localIdxLookup (1 MB at 512² — ships O(1) advection sampling,
  /// no hashing in the per-frame hot path). Advection only ever reads buffer
  /// ActiveBuffer and its lookup; a solve only ever writes the other pair.
  /// Domain refreshes rewrite DomainCache but never touch either lookup, so
  /// a front buffer stays interpretable until it is overwritten as a back
  /// buffer — the lookup IS the snapshot.
  /// </summary>
  public struct GroupFieldBuffers : IComponentData
  {
    // domain-compact scratch (single-buffered; only the in-flight chain touches these)
    public NativeArray<float4> F;
    public NativeArray<float4> C;
    public NativeArray<float> Phi;
    public NativeArray<byte> CellState;
    public NativeArray<int> HeapCells;
    public NativeArray<float> HeapKeys;
    public NativeArray<int> HeapPos;
    /// <summary>Cell count of the domain the in-flight/last chain was
    /// scheduled over (jobs and the visualizer read compact arrays up to
    /// this length). The live DomainCache is stable for the whole chain —
    /// the scheduler never starts a refresh while a chain is in flight.</summary>
    public int DomainLength;

    // double-buffered output (spec §3.6/§12.2)
    public NativeArray<float2> Velocity0;
    public NativeArray<float2> Velocity1;
    public NativeArray<int> LocalIdxLookup0; // full grid, −1 = not in that snapshot
    public NativeArray<int> LocalIdxLookup1;

    public bool IsCreated => Phi.IsCreated;
    public int Capacity => Phi.Length;

    public void Dispose()
    {
      if (F.IsCreated) F.Dispose();
      if (C.IsCreated) C.Dispose();
      if (Phi.IsCreated) Phi.Dispose();
      if (CellState.IsCreated) CellState.Dispose();
      if (HeapCells.IsCreated) HeapCells.Dispose();
      if (HeapKeys.IsCreated) HeapKeys.Dispose();
      if (HeapPos.IsCreated) HeapPos.Dispose();
      if (Velocity0.IsCreated) Velocity0.Dispose();
      if (Velocity1.IsCreated) Velocity1.Dispose();
      if (LocalIdxLookup0.IsCreated) LocalIdxLookup0.Dispose();
      if (LocalIdxLookup1.IsCreated) LocalIdxLookup1.Dispose();
    }
  }

  /// <summary>
  /// Cleanup component (spec §3.4): duplicates the container references so a
  /// destroyed group cannot leak them. ChainTail mirrors the group's
  /// in-flight handle so mid-solve destruction can Complete() before
  /// disposal (spec §12.4 Dispose(JobHandle) intent).
  /// </summary>
  public struct GroupFieldBuffersCleanup : ICleanupComponentData
  {
    public GroupFieldBuffers Buffers;
    public DomainCache Domain;
    public JobHandle ChainTail;

    public void Dispose()
    {
      ChainTail.Complete();
      Buffers.Dispose();
      Domain.Dispose();
    }
  }
}
