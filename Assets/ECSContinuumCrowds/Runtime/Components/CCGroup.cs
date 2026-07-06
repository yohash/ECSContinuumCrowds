using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// A solve group / request (spec §3.4, Phase-1 subset): one goal region,
  /// one potential/velocity solve, any number of member units. The Phase-3
  /// scheduler fields (Phase, ScheduleSlot, ActiveBuffer, LastSolveTime)
  /// land with Decision D7; Phase 1 solves every group on every solve tick.
  /// </summary>
  public struct CCGroup : IComponentData
  {
    public int GroupId;
    /// <summary>Per-group cost weight overrides; seeded from CCConstants at init.</summary>
    public float Alpha;
    public float Beta;
    public float Gamma;
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

  /// <summary>
  /// Marks a group whose runtime state (buffers, goal cells) is initialized.
  /// </summary>
  public struct CCGroupInitialized : IComponentData
  {
  }

  /// <summary>
  /// Per-group solve buffers (spec §3.4), allocated Persistent by
  /// CCGroupInitSystem, disposed via <see cref="GroupFieldBuffersCleanup"/>.
  ///
  /// Phase 1: the solve domain is the FULL GRID (Decision D5's flood-filled
  /// compact domains arrive in Phase 3), so all arrays are grid-sized and the
  /// local index == the global flat index. Velocity is single-buffered;
  /// Velocity1 + domain snapshots + ActiveBuffer flipping arrive with the
  /// Phase-3 scheduler (Decision D7).
  /// </summary>
  public struct GroupFieldBuffers : IComponentData
  {
    /// <summary>Speed field f, float4 ENSW per cell (scratch).</summary>
    public NativeArray<float4> F;
    /// <summary>Cost field C, float4 ENSW per cell (scratch).</summary>
    public NativeArray<float4> C;
    /// <summary>Potential φ (scratch).</summary>
    public NativeArray<float> Phi;
    /// <summary>Velocity field read by advection.</summary>
    public NativeArray<float2> Velocity0;

    // FMM solver scratch (spec §9.5: persistent per-group scratch is the
    // default — zero per-tick allocation)
    public NativeArray<byte> CellState;   // accepted/goal flags (CCCellState)
    public NativeArray<int> HeapCells;    // binary heap: slot -> cell
    public NativeArray<float> HeapKeys;   // binary heap: slot -> priority
    public NativeArray<int> HeapPos;      // cell -> heap slot (-1 = absent)

    public bool IsCreated => F.IsCreated;
  }

  /// <summary>
  /// Cleanup component (spec §3.4): duplicates the buffer references so a
  /// destroyed group entity cannot leak its native containers — disposal
  /// happens when this component is removed, after any in-flight solve.
  /// </summary>
  public struct GroupFieldBuffersCleanup : ICleanupComponentData
  {
    public NativeArray<float4> F;
    public NativeArray<float4> C;
    public NativeArray<float> Phi;
    public NativeArray<float2> Velocity0;
    public NativeArray<byte> CellState;
    public NativeArray<int> HeapCells;
    public NativeArray<float> HeapKeys;
    public NativeArray<int> HeapPos;

    public void Dispose()
    {
      if (F.IsCreated) F.Dispose();
      if (C.IsCreated) C.Dispose();
      if (Phi.IsCreated) Phi.Dispose();
      if (Velocity0.IsCreated) Velocity0.Dispose();
      if (CellState.IsCreated) CellState.Dispose();
      if (HeapCells.IsCreated) HeapCells.Dispose();
      if (HeapKeys.IsCreated) HeapKeys.Dispose();
      if (HeapPos.IsCreated) HeapPos.Dispose();
    }
  }
}
