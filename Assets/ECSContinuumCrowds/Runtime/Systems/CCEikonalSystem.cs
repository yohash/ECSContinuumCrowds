using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Eikonal solve pass (spec §9, Decision D8 Phase-1 half): appends one
  /// Burst FMM IJob per requested group to its chain. FMM is inherently
  /// serial (one priority queue) — parallelism comes from group solves
  /// running concurrently on separate workers, and the chain spans frames
  /// freely (the scheduler polls; nothing blocks). FIM + the hybrid decider
  /// are Phase 4.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCFieldSystem))]
  [BurstCompile]
  public partial struct CCEikonalSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCConstants>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var constants = SystemAPI.GetSingleton<CCConstants>();

      foreach (var (domain, buffers, solveState) in
        SystemAPI.Query<RefRO<DomainCache>, RefRO<GroupFieldBuffers>, RefRW<CCGroupSolveState>>()
          .WithAll<CCGroup, CCGroupInitialized, CCGroupSolveRequest>()) {
        int count = buffers.ValueRO.DomainLength;
        if (count == 0) {
          continue;
        }
        solveState.ValueRW.ChainTail = new FmmSolveJob {
          CellCount = count,
          Cells = domain.ValueRO.Cells.AsArray(),
          Neighbors = domain.ValueRO.NeighborLocalIdx.AsArray(),
          GlobalToLocal = domain.ValueRO.GlobalToLocal,
          Gi = fields.Indexer,
          C = buffers.ValueRO.C,
          Discomfort = fields.Discomfort,
          GoalCells = domain.ValueRO.GoalCellList,
          MaxWeight = constants.maxWeight,
          MinWeight = constants.minWeight,
          Phi = buffers.ValueRO.Phi,
          State = buffers.ValueRO.CellState,
          HeapCells = buffers.ValueRO.HeapCells,
          HeapKeys = buffers.ValueRO.HeapKeys,
          HeapPos = buffers.ValueRO.HeapPos,
        }.Schedule(solveState.ValueRO.ChainTail);
      }
    }

    /// <summary>Thin job wrapper: all logic lives in the parity-tested CCFmmSolver.</summary>
    [BurstCompile]
    private struct FmmSolveJob : Unity.Jobs.IJob
    {
      public int CellCount;
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeParallelHashMap<int, int> GlobalToLocal;
      public GridIndexer Gi;
      [ReadOnly] public NativeArray<float4> C;
      [ReadOnly] public NativeArray<float> Discomfort;
      [ReadOnly] public NativeList<int2> GoalCells;
      public float MaxWeight;
      public float MinWeight;
      public NativeArray<float> Phi;
      public NativeArray<byte> State;
      public NativeArray<int> HeapCells;
      public NativeArray<float> HeapKeys;
      public NativeArray<int> HeapPos;

      public void Execute()
      {
        CCFmmSolver.Solve(
          CellCount, Cells, Neighbors, GlobalToLocal, Gi, C, Discomfort,
          GoalCells.AsArray(), MaxWeight, MinWeight,
          Phi, State, HeapCells, HeapKeys, HeapPos);
      }
    }
  }
}
