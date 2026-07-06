using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Eikonal solve pass (spec §9, Decision D8 Phase-1 half): schedules one
  /// Burst FMM IJob per group on solve ticks. FMM is inherently serial (one
  /// priority queue) — parallelism comes from group solves running
  /// concurrently on separate workers. FIM + the hybrid decider are Phase 4.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCFieldSystem))]
  [BurstCompile]
  public partial struct CCEikonalSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
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

      var handles = new NativeList<JobHandle>(8, Allocator.Temp);
      foreach (var (buffers, entity) in
        SystemAPI.Query<RefRO<GroupFieldBuffers>>()
          .WithAll<CCGroup, CCGroupInitialized>()
          .WithEntityAccess()) {
        var goals = SystemAPI.GetBuffer<GoalCell>(entity)
          .Reinterpret<int2>().AsNativeArray();
        handles.Add(new FmmSolveJob {
          Gi = fields.Indexer,
          C = buffers.ValueRO.C,
          Discomfort = fields.Discomfort,
          GoalCells = goals,
          MaxWeight = constants.maxWeight,
          MinWeight = constants.minWeight,
          Phi = buffers.ValueRO.Phi,
          State = buffers.ValueRO.CellState,
          HeapCells = buffers.ValueRO.HeapCells,
          HeapKeys = buffers.ValueRO.HeapKeys,
          HeapPos = buffers.ValueRO.HeapPos,
        }.Schedule(state.Dependency));
      }
      if (handles.Length > 0) {
        state.Dependency = JobHandle.CombineDependencies(handles.AsArray());
      }
    }

    /// <summary>Thin job wrapper: all logic lives in the parity-tested CCFmmSolver.</summary>
    [BurstCompile]
    private struct FmmSolveJob : IJob
    {
      public GridIndexer Gi;
      [ReadOnly] public NativeArray<float4> C;
      [ReadOnly] public NativeArray<float> Discomfort;
      [ReadOnly] public NativeArray<int2> GoalCells;
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
          Gi, C, Discomfort, GoalCells, MaxWeight, MinWeight,
          Phi, State, HeapCells, HeapKeys, HeapPos);
      }
    }
  }
}
