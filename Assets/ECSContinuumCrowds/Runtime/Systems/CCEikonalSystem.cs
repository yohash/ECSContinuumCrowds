using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Eikonal solve pass with the HYBRID decider (spec §9–10, Decision D8):
  ///
  ///   solver = (domain ≥ FimThresholdCells && expectedIdleWorkers ≥ min)
  ///            ? FIM : FMM
  ///
  /// FMM (small/medium domains): one serial Burst IJob per group —
  /// parallelism comes from group solves running concurrently. FIM (large
  /// domains): a pre-scheduled batch of parallel relax/compact sweep pairs
  /// over the active list (sweeps past convergence are no-ops over empty
  /// deferred lists) plus a serial finisher for pathological characteristic
  /// lengths — the solve goes WIDE across idle workers instead of pinning
  /// one core (spec §10.1).
  ///
  /// expectedIdleWorkers = JobWorkerCount − chains already in flight −
  /// chains scheduled earlier this frame (spec §10.3: if the workers are
  /// already saturated by concurrent FMM solves, FIM's parallelism buys
  /// nothing — prefer FMM and let the multi-frame scheduler absorb latency).
  ///
  /// FimThresholdCells is the spec's PLACEHOLDER 32,768 — replace with the
  /// crossover benchmark harness's measured value (CrossoverBenchmark test,
  /// spec §10.3/§14.5).
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
      state.RequireForUpdate<CCSolveSettings>();
    }

    // NOT Burst-compiled: JobsUtility.JobWorkerCount + per-group chain
    // assembly is trivial scheduling work; every job it schedules is Burst
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var constants = SystemAPI.GetSingleton<CCConstants>();
      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      ref var telemetry = ref SystemAPI.GetSingletonRW<CCSolveTelemetry>().ValueRW;

      // in-flight chains occupy workers (FMM = 1 each; refreshes are short)
      int inFlight = 0;
      foreach (var g in SystemAPI.Query<RefRO<CCGroup>>().WithAll<CCGroupInitialized>()) {
        if (g.ValueRO.Phase == SolvePhase.Solving
          || g.ValueRO.Phase == SolvePhase.DomainRefreshing) {
          inFlight++;
        }
      }
      int scheduledThisFrame = 0;

      foreach (var (domain, buffers, solveState) in
        SystemAPI.Query<RefRO<DomainCache>, RefRO<GroupFieldBuffers>, RefRW<CCGroupSolveState>>()
          .WithAll<CCGroup, CCGroupInitialized, CCGroupSolveRequest>()) {
        int count = buffers.ValueRO.DomainLength;
        if (count == 0) {
          continue;
        }

        int expectedIdleWorkers =
          JobsUtility.JobWorkerCount - inFlight - scheduledThisFrame;
        bool useFim = settings.FimThresholdCells > 0
          && count >= settings.FimThresholdCells
          && expectedIdleWorkers >= math.max(settings.FimMinIdleWorkers, 1);
        scheduledThisFrame++;

        var d = domain.ValueRO;
        var b = buffers.ValueRO;
        var tail = solveState.ValueRO.ChainTail;

        if (!useFim) {
          telemetry.FmmSolves++;
          solveState.ValueRW.ChainTail = new FmmSolveJob {
            CellCount = count,
            Cells = d.Cells.AsArray(),
            Neighbors = d.NeighborLocalIdx.AsArray(),
            GlobalToLocal = d.GlobalToLocal,
            Gi = fields.Indexer,
            C = b.C,
            Discomfort = fields.Discomfort,
            GoalCells = d.GoalCellList,
            MaxWeight = constants.maxWeight,
            MinWeight = constants.minWeight,
            Phi = b.Phi,
            State = b.CellState,
            HeapCells = b.HeapCells,
            HeapKeys = b.HeapKeys,
            HeapPos = b.HeapPos,
          }.Schedule(tail);
          continue;
        }

        telemetry.FimSolves++;
        CCFimSolver.IterationWeights(
          settings.FimRootMode, constants.maxWeight, constants.minWeight,
          out float wMax, out float wMin);

        tail = new FimInitJob {
          CellCount = count,
          Cells = d.Cells.AsArray(),
          Neighbors = d.NeighborLocalIdx.AsArray(),
          GlobalToLocal = d.GlobalToLocal,
          Gi = fields.Indexer,
          Discomfort = fields.Discomfort,
          GoalCells = d.GoalCellList,
          Phi = b.Phi,
          State = b.CellState,
          Clean = b.FimActiveClean,
          Raw = b.FimActiveRaw,
          Status = b.FimStatus,
        }.Schedule(tail);

        for (int sweep = 0; sweep < math.max(settings.FimParallelSweeps, 1); sweep++) {
          var relax = new FimRelaxJob {
            Cells = d.Cells.AsArray(),
            Neighbors = d.NeighborLocalIdx.AsArray(),
            C = b.C,
            Discomfort = fields.Discomfort,
            Phi = b.Phi,
            State = b.CellState,
            Clean = b.FimActiveClean.AsDeferredJobArray(),
            Raw = b.FimActiveRaw.AsParallelWriter(),
            Eps = settings.FimEps,
            WMax = wMax,
            WMin = wMin,
          }.Schedule(b.FimActiveClean, 32, tail);
          tail = new FimCompactJob {
            Clean = b.FimActiveClean,
            Raw = b.FimActiveRaw,
            State = b.CellState,
            Status = b.FimStatus,
          }.Schedule(relax);
        }

        solveState.ValueRW.ChainTail = new FimFinishJob {
          CellCount = count,
          Cells = d.Cells.AsArray(),
          Neighbors = d.NeighborLocalIdx.AsArray(),
          C = b.C,
          Discomfort = fields.Discomfort,
          Phi = b.Phi,
          State = b.CellState,
          Clean = b.FimActiveClean,
          Raw = b.FimActiveRaw,
          Status = b.FimStatus,
          Scratch = b.HeapKeys, // unused by FIM; doubles as post-pass scratch
          Eps = settings.FimEps,
          MaxSweeps = math.max(settings.FimMaxSweeps, settings.FimParallelSweeps),
          Mode = settings.FimRootMode,
          MaxWeight = constants.maxWeight,
          MinWeight = constants.minWeight,
        }.Schedule(tail);
      }
    }

    /// <summary>Thin job wrapper: all logic lives in the parity-tested CCFmmSolver.</summary>
    [BurstCompile]
    private struct FmmSolveJob : IJob
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

    [BurstCompile]
    private struct FimInitJob : IJob
    {
      public int CellCount;
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeParallelHashMap<int, int> GlobalToLocal;
      public GridIndexer Gi;
      [ReadOnly] public NativeArray<float> Discomfort;
      [ReadOnly] public NativeList<int2> GoalCells;
      public NativeArray<float> Phi;
      public NativeArray<byte> State;
      public NativeList<int> Clean;
      public NativeList<int> Raw;
      public NativeArray<int> Status;

      public void Execute()
      {
        CCFimSolver.Init(
          CellCount, Cells, Neighbors, GlobalToLocal, Gi, Discomfort,
          GoalCells.AsArray(), Phi, State, Clean, Raw, Status);
      }
    }

    /// <summary>
    /// One Jacobi-flavored sweep over the active list (spec §10.2). Each
    /// item writes only its own cell's φ/state plus idempotent Pending
    /// flags; the deferred list is empty once converged, making leftover
    /// pre-scheduled sweeps effectively free.
    /// </summary>
    [BurstCompile]
    private struct FimRelaxJob : IJobParallelForDefer
    {
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeArray<float4> C;
      [ReadOnly] public NativeArray<float> Discomfort;
      [NativeDisableParallelForRestriction] public NativeArray<float> Phi;
      [NativeDisableParallelForRestriction] public NativeArray<byte> State;
      [ReadOnly] public NativeArray<int> Clean;
      public NativeList<int>.ParallelWriter Raw;
      public float Eps;
      public float WMax;
      public float WMin;

      public void Execute(int index)
      {
        CCFimSolver.RelaxCell(
          Clean[index], Cells, Neighbors, C, Discomfort, Phi, State,
          Eps, WMax, WMin, ref Raw);
      }
    }

    [BurstCompile]
    private struct FimCompactJob : IJob
    {
      public NativeList<int> Clean;
      public NativeList<int> Raw;
      public NativeArray<byte> State;
      public NativeArray<int> Status;

      public void Execute() => CCFimSolver.Compact(Clean, Raw, State, Status);
    }

    [BurstCompile]
    private struct FimFinishJob : IJob
    {
      public int CellCount;
      [ReadOnly] public NativeArray<int> Cells;
      [ReadOnly] public NativeArray<int4> Neighbors;
      [ReadOnly] public NativeArray<float4> C;
      [ReadOnly] public NativeArray<float> Discomfort;
      public NativeArray<float> Phi;
      public NativeArray<byte> State;
      public NativeList<int> Clean;
      public NativeList<int> Raw;
      public NativeArray<int> Status;
      public NativeArray<float> Scratch;
      public float Eps;
      public int MaxSweeps;
      public FimRootMode Mode;
      public float MaxWeight;
      public float MinWeight;

      public void Execute()
      {
        CCFimSolver.Finish(
          CellCount, Cells, Neighbors, C, Discomfort, Phi, State,
          Clean, Raw, Status, Scratch, Eps, MaxSweeps, Mode, MaxWeight, MinWeight);
      }
    }
  }
}
