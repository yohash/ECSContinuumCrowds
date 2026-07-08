using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Eikonal root handling for FIM iteration (spec §10.4 risk + fallback).
  ///
  /// WeightedBlend (default): iterate with the repo's weighted-mean root
  /// (D9) — the same update FMM uses. Known risk (spec §10.4): the blend
  /// slightly weakens the upwind monotonicity FIM's convergence proofs
  /// assume; the max root satisfies it strictly. Empirical agreement with
  /// FMM is the acceptance bar (standing parity test); sweep caps guarantee
  /// termination regardless.
  ///
  /// MaxRootWithBlendedPostPass (documented fallback, spec §10.4): iterate
  /// with the pure max root (paper-faithful, provably monotone), then a
  /// single final post-pass re-evaluates every cell once with the weighted
  /// blend. LOG WHICH PATH SHIPPED: WeightedBlend is the shipping default;
  /// flip CCSolveSettings.FimRootMode if in-editor validation shows the
  /// blend destabilizing convergence (FimStatus hit-cap telemetry).
  /// </summary>
  public enum FimRootMode : byte
  {
    WeightedBlend = 0,
    MaxRootWithBlendedPostPass = 1,
  }

  /// <summary>
  /// Fast Iterative Method eikonal solver (spec §10, Decision D8's second
  /// half; Jeong &amp; Whitaker 2008, CPU-adapted per-cell form): cells sit on
  /// an active list and are iteratively relaxed with the SAME update formula
  /// as FMM (CCMath.EikonalSolve — one function, two drivers, spec §16.4)
  /// until convergence. No global priority queue, so active cells within a
  /// sweep relax in parallel — FIM does modestly more total work than FMM
  /// but scales across all workers, which is why the hybrid decider sends
  /// large domains here (spec §10.3).
  ///
  /// In-place φ (spec §10.2 CPU note): each work item writes ONLY its own
  /// cell's φ; neighbor reads may observe same-sweep values (Gauss–Seidel
  /// flavored). The spec sanctions this — with monotone-decreasing φ and
  /// idempotent updates, races only affect the PATH to convergence, never
  /// the result: the fixed point of the update is unique, so the final φ is
  /// deterministic even though sweep counts may vary run to run.
  ///
  /// ⚠ Semantic note vs FMM parity (spec §10.4): our FMM preserves the
  /// repo's quirk of reading previously-ACCEPTED neighbors as +∞; FIM has no
  /// accepted set and converges to the full-information fixed point. On
  /// benign fields the two agree tightly; under strong cost heterogeneity
  /// FIM's φ can be marginally LOWER (never higher). The standing parity
  /// test bounds the drift; empirical agreement is the acceptance bar.
  ///
  /// State bits share the CellState array with FMM: Goal=2 (CCCellState),
  /// Active=4, Pending=8. Goal cells are never relaxed and always read as
  /// 0 + C (the repo's goal guard, preserved).
  /// </summary>
  public static class CCFimSolver
  {
    public const byte FlagGoal = CCCellState.Goal; // 2
    public const byte FlagActive = 4;
    public const byte FlagPending = 8;

    // FimStatus layout
    public const int StatusConverged = 0;
    public const int StatusSweeps = 1;
    public const int StatusHitCap = 2;

    /// <summary>Weighted or max-root update for one cell from current φ.</summary>
    public static float ComputeUpdate(
      int n,
      in NativeArray<int> cells,
      in NativeArray<int4> neighbors,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
      in NativeArray<float> phi,
      in NativeArray<byte> state,
      float maxWeight,
      float minWeight
    )
    {
      var Cn = C[n];
      var nbs = neighbors[n];
      var phi_m = new float4(float.PositiveInfinity);
      for (int dd = 0; dd < CCMath.NumDirections; dd++) {
        int nn = nbs[dd];
        if (nn < 0) {
          continue; // out-of-domain = ∞ (spec §8.2)
        }
        if ((state[nn] & FlagGoal) != 0) {
          // goal cells have φ of 0 (repo goal guard, incl. invalid goals in
          // identity/parity domains)
          phi_m[dd] = 0f + Cn[dd];
        } else if (discomfortGlobal[cells[nn]] < 1f) {
          phi_m[dd] = phi[nn] + Cn[dd];
        }
      }
      return CCMath.EikonalSolve(phi_m, Cn, maxWeight, minWeight);
    }

    /// <summary>Iteration-time root weights for the configured mode.</summary>
    public static void IterationWeights(
      FimRootMode mode, float maxWeight, float minWeight,
      out float wMax, out float wMin)
    {
      if (mode == FimRootMode.MaxRootWithBlendedPostPass) {
        // pure max root == weighted blend with weights (1, 0)
        wMax = 1f;
        wMin = 0f;
      } else {
        wMax = maxWeight;
        wMin = minWeight;
      }
    }

    /// <summary>
    /// Initialize φ/state and seed the active list with the neighbors of the
    /// goal cells (spec §10.2). Serial (runs as one small job).
    /// </summary>
    public static void Init(
      int cellCount,
      in NativeArray<int> cells,
      in NativeArray<int4> neighbors,
      in NativeParallelHashMap<int, int> globalToLocal,
      in GridIndexer gi,
      in NativeArray<float> discomfortGlobal,
      in NativeArray<int2> goalCells,
      NativeArray<float> phi,
      NativeArray<byte> state,
      NativeList<int> clean,
      NativeList<int> raw,
      NativeArray<int> status
    )
    {
      for (int i = 0; i < cellCount; i++) {
        phi[i] = float.PositiveInfinity;
        state[i] = 0;
      }
      clean.Clear();
      raw.Clear();
      status[StatusConverged] = 0;
      status[StatusSweeps] = 0;
      status[StatusHitCap] = 0;

      for (int i = 0; i < goalCells.Length; i++) {
        var gc = goalCells[i];
        if (!gi.InBounds(gc) || !globalToLocal.TryGetValue(gi.Flat(gc), out int local)) {
          continue;
        }
        state[local] |= FlagGoal;
        if (discomfortGlobal[cells[local]] < 1f) {
          phi[local] = 0f;
        }
      }
      // active seed = walkable non-goal neighbors of goal cells
      for (int i = 0; i < goalCells.Length; i++) {
        var gc = goalCells[i];
        if (!gi.InBounds(gc) || !globalToLocal.TryGetValue(gi.Flat(gc), out int local)) {
          continue;
        }
        var nbs = neighbors[local];
        for (int d = 0; d < CCMath.NumDirections; d++) {
          int nb = nbs[d];
          if (nb < 0 || (state[nb] & (FlagGoal | FlagActive)) != 0) {
            continue;
          }
          if (discomfortGlobal[cells[nb]] >= 1f) {
            continue;
          }
          state[nb] |= FlagActive;
          clean.Add(nb);
        }
      }
      if (clean.Length == 0) {
        status[StatusConverged] = 1;
      }
    }

    /// <summary>
    /// Relax one active cell (spec §10.2 loop body) — the shared core of the
    /// parallel sweep job and the serial finisher. Writes ONLY phi[n]/state[n]
    /// plus Pending flags + raw-list entries for cells it activates
    /// (idempotent same-bit byte stores; duplicates deduped by Compact).
    /// </summary>
    public static void RelaxCell(
      int n,
      in NativeArray<int> cells,
      in NativeArray<int4> neighbors,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
      NativeArray<float> phi,
      NativeArray<byte> state,
      float eps,
      float wMax,
      float wMin,
      ref NativeList<int>.ParallelWriter raw
    )
    {
      if ((state[n] & FlagGoal) != 0) {
        return;
      }
      float p = ComputeUpdate(n, cells, neighbors, C, discomfortGlobal, phi, state, wMax, wMin);
      float pMin = math.min(phi[n], p);

      if (phi[n] - pMin < eps || float.IsInfinity(pMin)) {
        // converged (or still unreachable): retire and probe neighbors
        phi[n] = pMin;
        state[n] = (byte)(state[n] & ~(FlagActive | FlagPending));
        var nbs = neighbors[n];
        for (int d = 0; d < CCMath.NumDirections; d++) {
          int nb = nbs[d];
          if (nb < 0 || (state[nb] & (FlagGoal | FlagActive | FlagPending)) != 0) {
            continue;
          }
          if (discomfortGlobal[cells[nb]] >= 1f) {
            continue;
          }
          float q = ComputeUpdate(nb, cells, neighbors, C, discomfortGlobal, phi, state, wMax, wMin);
          if (q < phi[nb]) {
            state[nb] |= FlagPending;
            raw.AddNoResize(nb);
          }
        }
      } else {
        // still improving: keep iterating next sweep
        phi[n] = pMin;
        state[n] = (byte)((state[n] & ~FlagActive) | FlagPending);
        raw.AddNoResize(n);
      }
    }

    /// <summary>
    /// Between-sweep list management (spec §10.2): promote Pending entries
    /// from the raw list into the clean list (duplicates skip — the first
    /// promotion clears Pending), clear raw, flag convergence when empty.
    /// </summary>
    public static void Compact(
      NativeList<int> clean,
      NativeList<int> raw,
      NativeArray<byte> state,
      NativeArray<int> status
    )
    {
      if (status[StatusConverged] != 0) {
        raw.Clear();
        clean.Clear();
        return;
      }
      clean.Clear();
      for (int i = 0; i < raw.Length; i++) {
        int e = raw[i];
        if ((state[e] & FlagPending) != 0) {
          state[e] = (byte)((state[e] & ~FlagPending) | FlagActive);
          clean.Add(e);
        }
      }
      raw.Clear();
      status[StatusSweeps]++;
      if (clean.Length == 0) {
        status[StatusConverged] = 1;
      }
    }

    /// <summary>
    /// Serial tail: finish any sweeps the fixed parallel-batch didn't cover
    /// (pathological characteristics — spiral mazes — need many sweeps), cap
    /// at maxSweeps (termination guarantee; sets HitCap telemetry — if this
    /// ever fires with WeightedBlend, ship the MaxRootWithBlendedPostPass
    /// fallback, spec §10.4). Then the optional weighted post-pass.
    /// </summary>
    public static void Finish(
      int cellCount,
      in NativeArray<int> cells,
      in NativeArray<int4> neighbors,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
      NativeArray<float> phi,
      NativeArray<byte> state,
      NativeList<int> clean,
      NativeList<int> raw,
      NativeArray<int> status,
      NativeArray<float> scratch,
      float eps,
      int maxSweeps,
      FimRootMode mode,
      float maxWeight,
      float minWeight
    )
    {
      IterationWeights(mode, maxWeight, minWeight, out float wMax, out float wMin);
      var rawWriter = raw.AsParallelWriter();
      while (status[StatusConverged] == 0) {
        if (status[StatusSweeps] >= maxSweeps) {
          status[StatusHitCap] = 1;
          break;
        }
        for (int i = 0; i < clean.Length; i++) {
          RelaxCell(clean[i], cells, neighbors, C, discomfortGlobal,
            phi, state, eps, wMax, wMin, ref rawWriter);
        }
        Compact(clean, raw, state, status);
      }

      if (mode == FimRootMode.MaxRootWithBlendedPostPass) {
        // single weighted-blend re-evaluation over the converged max-root φ
        // (two-phase through scratch so the pass is order-independent);
        // scratch is the FMM heap-keys array — unused by FIM, capacity ≥ count
        for (int n = 0; n < cellCount; n++) {
          if ((state[n] & FlagGoal) != 0 || float.IsInfinity(phi[n])) {
            scratch[n] = phi[n];
            continue;
          }
          float p = ComputeUpdate(n, cells, neighbors, C, discomfortGlobal,
            phi, state, maxWeight, minWeight);
          scratch[n] = math.min(phi[n], p);
        }
        for (int n = 0; n < cellCount; n++) {
          phi[n] = scratch[n];
        }
      }
    }

    /// <summary>
    /// Complete single-threaded driver — used by the parity tests and the
    /// crossover benchmark harness (spec §10.3/§14.5). Returns sweep count.
    /// </summary>
    public static int SolveSerial(
      int cellCount,
      in NativeArray<int> cells,
      in NativeArray<int4> neighbors,
      in NativeParallelHashMap<int, int> globalToLocal,
      in GridIndexer gi,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
      in NativeArray<int2> goalCells,
      float eps,
      int maxSweeps,
      FimRootMode mode,
      float maxWeight,
      float minWeight,
      NativeArray<float> phi,
      NativeArray<byte> state,
      NativeList<int> clean,
      NativeList<int> raw,
      NativeArray<int> status,
      NativeArray<float> scratch
    )
    {
      Init(cellCount, cells, neighbors, globalToLocal, gi, discomfortGlobal,
        goalCells, phi, state, clean, raw, status);
      Finish(cellCount, cells, neighbors, C, discomfortGlobal, phi, state,
        clean, raw, status, scratch, eps, maxSweeps, mode, maxWeight, minWeight);
      return status[StatusSweeps];
    }
  }
}
