using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>Cell state flags written by the FMM solver, read by advection (goal test).</summary>
  public static class CCCellState
  {
    public const byte Accepted = 1;
    public const byte Goal = 2;
  }

  /// <summary>
  /// Fast Marching Method eikonal solver (spec §9), reproducing
  /// yohash/ContinuumCrowds EikonalSolver semantics exactly — this is the
  /// parity-tested core. Static and Burst-compatible so the FMM job and the
  /// edit-mode oracle-parity tests run the identical code path.
  ///
  /// Repo semantics preserved (verified against EikonalSolver.cs):
  /// - Goal cells are seeded INTO THE QUEUE at priority 0, not pre-accepted
  ///   (zero-priority nodes pop first — simpler and equivalent).
  /// - φ is set to 0 only for VALID goal cells (g &lt; 1); invalid goal cells
  ///   are still marked goal and enqueued, and radiate 0 + C to neighbors
  ///   via the goal-set read below.
  /// - Update TARGETS must be in-bounds, g &lt; 1, NOT accepted, NOT goal
  ///   (goal φ stays 0 forever).
  /// - φ READS for phi_m must be in-bounds, g &lt; 1, and NOT accepted —
  ///   previously-accepted cells are read as +∞ in later updates. This
  ///   deviates from textbook FMM (which reads accepted values); it still
  ///   propagates because a cell is marked accepted AFTER its neighbors are
  ///   updated, so the cell being finalized is readable at that moment.
  /// - A neighbor in the goal set that fails the read test contributes
  ///   0 + C instead (the repo's off-tile-goal guard; in our architecture
  ///   goals are always in-bounds, but multi-cell goals straddling
  ///   walkability still exercise it).
  /// - The cell popped from the queue updates its neighbors FIRST and is
  ///   marked accepted AFTER (ordering matters for parity).
  ///
  /// One deliberate deviation: exact decrease-key instead of the repo's
  /// effectively-duplicate-enqueueing managed queue (its reference-equality
  /// Contains() never matches a freshly constructed location, so
  /// UpdatePriority never fires and stale duplicates pop as no-ops). With
  /// monotone updates the final φ agrees; parity tests enforce it.
  /// </summary>
  public static class CCFmmSolver
  {
    /// <summary>
    /// Solve φ over the (Phase-1 full-grid) domain. All arrays are
    /// grid-sized; heap arrays are persistent scratch (spec §9.5).
    /// </summary>
    public static void Solve(
      in GridIndexer gi,
      in NativeArray<float4> C,
      in NativeArray<float> discomfort,
      in NativeArray<int2> goalCells,
      float maxWeight,
      float minWeight,
      NativeArray<float> phi,
      NativeArray<byte> state,
      NativeArray<int> heapCells,
      NativeArray<float> heapKeys,
      NativeArray<int> heapPos
    )
    {
      var heap = new IndexedMinHeap {
        Cells = heapCells,
        Keys = heapKeys,
        Pos = heapPos,
      };
      heap.Reset();

      for (int i = 0; i < phi.Length; i++) {
        phi[i] = float.PositiveInfinity;
        state[i] = 0;
      }

      // seed: goal cells at priority 0 (repo: markGoal + Enqueue for every
      // goal cell; φ = 0 only when the point is valid)
      for (int i = 0; i < goalCells.Length; i++) {
        var gc = goalCells[i];
        if (!gi.InBounds(gc)) {
          continue; // group init clamps goals in-bounds; guard regardless
        }
        int flat = gi.Flat(gc);
        state[flat] |= CCCellState.Goal;
        if (discomfort[flat] < 1f) {
          phi[flat] = 0f;
        }
        if (!heap.Contains(flat)) {
          heap.Push(flat, 0f);
        }
      }

      // the eikonal update loop
      while (heap.Count > 0) {
        int current = heap.PopMin();
        UpdateNeighbors(current, gi, C, discomfort, phi, state, ref heap, maxWeight, minWeight);
        state[current] |= CCCellState.Accepted; // AFTER updating neighbors
      }
    }

    /// <summary>Repo EikonalUpdateFormula: propose new φ for each valid neighbor of a popped cell.</summary>
    private static void UpdateNeighbors(
      int current,
      in GridIndexer gi,
      in NativeArray<float4> C,
      in NativeArray<float> discomfort,
      NativeArray<float> phi,
      NativeArray<byte> state,
      ref IndexedMinHeap heap,
      float maxWeight,
      float minWeight
    )
    {
      var cur = gi.Coord(current);
      for (int d = 0; d < CCMath.NumDirections; d++) {
        var nb = cur + CCMath.ENSWint(d);
        if (!gi.InBounds(nb)) {
          continue;
        }
        int nbFlat = gi.Flat(nb);
        // valid-as-target: not goal, not accepted, walkable
        if ((state[nbFlat] & (CCCellState.Accepted | CCCellState.Goal)) != 0) {
          continue;
        }
        if (discomfort[nbFlat] >= 1f) {
          continue;
        }

        // assemble phi_m from nb's own four neighbors; C indexed at nb (the
        // cell being updated) — into-cell convention already baked into C
        var Cnb = C[nbFlat];
        var phi_m = new float4(float.PositiveInfinity);
        for (int dd = 0; dd < CCMath.NumDirections; dd++) {
          var nn = nb + CCMath.ENSWint(dd);
          if (!gi.InBounds(nn)) {
            continue;
          }
          int nnFlat = gi.Flat(nn);
          if (discomfort[nnFlat] < 1f && (state[nnFlat] & CCCellState.Accepted) == 0) {
            // valid-to-read (accepted cells excluded — repo quirk, see class doc)
            phi_m[dd] = phi[nnFlat] + Cnb[dd];
          } else if ((state[nnFlat] & CCCellState.Goal) != 0) {
            // goal cells have φ of 0 (repo off-tile-goal guard)
            phi_m[dd] = 0f + Cnb[dd];
          }
        }

        float proposed = CCMath.EikonalSolve(phi_m, Cnb, maxWeight, minWeight);
        if (proposed < phi[nbFlat]) {
          phi[nbFlat] = proposed;
          heap.PushOrUpdate(nbFlat, proposed);
        }
      }
    }
  }
}
