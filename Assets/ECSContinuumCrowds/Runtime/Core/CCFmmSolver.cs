using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>Cell state flags written by the FMM solver (compact, per domain cell).</summary>
  public static class CCCellState
  {
    public const byte Accepted = 1;
    public const byte Goal = 2;
  }

  /// <summary>
  /// Fast Marching Method eikonal solver (spec §9) over a compact solve
  /// domain (spec §8.3), reproducing yohash/ContinuumCrowds EikonalSolver
  /// semantics exactly — this is the parity-tested core, driven through an
  /// identity domain by the oracle tests. Static and Burst-compatible so the
  /// FMM job and the edit-mode tests run the identical code path.
  ///
  /// All indices below are DOMAIN-LOCAL; adjacency comes from the
  /// precomputed NeighborLocalIdx table (E,N,W,S, −1 = outside the domain =
  /// infinite cost, per spec §8.2). Walkability (g ≥ 1) is still checked per
  /// cell so identity domains (which include walls, for repo parity) behave
  /// exactly like the Phase-1 full-grid solver.
  ///
  /// Repo semantics preserved (verified against EikonalSolver.cs):
  /// - Goal cells are seeded INTO THE QUEUE at priority 0, not pre-accepted.
  /// - φ is set to 0 only for VALID goal cells (g &lt; 1); invalid goal cells
  ///   in the domain are still marked goal and enqueued, and radiate 0 + C
  ///   to neighbors via the goal-set read below. (Flood-fill domains never
  ///   contain invalid cells; identity/parity domains do.)
  /// - Update TARGETS must be in-domain, g &lt; 1, NOT accepted, NOT goal.
  /// - φ READS for phi_m must be in-domain, g &lt; 1, and NOT accepted —
  ///   previously-accepted cells read as +∞ (repo quirk). Propagation works
  ///   because a cell is marked accepted AFTER its neighbors are updated.
  /// - A neighbor in the goal set that fails the read test contributes 0 + C
  ///   (the repo's off-tile-goal guard).
  /// - Exact decrease-key replaces the repo's duplicate-enqueueing managed
  ///   queue (final φ agrees; parity-tested).
  /// </summary>
  public static class CCFmmSolver
  {
    /// <summary>
    /// Solve φ over the domain. All arrays are domain-compact with
    /// Length ≥ cellCount (persistent per-group scratch, spec §9.5);
    /// goalCells are GLOBAL grid coords, mapped through globalToLocal.
    /// </summary>
    public static void Solve(
      int cellCount,
      in NativeArray<int> cells,
      in NativeArray<int4> neighborLocalIdx,
      in NativeParallelHashMap<int, int> globalToLocal,
      in GridIndexer gi,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
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
      // Reset clears Pos over the array's full capacity; bound the loop to
      // the live domain
      heap.Count = 0;
      for (int i = 0; i < cellCount; i++) {
        phi[i] = float.PositiveInfinity;
        state[i] = 0;
        heapPos[i] = -1;
      }

      // seed: goal cells at priority 0 (repo: markGoal + Enqueue; φ = 0 only
      // when the point is valid)
      for (int i = 0; i < goalCells.Length; i++) {
        var gc = goalCells[i];
        if (!gi.InBounds(gc) || !globalToLocal.TryGetValue(gi.Flat(gc), out int local)) {
          continue; // goal outside the domain (e.g. unwalkable goal cell excluded by the fill)
        }
        state[local] |= CCCellState.Goal;
        if (discomfortGlobal[cells[local]] < 1f) {
          phi[local] = 0f;
        }
        if (!heap.Contains(local)) {
          heap.Push(local, 0f);
        }
      }

      // the eikonal update loop
      while (heap.Count > 0) {
        int current = heap.PopMin();
        UpdateNeighbors(
          current, cells, neighborLocalIdx, C, discomfortGlobal,
          phi, state, ref heap, maxWeight, minWeight);
        state[current] |= CCCellState.Accepted; // AFTER updating neighbors
      }
    }

    /// <summary>Repo EikonalUpdateFormula: propose new φ for each valid neighbor of a popped cell.</summary>
    private static void UpdateNeighbors(
      int current,
      in NativeArray<int> cells,
      in NativeArray<int4> neighborLocalIdx,
      in NativeArray<float4> C,
      in NativeArray<float> discomfortGlobal,
      NativeArray<float> phi,
      NativeArray<byte> state,
      ref IndexedMinHeap heap,
      float maxWeight,
      float minWeight
    )
    {
      var currentNeighbors = neighborLocalIdx[current];
      for (int d = 0; d < CCMath.NumDirections; d++) {
        int nb = currentNeighbors[d];
        if (nb < 0) {
          continue; // outside the domain — wall at the domain edge (§8.6)
        }
        // valid-as-target: not goal, not accepted, walkable
        if ((state[nb] & (CCCellState.Accepted | CCCellState.Goal)) != 0) {
          continue;
        }
        if (discomfortGlobal[cells[nb]] >= 1f) {
          continue;
        }

        // assemble phi_m from nb's own four neighbors; C indexed at nb (the
        // cell being updated) — into-cell convention already baked into C
        var Cnb = C[nb];
        var nbNeighbors = neighborLocalIdx[nb];
        var phi_m = new float4(float.PositiveInfinity);
        for (int dd = 0; dd < CCMath.NumDirections; dd++) {
          int nn = nbNeighbors[dd];
          if (nn < 0) {
            continue; // out-of-domain neighbor = ∞ (spec §8.2)
          }
          if (discomfortGlobal[cells[nn]] < 1f && (state[nn] & CCCellState.Accepted) == 0) {
            // valid-to-read (accepted cells excluded — repo quirk, see class doc)
            phi_m[dd] = phi[nn] + Cnb[dd];
          } else if ((state[nn] & CCCellState.Goal) != 0) {
            // goal cells have φ of 0 (repo off-tile-goal guard)
            phi_m[dd] = 0f + Cnb[dd];
          }
        }

        float proposed = CCMath.EikonalSolve(phi_m, Cnb, maxWeight, minWeight);
        if (proposed < phi[nb]) {
          phi[nb] = proposed;
          heap.PushOrUpdate(nb, proposed);
        }
      }
    }
  }
}
