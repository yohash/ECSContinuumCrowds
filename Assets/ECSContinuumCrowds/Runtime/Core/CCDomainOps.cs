using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>Why a domain refresh fired (telemetry + doubled-pad handling).</summary>
  public enum DomainRefreshReason : byte
  {
    None = 0,
    NeverBuilt,
    GoalChanged,
    GroupMoved,
    GroupGrew,
    UnitEscaped,
    WalkabilityEdited,
    UnitStalled,
  }

  /// <summary>
  /// Solve-domain derivation and cache invalidation (spec §8, Decisions D5,
  /// D6) — static and Burst-compatible, shared by the domain job and the
  /// edit-mode tests.
  ///
  /// A domain is a transient, terrain-shaped working set: the compact list
  /// of global cells over which one group's f, C, φ and velocity are
  /// computed. Domains replace tiles entirely: because they follow walkable
  /// connectivity, a long narrow canyon is ONE domain (never split
  /// lengthwise); because they are per-group and read-only over the shared
  /// stamped fields, domains from different groups overlap freely with zero
  /// coordination.
  /// </summary>
  public static class CCDomainOps
  {
    /// <summary>
    /// BFS flood fill from the goal cells over 4-connected walkable cells
    /// (spec §8.2). A cell is admitted if it is (a) within the padded AABB
    /// (primary spatial bound) AND (b) within HorizonCells graph-distance of
    /// a seed (hard cap so pathological geometry can't explode the fill).
    ///
    /// Outputs: Cells (flat global indices, BFS order), GlobalToLocal
    /// (global flat → compact [0..N)), and the precomputed per-cell int4
    /// NeighborLocalIdx table (E,N,W,S local indices, −1 absent) so the
    /// field/eikonal/gradient hot loops do ZERO hashing (spec §8.3).
    ///
    /// Correctness note (spec §8.6): treating out-of-domain neighbors as
    /// infinite cost is equivalent to walls at the domain edge. This is safe
    /// iff the domain contains the true optimal paths of all member units —
    /// the pad + escape/stall/goal/centroid triggers jointly maintain this.
    /// </summary>
    public static void FloodFill(
      in GridIndexer gi,
      in NativeArray<byte> walkable,
      in NativeArray<int2> goalCells,
      int2 paddedMin,
      int2 paddedMax,
      int horizonCells,
      NativeList<int> cells,
      NativeParallelHashMap<int, int> globalToLocal,
      NativeList<int4> neighborLocalIdx
    )
    {
      cells.Clear();
      globalToLocal.Clear();
      neighborLocalIdx.Clear();

      paddedMin = math.clamp(paddedMin, int2.zero, new int2(gi.W - 1, gi.H - 1));
      paddedMax = math.clamp(paddedMax, int2.zero, new int2(gi.W - 1, gi.H - 1));

      // frontier queue: (flat, depth) pairs, job-local scratch
      var frontier = new NativeList<int2>(256, Allocator.Temp);
      int head = 0;

      // seed: all VALID goal cells (walkable, in the padded region). Note:
      // unlike the Phase-1 full-grid path, an impassable (g ≥ 1) goal cell
      // is simply not part of the domain — the repo's invalid-goal 0+C
      // radiation quirk only applies within a domain (parity tests exercise
      // it through identity domains).
      foreach (var gc in goalCells) {
        if (!gi.InBounds(gc) || math.any(gc < paddedMin) || math.any(gc > paddedMax)) {
          continue;
        }
        int flat = gi.Flat(gc);
        if (walkable[flat] == 0 || !globalToLocal.TryAdd(flat, cells.Length)) {
          continue;
        }
        cells.Add(flat);
        frontier.Add(new int2(flat, 0));
      }

      while (head < frontier.Length) {
        var cur = frontier[head++];
        if (cur.y >= horizonCells) {
          continue;
        }
        var c = gi.Coord(cur.x);
        for (int d = 0; d < CCMath.NumDirections; d++) {
          var n = c + CCMath.ENSWint(d);
          if (!gi.InBounds(n) || math.any(n < paddedMin) || math.any(n > paddedMax)) {
            continue;
          }
          int nFlat = gi.Flat(n);
          if (walkable[nFlat] == 0 || !globalToLocal.TryAdd(nFlat, cells.Length)) {
            continue;
          }
          cells.Add(nFlat);
          frontier.Add(new int2(nFlat, cur.y + 1));
        }
      }
      frontier.Dispose();

      // neighbor-index table: converts all hot-loop neighbor access to
      // plain array indexing (strongly recommended by spec §8.3)
      neighborLocalIdx.Resize(cells.Length, NativeArrayOptions.UninitializedMemory);
      for (int i = 0; i < cells.Length; i++) {
        var c = gi.Coord(cells[i]);
        var entry = new int4(-1);
        for (int d = 0; d < CCMath.NumDirections; d++) {
          var n = c + CCMath.ENSWint(d);
          if (gi.InBounds(n) && globalToLocal.TryGetValue(gi.Flat(n), out int local)) {
            entry[d] = local;
          }
        }
        neighborLocalIdx[i] = entry;
      }
    }

    /// <summary>
    /// Cache invalidation triggers (spec §8.4, Decision D6), evaluated per
    /// solve tick — cheap comparisons only. The PadCells inflation IS the
    /// hysteresis: the domain is derived larger than strictly needed, so the
    /// group must drift ~half the pad before the centroid trigger fires.
    /// </summary>
    public static DomainRefreshReason EvaluateTriggers(
      bool valid,
      float2 cachedGoalCentroid,
      int cachedGoalCount,
      float2 goalCentroid,
      int goalCount,
      float2 cachedGroupCentroid,
      float cachedGroupRadius,
      float2 groupCentroid,
      float groupRadius,
      int cachedWalkabilityVersion,
      int walkabilityVersion,
      bool unitEscaped,
      bool unitStalled,
      float padCells
    )
    {
      if (!valid) return DomainRefreshReason.NeverBuilt;
      // hard triggers first (immediate refresh)
      if (unitStalled) return DomainRefreshReason.UnitStalled;
      if (unitEscaped) return DomainRefreshReason.UnitEscaped;
      if (cachedWalkabilityVersion != walkabilityVersion) {
        return DomainRefreshReason.WalkabilityEdited;
      }
      // goal changed: buffer length change or centroid moved ≥ 1 cell
      if (goalCount != cachedGoalCount
        || math.distance(goalCentroid, cachedGoalCentroid) >= 1f) {
        return DomainRefreshReason.GoalChanged;
      }
      // hysteresis pair (spec table): centroid drift or radius growth > pad/2
      if (math.distance(groupCentroid, cachedGroupCentroid) > padCells * 0.5f) {
        return DomainRefreshReason.GroupMoved;
      }
      if (groupRadius > cachedGroupRadius + padCells * 0.5f) {
        return DomainRefreshReason.GroupGrew;
      }
      return DomainRefreshReason.None;
    }

    /// <summary>
    /// Padded fill bound (spec §8.2): AABB(goalCells ∪ unitExtent) inflated
    /// by PadCells (doubled after a stall, §8.6).
    /// </summary>
    public static void PaddedBounds(
      float2 unitMin, float2 unitMax, int2 goalMin, int2 goalMax, float padCells,
      out int2 paddedMin, out int2 paddedMax)
    {
      var lo = math.min((int2)math.floor(unitMin), goalMin);
      var hi = math.max((int2)math.floor(unitMax), goalMax);
      int pad = (int)math.ceil(padCells);
      paddedMin = lo - pad;
      paddedMax = hi + pad;
    }
  }
}
