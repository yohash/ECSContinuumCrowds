using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Unit snapshot payload for the stamping hash (spec §3.5): written once at
  /// hash build, read by the gather pass. Position is grid-space; Velocity is
  /// world-units/second (what the momentum term and the flow-speed comparison
  /// need). FootprintSize is carried for the future radial-extension path —
  /// default units use the pure 2×2 splat (spec §6.4).
  /// </summary>
  public struct UnitStampData
  {
    public float2 Position;
    public float2 Velocity;
    public float Mass;
    public float FootprintSize;
  }

  /// <summary>
  /// Stamping math (spec §6–7), shared verbatim by the gather job and the
  /// edit-mode tests. Two formulations live here:
  ///
  /// - GATHER (shipping, Decision D3): each active cell pulls contributions
  ///   from units in its 9 surrounding hash buckets. Every cell is written by
  ///   exactly one thread — no races, no atomics, no reduction.
  /// - SCATTER reference (test oracle): the naive per-unit deposit loop
  ///   (static splat + discrete predictive ghost chain). The Phase-1 runtime
  ///   scatter system was retired in favor of gather; its semantics survive
  ///   here as the brute-force reference the gather form is validated
  ///   against (spec §7.2, §15 P2, risk register).
  /// </summary>
  public static class CCStampOps
  {
    /// <summary>Half-extent (in cells, from a cell center) of the 2×2 splat support.</summary>
    public const float StaticReachCells = 1.5f;

    // -----------------------------------------------------------------
    //  Bucketing (spec §6.3)
    // -----------------------------------------------------------------

    /// <summary>
    /// Bucket edge length in cells: BucketSize = ceil(R_max) where R_max is
    /// the maximum footprint reach including the predictive extension, so any
    /// cell's contributors lie within its own bucket and the 8 neighbors — a
    /// fixed 9-bucket query (spec §6.3).
    ///
    /// The predictive extent is CAPPED (spec §6.3: clamp extrapolation
    /// distance rather than growing buckets unboundedly) by
    /// v_predictiveDistanceCapCells — the cap is documented next to the
    /// constants; with defaults reach = 1.5 + 0 + min(8, 20·1/1) = 9.5 → 10.
    /// </summary>
    public static int BucketCells(in CCConstants c, float cellSize)
    {
      float predictiveReach = math.min(
        c.v_predictiveDistanceCapCells,
        c.f_speedMax * c.v_predictiveSeconds / cellSize);
      return (int)math.ceil(
        StaticReachCells + c.u_unitRadialFalloff + math.max(predictiveReach, 0f));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 BucketDims(in GridIndexer gi, int bucketCells)
      => new int2(
        (gi.W + bucketCells - 1) / bucketCells,
        (gi.H + bucketCells - 1) / bucketCells);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 BucketOf(float2 gridPos, int bucketCells)
      => (int2)math.floor(gridPos / bucketCells);

    /// <summary>Buckets form a small dense grid, so the key is a plain flat index — no hashing.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int BucketKey(int2 bucket, int2 dims) => bucket.y * dims.x + bucket.x;

    // -----------------------------------------------------------------
    //  Predictive velocity — gather-side closed form (Decision D4, spec §7)
    // -----------------------------------------------------------------
    // ⚠ Rationale (spec §7.1, record verbatim in code docs):
    // The paper's §3.3 "predictive discomfort" deposits discomfort ahead of
    // each moving unit so others avoid the space it is about to occupy.
    // Rejected: a unit with projected discomfort in front of itself will
    // path-adjust to avoid its own projection; the projection then moves
    // with the new heading, and the unit adjusts again — a feedback
    // oscillation. Predictive VELOCITY (the repo's approach) instead
    // extrapolates the unit's density/velocity footprint forward along its
    // velocity. Because the speed field's flow term (§2.4 step 4) is
    // directional — moving with the local average velocity is not penalized
    // — a unit is never obstructed by its own forward projection when
    // continuing straight, so no self-avoidance loop exists. Additionally,
    // forward-projected velocity strengthens v̄ along movement corridors,
    // which reinforces fast lanes and slow lanes: followers see high
    // same-direction flow speed (cheap), opposers see it as resistance
    // (expensive).

    /// <summary>
    /// Gather-side predictive weight (spec §7.2): instead of the unit
    /// depositing K ghost stamps (scatter thinking), the cell projects itself
    /// onto the unit's extrapolation segment
    /// [Position, Position + Velocity·v_predictiveSeconds] (length capped),
    /// and — if within kernel support — contributes
    /// scale(t*) · SplatWeight(P(t*), cell, λ), the spec's closed-form
    /// approximation of the ghost-chain integral. Validated against the
    /// brute-force scatter reference in PredictiveStampingTests.
    /// </summary>
    public static float PredictiveWeight(
      in UnitStampData u, int2 cell, in CCConstants c, float cellSize)
    {
      float speed = math.length(u.Velocity);
      if (speed < c.v_dynamicFootprintThreshold) {
        return 0f; // static footprint only below the dynamic threshold
      }

      // extrapolation segment in grid space (direction is scale-invariant)
      var dir = u.Velocity / speed;
      float lengthCells = math.min(
        speed * c.v_predictiveSeconds / cellSize,
        c.v_predictiveDistanceCapCells);
      if (lengthCells <= 1e-4f) {
        return 0f;
      }

      // project the cell center onto the segment; ghosts live on the OPEN
      // interval t ∈ (0, T] (the static stamp already covers t = 0)
      float t = math.clamp(
        math.dot(CCMath.CellCenter(cell) - u.Position, dir), 0f, lengthCells);
      if (t <= 1e-4f) {
        return 0f;
      }

      var ghost = u.Position + dir * t;
      float w = CCMath.SplatWeight(ghost, cell, c.lambda);
      if (w <= 0f) {
        return 0f;
      }

      // scale fades with lookahead time (repo semantics)
      float tTime = t * cellSize / speed;
      float scale = math.lerp(
        c.v_scaleMax, c.v_scaleMin, math.saturate(tTime / c.v_predictiveSeconds));
      return scale * w;
    }

    // -----------------------------------------------------------------
    //  Gather inner loop (spec §6.5)
    // -----------------------------------------------------------------

    /// <summary>
    /// Accumulate ρ and momentum for one cell from the stamping hash.
    /// Ghost (predictive) contributions add to BOTH ρ and momentum with the
    /// unit's full velocity, exactly like the real stamp — that is what
    /// projects the flow field forward, and it is the lane-formation booster
    /// (spec §7.2). The into-cell speed evaluation already prevents static
    /// self-obstruction; the directional flow clamp prevents it for the
    /// predictive stamp.
    /// </summary>
    public static void GatherCell(
      int2 cell,
      in NativeParallelMultiHashMap<int, UnitStampData> map,
      int bucketCells,
      int2 bucketDims,
      in CCConstants c,
      float cellSize,
      out float rho,
      out float2 momentum
    )
    {
      rho = 0f;
      momentum = float2.zero;
      var myBucket = BucketOf(CCMath.CellCenter(cell), bucketCells);

      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          var b = myBucket + new int2(dx, dy);
          if (math.any(b < 0) || math.any(b >= bucketDims)) {
            continue;
          }
          if (!map.TryGetFirstValue(BucketKey(b, bucketDims), out var u, out var it)) {
            continue;
          }
          do {
            float w = CCMath.SplatWeight(u.Position, cell, c.lambda);
            float wp = PredictiveWeight(u, cell, c, cellSize);
            float wt = (w + wp) * u.Mass; // ⚠ DIVERGENCE (repo, kept): mass scaling
            rho += wt;
            momentum += wt * u.Velocity;
          } while (map.TryGetNextValue(out u, ref it));
        }
      }
    }

    // -----------------------------------------------------------------
    //  Active-cell derivation (spec §6.2), shared by job and tests
    // -----------------------------------------------------------------

    /// <summary>Active buckets = occupied buckets dilated by one ring (bucket ≥ gather reach).</summary>
    public static void MarkActiveBuckets(
      in NativeArray<byte> occupied, NativeArray<byte> active, int2 dims)
    {
      for (int i = 0; i < active.Length; i++) {
        active[i] = 0;
      }
      for (int by = 0; by < dims.y; by++) {
        for (int bx = 0; bx < dims.x; bx++) {
          if (occupied[by * dims.x + bx] == 0) {
            continue;
          }
          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              var b = new int2(bx + dx, by + dy);
              if (math.all(b >= 0) && math.all(b < dims)) {
                active[BucketKey(b, dims)] = 1;
              }
            }
          }
        }
      }
    }

    /// <summary>Emit the flat cell indices covered by active buckets (clamped to the grid).</summary>
    public static void EmitActiveCells(
      in NativeArray<byte> active, int2 dims, int bucketCells, in GridIndexer gi,
      NativeList<int> cells)
    {
      cells.Clear();
      for (int by = 0; by < dims.y; by++) {
        for (int bx = 0; bx < dims.x; bx++) {
          if (active[by * dims.x + bx] == 0) {
            continue;
          }
          int yEnd = math.min((by + 1) * bucketCells, gi.H);
          int xEnd = math.min((bx + 1) * bucketCells, gi.W);
          for (int y = by * bucketCells; y < yEnd; y++) {
            for (int x = bx * bucketCells; x < xEnd; x++) {
              cells.Add(gi.Flat(x, y));
            }
          }
        }
      }
    }

    // -----------------------------------------------------------------
    //  Brute-force scatter reference (tests; spec §7.2 validation)
    // -----------------------------------------------------------------

    /// <summary>
    /// Reference deposit of one unit's STATIC 2×2 splat (the retired Phase-1
    /// scatter path, preserved as the gather correctness oracle).
    /// </summary>
    public static void ScatterStatic(
      in UnitStampData u, in GridIndexer gi, float lambda,
      NativeArray<float> rho, NativeArray<float2> momentum)
    {
      var baseCell = CCMath.SplatBaseCell(u.Position);
      var w = CCMath.SplatWeights(u.Position, lambda); // (A,B,C,D) = LL,LR,UR,UL
      DepositReference(baseCell, w.x, u, gi, rho, momentum);
      DepositReference(baseCell + new int2(1, 0), w.y, u, gi, rho, momentum);
      DepositReference(baseCell + new int2(1, 1), w.z, u, gi, rho, momentum);
      DepositReference(baseCell + new int2(0, 1), w.w, u, gi, rho, momentum);
    }

    /// <summary>
    /// Reference discrete ghost chain (spec §7.2 scatter form): K ghost
    /// stamps ≤ 1 cell apart along the (capped) extrapolated path, each
    /// depositing scale(t_k)·SplatWeight·Mass into ρ and ρ·v. K clamped to
    /// MaxGhosts (spec: e.g. 8) — gaps in the chain create striped density.
    /// </summary>
    public const int MaxGhosts = 8;

    public static void ScatterPredictiveGhosts(
      in UnitStampData u, in GridIndexer gi, in CCConstants c, float cellSize,
      NativeArray<float> rho, NativeArray<float2> momentum)
    {
      float speed = math.length(u.Velocity);
      if (speed < c.v_dynamicFootprintThreshold) {
        return;
      }
      var dir = u.Velocity / speed;
      float lengthCells = math.min(
        speed * c.v_predictiveSeconds / cellSize, c.v_predictiveDistanceCapCells);
      if (lengthCells <= 1e-4f) {
        return;
      }
      int k = math.clamp((int)math.ceil(lengthCells), 1, MaxGhosts);
      for (int i = 1; i <= k; i++) {
        float t = lengthCells * i / k; // cells along the path, t ∈ (0, L]
        var ghostPos = u.Position + dir * t;
        float tTime = t * cellSize / speed;
        float scale = math.lerp(
          c.v_scaleMax, c.v_scaleMin, math.saturate(tTime / c.v_predictiveSeconds));

        var baseCell = CCMath.SplatBaseCell(ghostPos);
        var w = CCMath.SplatWeights(ghostPos, c.lambda) * scale;
        DepositReference(baseCell, w.x, u, gi, rho, momentum);
        DepositReference(baseCell + new int2(1, 0), w.y, u, gi, rho, momentum);
        DepositReference(baseCell + new int2(1, 1), w.z, u, gi, rho, momentum);
        DepositReference(baseCell + new int2(0, 1), w.w, u, gi, rho, momentum);
      }
    }

    private static void DepositReference(
      int2 cell, float w, in UnitStampData u, in GridIndexer gi,
      NativeArray<float> rho, NativeArray<float2> momentum)
    {
      if (w <= 0f || !gi.InBounds(cell)) {
        return;
      }
      int i = gi.Flat(cell);
      float wm = w * u.Mass;
      rho[i] += wm;
      momentum[i] += wm * u.Velocity;
    }
  }
}
