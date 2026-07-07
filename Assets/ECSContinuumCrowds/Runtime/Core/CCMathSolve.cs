using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Solver math shared by the field pass, FMM (and FIM in Phase 4), the
  /// velocity-derivation pass, advection, and the parity test suite.
  /// </summary>
  public static partial class CCMath
  {
    // *************************************************************************
    //    Speed field (spec §2.4; repo computeSpeedFieldPoint)
    // *************************************************************************

    /// <summary>
    /// Topographical speed for a direction (spec §2.4 step 3): downhill in
    /// the direction of travel → faster; uphill → slower.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float TopographicalSpeed(float2 dhInto, float2 dir, in CCConstants c)
    {
      // dot with the direction extracts the slope along travel with proper
      // sign (repo computeTopographicalSpeed)
      float slope = math.dot(dhInto, dir);
      return c.f_speedMax
        + (slope - c.f_slopeMin) / (c.f_slopeMax - c.f_slopeMin)
        * (c.f_speedMin - c.f_speedMax);
    }

    /// <summary>
    /// Flow speed for a direction (spec §2.4 step 4): v̄ of the into-cell
    /// dotted with the direction of travel.
    ///
    /// The max(0, ·) clamp is MANDATORY (repo changelog v0.2.7, quoting the
    /// paper): "the flow speed is clamped to be nonnegative, implying that
    /// the crowd can slow people down, but never carry them backwards."
    /// This clamp + the directional dot product is the term that produces
    /// lane formation — do not "optimize" it away.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float FlowSpeed(float2 vAveInto, float2 dir)
      => math.max(0f, math.dot(vAveInto, dir));

    /// <summary>
    /// Anisotropic speed for one direction d of a cell, from the INTO-CELL's
    /// ρ/∇h/v̄ (spec §2.4 steps 2–6). Caller is responsible for the into-cell
    /// rule (§2.1): all three inputs must be sampled at M + ENSWint[d], and
    /// invalid into-cells short-circuit to f_speedMin before calling this.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float SpeedFieldPoint(
      float rhoInto, float2 dhInto, float2 vAveInto, int d, in CCConstants c)
    {
      var dir = ENSW(d);
      float ff;
      if (rhoInto < c.f_rhoMin) {
        // low density → topographical speed
        ff = TopographicalSpeed(dhInto, dir, c);
      } else if (rhoInto > c.f_rhoMax) {
        // high density → flow speed
        ff = FlowSpeed(vAveInto, dir);
      } else {
        // medium density → linear interpolation (repo ordering preserved)
        float ft = TopographicalSpeed(dhInto, dir, c);
        float fv = FlowSpeed(vAveInto, dir);
        ff = ft + (fv - ft) * (rhoInto - c.f_rhoMin) / (c.f_rhoMax - c.f_rhoMin);
      }
      return math.clamp(ff, c.f_speedMin, c.f_speedMax);
    }

    // *************************************************************************
    //    Cost field (spec §2.5; repo computeCostFieldValue)
    // *************************************************************************

    /// <summary>
    /// Unit cost for one direction: C = α + β/f + γ·g'/f (algebraically the
    /// paper's C = (αf + β + γg)/f). Zero speed or an invalid into-cell → +∞.
    ///
    /// ⚠ DIVERGENCE (repo, kept): the paper leaves g unbounded; the repo
    /// clamps g to [0,1] and treats g ≥ 1 as absolutely impassable, folding
    /// boundaries into the discomfort field. Cleaner than a separate mask.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float CostFieldValue(
      float f, float gInto, bool intoValid, float alpha, float beta, float gamma)
    {
      if (f == 0f || !intoValid) {
        return float.PositiveInfinity;
      }
      float g = math.clamp(gInto, 0f, 1f);
      return alpha + beta / f + gamma * g / f;
    }

    // *************************************************************************
    //    Eikonal update (spec §9.3–9.4; repo EikonalUpdateFormula core)
    // *************************************************************************

    /// <summary>
    /// The finite-difference eikonal update shared by FMM (and FIM, Phase 4):
    /// one function, two drivers (spec §16.4).
    ///
    /// Inputs: phi_m[dd] = φ(neighbor in direction dd) + C[cell][dd], with
    /// +∞ for unreadable neighbors, and the cell's own anisotropic cost C.
    /// The cost is indexed at the cell being updated with the direction
    /// toward each of its neighbors — the into-cell convention is already
    /// baked into C (spec §2.5); follow the repo exactly, do not re-derive
    /// which cell's C to read.
    ///
    /// Reference: the paper's quadratic is Eq. 11,
    ///   (φ−φ_mx)²/C_mx² + (φ−φ_my)²/C_my² = 1, taking the larger root.
    /// ⚠ DIVERGENCE (repo, kept): the repo's discriminant test and root
    /// handling below differ from the paper; we ship the repo's formulation.
    /// </summary>
    public static float EikonalSolve(float4 phi_m, float4 C, float maxWeight, float minWeight)
    {
      // select the cheapest arrival per axis (E/W then N/S); on a tie the
      // repo picks the first component — preserve that
      float phi_mx = math.min(phi_m.x, phi_m.z);
      float phi_my = math.min(phi_m.y, phi_m.w);
      float C_mx = phi_mx == phi_m.x ? C.x : C.z;
      float C_my = phi_my == phi_m.y ? C.y : C.w;

      bool xValid = phi_mx < float.PositiveInfinity;
      bool yValid = phi_my < float.PositiveInfinity;

      // ⚠ NOTE (spec §9.3 degenerate case — deliberate deviation from raw
      // repo float behavior): if one axis has both neighbors infinite, drop
      // that dimension and use the 1-D solution on the other axis. The repo
      // reaches the same 1-D answer through its discriminant test whenever
      // the chosen ∞-direction still has FINITE cost (e.g. an accepted
      // neighbor), but when the axis is fully walled (both costs also ∞ —
      // a 1-wide corridor) the repo's arithmetic produces NaN and silently
      // skips the update, leaving corridor cells at φ = ∞ forever. The spec
      // mandates the 1-D solution; parity tests document this divergence
      // explicitly (EikonalParityTests.OneWideCorridor_DivergesFromOracle).
      if (!xValid & !yValid) {
        return float.PositiveInfinity;
      }
      if (!yValid) {
        return phi_mx + C_mx;
      }
      if (!xValid) {
        return phi_my + C_my;
      }

      float C_mx_Sq = C_mx * C_mx;
      float C_my_Sq = C_my * C_my;
      float phi_mDiff_Sq = (phi_mx - phi_my) * (phi_mx - phi_my);

      // ⚠ NOTE (repo-specific, preserve as-is): this discriminant test is
      // not a form found in the paper.
      float valTest = C_mx_Sq + C_my_Sq - 1f / (C_mx_Sq * C_my_Sq);

      if (phi_mDiff_Sq > valTest) {
        // simplified 1-D solution: min axis + its cost
        float phi_min = math.min(phi_mx, phi_my);
        float cost_min = phi_min == phi_mx ? C_mx : C_my;
        return phi_min + cost_min;
      }

      // solve the quadratic
      float radical = math.sqrt(C_mx_Sq * C_my_Sq * (C_mx_Sq + C_my_Sq - phi_mDiff_Sq));

      // guard rail (spec §9.4): a NaN radical shouldn't occur past the
      // discriminant test, but float drift happens — fall back to 1-D.
      // Never let NaN into φ; a single NaN poisons the gradient silently.
      if (float.IsNaN(radical)) {
        float phi_min = math.min(phi_mx, phi_my);
        float cost_min = phi_min == phi_mx ? C_mx : C_my;
        return phi_min + cost_min;
      }

      float denom = C_mx_Sq + C_my_Sq;
      float soln1 = (C_my_Sq * phi_mx + C_mx_Sq * phi_my + radical) / denom;
      float soln2 = (C_my_Sq * phi_mx + C_mx_Sq * phi_my - radical) / denom;

      // Root selection experiments (from yohash/ContinuumCrowds — preserve
      // this record, Decision D9):
      //   max            → paper's choice; prefers diagonals (visible
      //                    diagonal bias in paths)
      //   min            → prefers cardinals
      //   mean           → better mix but still prefers diagonals
      //   geometric mean → effectively identical to mean
      //   WEIGHTED MEAN  → best compromise found; shipped.
      //                    weights: maxWeight=2.5, minWeight=1.0
      float maxRoot = math.max(soln1, soln2);
      float minRoot = math.min(soln1, soln2);
      return (maxRoot * maxWeight + minRoot * minWeight) / (maxWeight + minWeight);
    }

    // *************************************************************************
    //    Potential gradient (spec §11, Decision D10)
    // *************************************************************************

    /// <summary>
    /// One axis of the shipping (repo) gradient: both neighbors ∞ → 0; one
    /// ∞ → ±1 one-sided fallback (repo Mathf.Sign semantics: sign(0) = +1);
    /// else central difference over the span. Edge cells pass a clamped
    /// span of 1 (one-sided), matching the repo's enumerated edge cases.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float GradientAxisCentral(float phiLo, float phiHi, int span)
    {
      bool loInf = float.IsInfinity(phiLo);
      bool hiInf = float.IsInfinity(phiHi);
      if (loInf && hiInf) {
        return 0f;
      }
      if (loInf || hiInf) {
        return phiHi - phiLo >= 0f ? 1f : -1f;
      }
      return span > 0 ? (phiHi - phiLo) / span : 0f;
    }

    /// <summary>
    /// Shipping gradient (Decision D10, GradientScheme.CentralRepo) over a
    /// compact domain: central difference with infinity fallback, normalized
    /// (zero-safe). Neighbor access goes through the domain's
    /// NeighborLocalIdx table: a −1 entry (out of domain — wall, unwalkable,
    /// or beyond the pad) reads as +∞ and takes the ±1 one-sided fallback,
    /// while GRID-edge cells reproduce the repo's clamped one-sided
    /// differences exactly (the repo's enumerated edge cases; parity-tested
    /// through identity domains). See GradientScheme docs for the recorded
    /// trade-off vs the paper's upwind scheme.
    /// </summary>
    public static float2 PotentialGradientCentral(
      in NativeArray<float> phi, int local, int2 cellCoord, int4 neighbors, in GridIndexer gi)
    {
      float pc = phi[local];

      bool loEdge = cellCoord.x == 0;
      bool hiEdge = cellCoord.x == gi.W - 1;
      float lo = loEdge ? pc : Read(phi, neighbors[DirW]);
      float hi = hiEdge ? pc : Read(phi, neighbors[DirE]);
      float dx = GradientAxisCentral(lo, hi, loEdge || hiEdge ? 1 : 2);

      loEdge = cellCoord.y == 0;
      hiEdge = cellCoord.y == gi.H - 1;
      lo = loEdge ? pc : Read(phi, neighbors[DirS]);
      hi = hiEdge ? pc : Read(phi, neighbors[DirN]);
      float dy = GradientAxisCentral(lo, hi, loEdge || hiEdge ? 1 : 2);

      return math.normalizesafe(new float2(dx, dy));
    }

    /// <summary>
    /// Reference gradient (Decision D10, GradientScheme.UpwindPaper) over a
    /// compact domain: per axis, difference φ against the upwind (lower-φ)
    /// neighbor — the same neighbor the eikonal update selected. Consistent
    /// with the discrete solution's characteristics; decisively picks a side
    /// at shocklines (direction quantization, hard flips across shockline
    /// cells). Debug / reference use; central-repo is the shipping path.
    /// </summary>
    public static float2 PotentialGradientUpwind(
      in NativeArray<float> phi, int local, int4 neighbors)
    {
      float pc = phi[local];
      float dx = GradientAxisUpwind(pc, Read(phi, neighbors[DirW]), Read(phi, neighbors[DirE]));
      float dy = GradientAxisUpwind(pc, Read(phi, neighbors[DirS]), Read(phi, neighbors[DirN]));
      return math.normalizesafe(new float2(dx, dy));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float Read(in NativeArray<float> phi, int local)
      => local >= 0 ? phi[local] : float.PositiveInfinity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float GradientAxisUpwind(float pc, float phiLo, float phiHi)
    {
      if (float.IsInfinity(pc)) {
        // unreached cell: same fallback family as the central scheme so the
        // fringe still flows toward known potential
        return GradientAxisCentral(phiLo, phiHi, 2);
      }
      float best = math.min(phiLo, phiHi);
      if (float.IsInfinity(best) || best >= pc) {
        return 0f; // local minimum (goal) or no informed neighbor
      }
      // descend toward the lower neighbor: gradient points up-φ
      return phiLo <= phiHi ? pc - phiLo : phiHi - pc;
    }

    // *************************************************************************
    //    Velocity (spec §2.7; repo calculateVelocityField)
    // *************************************************************************

    /// <summary>
    /// v = −f(direction faces) · ∇φ̂: each component scaled by the
    /// direction-appropriate face of the anisotropic speed field
    /// (f = ENSW float4: x=E, y=N, z=W, w=S).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 VelocityFromGradient(float2 dPhi, float4 f)
    {
      return new float2(
        dPhi.x > 0f ? -f.z * dPhi.x : -f.x * dPhi.x,  // moving −x: W face, else E
        dPhi.y > 0f ? -f.w * dPhi.y : -f.y * dPhi.y); // moving −y: S face, else N
    }

    // *************************************************************************
    //    Bilinear velocity sampling (spec §13.1)
    // *************************************************************************

    /// <summary>
    /// Bilinear sample of a snapshot velocity buffer at a grid-space
    /// position (spec §13.1 + §12.2): the buffer is domain-compact and is
    /// interpreted through ITS OWN full-grid localIdxLookup snapshot —
    /// corners outside the grid or outside the snapshot domain (lookup −1)
    /// get weight 0 and the remaining weights are renormalized, so velocity
    /// fades gracefully at the padded fringe rather than snapping to zero.
    /// </summary>
    public static float2 BilinearSampleVelocity(
      in NativeArray<float2> velocity,
      in NativeArray<int> localIdxLookup,
      in GridIndexer gi,
      float2 pos)
    {
      var baseCell = SplatBaseCell(pos);
      var d = SplatDelta(pos);
      float2 acc = float2.zero;
      float wSum = 0f;
      for (int dy = 0; dy <= 1; dy++) {
        for (int dx = 0; dx <= 1; dx++) {
          var cell = baseCell + new int2(dx, dy);
          if (!gi.InBounds(cell)) {
            continue;
          }
          int local = localIdxLookup[gi.Flat(cell)];
          if (local < 0) {
            continue; // outside this snapshot's domain
          }
          float w = (dx == 0 ? 1f - d.x : d.x) * (dy == 0 ? 1f - d.y : d.y);
          acc += velocity[local] * w;
          wSum += w;
        }
      }
      return wSum > 1e-6f ? acc / wSum : float2.zero;
    }
  }
}
