using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Per-cell speed/cost field computation (spec §2.4–2.5) over a compact
  /// solve domain (spec §8.3), shared verbatim by the field-pass job and the
  /// oracle-parity tests (which drive it through an identity domain).
  /// </summary>
  public static class CCFieldOps
  {
    /// <summary>
    /// Compute the anisotropic speed and cost float4s for one DOMAIN cell.
    ///
    /// Into-cell rule (spec §2.1 — critical, easy to get wrong): for cell M
    /// in direction d, ALL density/velocity/discomfort/slope reads are taken
    /// from the neighbor cell the mover would enter — M + ENSWint[d], not M
    /// itself (repo xGlobalInto/yGlobalInto). This is the discrete analog of
    /// the paper's r·n_θ offset and is why a unit's own density contribution
    /// never obstructs its own motion.
    ///
    /// Domain semantics (spec §8.2/§8.6): the neighbor table encodes
    /// adjacency; a −1 entry means the into-cell is outside the domain
    /// (out of bounds, unwalkable, or beyond the pad) → speed f_speedMin and
    /// cost +∞, i.e. a wall at the domain edge. Global stamped data (ρ, v̄,
    /// g, ∇h) is read at the neighbor's GLOBAL index — dense global map,
    /// zero hashing (Decision D2 + §8.3 neighbor table).
    /// </summary>
    public static void ComputeCell(
      int local,
      in NativeArray<int> cells,
      in NativeArray<int4> neighborLocalIdx,
      in NativeArray<float> rho,
      in NativeArray<float2> vAve,
      in NativeArray<float> discomfort,
      in NativeArray<float2> dh,
      in CCConstants constants,
      float alpha,
      float beta,
      float gamma,
      out float4 f,
      out float4 cost
    )
    {
      var neighbors = neighborLocalIdx[local];
      f = default;
      cost = default;
      for (int d = 0; d < CCMath.NumDirections; d++) {
        int intoLocal = neighbors[d];
        bool valid = intoLocal >= 0;
        // identity domains used in parity tests include unwalkable cells;
        // preserve the repo's g ≥ 1 invalidity there (flood-fill domains
        // never contain them, so this check is free in production)
        int intoGlobal = valid ? cells[intoLocal] : 0;
        valid = valid && discomfort[intoGlobal] < 1f;

        float fd = valid
          ? CCMath.SpeedFieldPoint(rho[intoGlobal], dh[intoGlobal], vAve[intoGlobal], d, constants)
          : constants.f_speedMin;
        f[d] = fd;
        cost[d] = CCMath.CostFieldValue(fd, valid ? discomfort[intoGlobal] : 0f, valid, alpha, beta, gamma);
      }
    }
  }
}
