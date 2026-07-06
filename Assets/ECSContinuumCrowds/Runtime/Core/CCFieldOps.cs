using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Per-cell speed/cost field computation (spec §2.4–2.5), shared verbatim
  /// by the field-pass job and the oracle-parity tests.
  /// </summary>
  public static class CCFieldOps
  {
    /// <summary>
    /// Compute the anisotropic speed and cost float4s for one cell.
    ///
    /// Into-cell rule (spec §2.1 — critical, easy to get wrong): for cell M
    /// in direction d, ALL density/velocity/discomfort/slope reads are taken
    /// from the neighbor cell the mover would enter — M + ENSWint[d], not M
    /// itself (repo xGlobalInto/yGlobalInto). This is the discrete analog of
    /// the paper's r·n_θ offset and is why a unit's own density contribution
    /// never obstructs its own motion.
    /// </summary>
    public static void ComputeCell(
      int flat,
      in GridIndexer gi,
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
      var m = gi.Coord(flat);
      f = default;
      cost = default;
      for (int d = 0; d < CCMath.NumDirections; d++) {
        var into = m + CCMath.ENSWint(d);
        bool valid = gi.InBounds(into);
        int intoFlat = valid ? gi.Flat(into) : 0;
        valid = valid && discomfort[intoFlat] < 1f;

        // if the global "into" is not valid, speed is f_speedMin (repo)
        float fd = valid
          ? CCMath.SpeedFieldPoint(rho[intoFlat], dh[intoFlat], vAve[intoFlat], d, constants)
          : constants.f_speedMin;
        f[d] = fd;
        cost[d] = CCMath.CostFieldValue(fd, valid ? discomfort[intoFlat] : 0f, valid, alpha, beta, gamma);
      }
    }
  }
}
