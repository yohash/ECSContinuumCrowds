using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// The two most consequential lines in the system (spec §16.3) tested in
  /// isolation: the flow-speed nonnegativity clamp (lane formation) and the
  /// speed/cost regime plumbing around the into-cell rule.
  /// </summary>
  public class SpeedCostFieldTests
  {
    private static readonly CCConstants C = CCConstants.Defaults;

    [Test]
    public void FlowSpeedClampsNegativeDotToZero()
    {
      // opposing average velocity must slow you to zero, never carry you
      // backwards (repo changelog v0.2.7; paper §3.2)
      var opposing = new float2(-3f, 0f);
      Assert.AreEqual(0f, CCMath.FlowSpeed(opposing, CCMath.ENSW(CCMath.DirE)));
      // ... and the same crowd is full speed WITH the flow — this asymmetry
      // is lane formation
      Assert.AreEqual(3f, CCMath.FlowSpeed(opposing, CCMath.ENSW(CCMath.DirW)));
    }

    [Test]
    public void TopographicalSpeed_FlatMatchesRepoFlatSpeed()
    {
      float f = CCMath.TopographicalSpeed(float2.zero, CCMath.ENSW(CCMath.DirE), C);
      Assert.AreEqual(C.FlatSpeed, f, 1e-5f); // 10 with defaults
    }

    [Test]
    public void TopographicalSpeed_DownhillFasterUphillSlower()
    {
      var dh = new float2(0.5f, 0f); // rises toward +x
      float uphill = CCMath.TopographicalSpeed(dh, CCMath.ENSW(CCMath.DirE), C);
      float downhill = CCMath.TopographicalSpeed(dh, CCMath.ENSW(CCMath.DirW), C);
      Assert.Less(uphill, C.FlatSpeed);
      Assert.Greater(downhill, C.FlatSpeed);
    }

    [Test]
    public void SpeedField_RegimeBoundariesAreContinuous()
    {
      var dh = new float2(0.2f, -0.1f);
      var vAve = new float2(2f, 1f);
      const float eps = 1e-4f;
      for (int d = 0; d < CCMath.NumDirections; d++) {
        // crossing f_rhoMin: topographical → lerp(t≈0)
        float below = CCMath.SpeedFieldPoint(C.f_rhoMin - eps, dh, vAve, d, C);
        float above = CCMath.SpeedFieldPoint(C.f_rhoMin + eps, dh, vAve, d, C);
        Assert.AreEqual(below, above, 1e-2f, $"discontinuity at f_rhoMin, dir {d}");
        // crossing f_rhoMax: lerp(t≈1) → flow
        below = CCMath.SpeedFieldPoint(C.f_rhoMax - eps, dh, vAve, d, C);
        above = CCMath.SpeedFieldPoint(C.f_rhoMax + eps, dh, vAve, d, C);
        Assert.AreEqual(below, above, 1e-2f, $"discontinuity at f_rhoMax, dir {d}");
      }
    }

    [Test]
    public void SpeedField_ClampsToConfiguredRange()
    {
      // absurd downhill would exceed f_speedMax without the final clamp
      var dh = new float2(-50f, 0f);
      float f = CCMath.SpeedFieldPoint(0f, dh, float2.zero, CCMath.DirE, C);
      Assert.LessOrEqual(f, C.f_speedMax);
      // absurd uphill floors at f_speedMin
      f = CCMath.SpeedFieldPoint(0f, new float2(50f, 0f), float2.zero, CCMath.DirE, C);
      Assert.AreEqual(C.f_speedMin, f);
    }

    [Test]
    public void CostField_ZeroSpeedOrInvalidIsInfinite()
    {
      Assert.IsTrue(float.IsPositiveInfinity(CCMath.CostFieldValue(0f, 0f, true, 1f, 1f, 1f)));
      Assert.IsTrue(float.IsPositiveInfinity(CCMath.CostFieldValue(5f, 0f, false, 1f, 1f, 1f)));
    }

    [Test]
    public void CostField_MatchesFormulaAndClampsDiscomfort()
    {
      // C = α + β/f + γ·g'/f with g' clamped to [0,1] (⚠ repo divergence)
      Assert.AreEqual(1f + 1f / 4f + 1f * 0.5f / 4f,
        CCMath.CostFieldValue(4f, 0.5f, true, 1f, 1f, 1f), 1e-6f);
      // g > 1 clamps to 1 (the ≥1 = impassable rejection happens upstream
      // via intoValid; this covers the 0.999…→1 clamp inside the formula)
      Assert.AreEqual(CCMath.CostFieldValue(4f, 1f, true, 1f, 1f, 1f),
        CCMath.CostFieldValue(4f, 3f, true, 1f, 1f, 1f), 1e-6f);
      // negative discomfort clamps to 0
      Assert.AreEqual(CCMath.CostFieldValue(4f, 0f, true, 1f, 1f, 1f),
        CCMath.CostFieldValue(4f, -2f, true, 1f, 1f, 1f), 1e-6f);
    }

    [Test]
    public void VelocityFromGradient_SelectsDirectionFaces()
    {
      // f: E=1, N=2, W=3, S=4
      var f = new float4(1f, 2f, 3f, 4f);
      // gradient +x → motion −x → W face (3)
      var v = CCMath.VelocityFromGradient(new float2(1f, 0f), f);
      Assert.AreEqual(-3f, v.x, 1e-6f);
      // gradient −x → motion +x → E face (1)
      v = CCMath.VelocityFromGradient(new float2(-1f, 0f), f);
      Assert.AreEqual(1f, v.x, 1e-6f);
      // gradient +y → motion −y → S face (4)
      v = CCMath.VelocityFromGradient(new float2(0f, 1f), f);
      Assert.AreEqual(-4f, v.y, 1e-6f);
      // gradient −y → motion +y → N face (2)
      v = CCMath.VelocityFromGradient(new float2(0f, -1f), f);
      Assert.AreEqual(2f, v.y, 1e-6f);
    }
  }
}
