using NUnit.Framework;

namespace Yohash.ECSContinuumCrowds.Tests
{
  public class CCConstantsTests
  {
    [Test]
    public void DefaultsMatchReferenceRepo()
    {
      // verified against yohash/ContinuumCrowds Constants.cs
      var c = CCConstants.Defaults;
      Assert.AreEqual(0f, c.u_unitRadialFalloff);
      Assert.AreEqual(0.25f, c.v_dynamicFootprintThreshold);
      Assert.AreEqual(1f, c.v_predictiveSeconds);
      Assert.AreEqual(0.3f, c.v_scaleMax);
      Assert.AreEqual(0.25f, c.v_scaleMin);
      Assert.AreEqual(1f, c.f_slopeMax);
      Assert.AreEqual(-1f, c.f_slopeMin);
      Assert.AreEqual(0.8f, c.f_rhoMax);
      Assert.AreEqual(0.3f, c.f_rhoMin);
      Assert.AreEqual(0f, c.f_speedMin);
      Assert.AreEqual(20f, c.f_speedMax);
      Assert.AreEqual(1f, c.C_alpha);
      Assert.AreEqual(1f, c.C_beta);
      Assert.AreEqual(1f, c.C_gamma);
      Assert.AreEqual(2.5f, c.maxWeight);
      Assert.AreEqual(1f, c.minWeight);
    }

    [Test]
    public void FlatSpeedMatchesRepoHelper()
    {
      // repo Constants.FlatSpeed with defaults:
      // 20 + (1)/(2) * (0 − 20) = 10
      Assert.AreEqual(10f, CCConstants.Defaults.FlatSpeed, 1e-5f);
    }

    [Test]
    public void RhoBarIsDerivedFromLambda()
    {
      var c = CCConstants.Defaults;
      c.lambda = 3f;
      c.DeriveRhoBar();
      Assert.AreEqual(0.125f, c.rhoBar, 1e-6f);
    }

    [Test]
    public void ValidationCatchesRhoBarViolation()
    {
      var c = CCConstants.Defaults;
      c.lambda = 1f; // ρ̄ = 0.5 > f_rhoMin = 0.3
      c.DeriveRhoBar();
      Assert.IsFalse(c.IsValid);
    }
  }
}
