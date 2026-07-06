using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Unit tests for CCMath.EikonalSolve — the single update formula shared
  /// by FMM (and FIM in Phase 4). Hand-computed cases follow the repo's
  /// exact arithmetic (spec §9.3–9.4).
  /// </summary>
  public class EikonalSolveTests
  {
    private const float Inf = float.PositiveInfinity;
    private const float WMax = 2.5f;
    private const float WMin = 1f;

    [Test]
    public void QuadraticWeightedBlend_HandComputed()
    {
      // phi_m: E arrival = 1 (φ=0 + C=1), N arrival = 1 → phi_mx = phi_my = 1,
      // C_mx = C_my = 1. valTest = 1 + 1 − 1 = 1; diff² = 0 ≤ 1 → quadratic:
      // radical = sqrt(1·1·(2 − 0)) = √2; soln1,2 = (1 + 1 ± √2)/2
      // blend = (max·2.5 + min·1)/3.5
      var phi_m = new float4(1f, 1f, Inf, Inf); // E=0+1, N=0+1
      var c = new float4(1f, 1f, 1f, 1f);
      float expectedMax = (2f + math.sqrt(2f)) / 2f;
      float expectedMin = (2f - math.sqrt(2f)) / 2f;
      float expected = (expectedMax * WMax + expectedMin * WMin) / (WMax + WMin);
      Assert.AreEqual(expected, CCMath.EikonalSolve(phi_m, c, WMax, WMin), 1e-5f);
    }

    [Test]
    public void DiscriminantFailure_UsesOneDimensionalSolution()
    {
      // large φ difference between axes → (phi_mx − phi_my)² > valTest →
      // simplified solution: min + its cost
      var phi_m = new float4(1.5f, 100f, Inf, Inf);
      var c = new float4(1.5f, 1f, 2f, 1f);
      Assert.AreEqual(1.5f + 1.5f, CCMath.EikonalSolve(phi_m, c, WMax, WMin), 1e-5f);
    }

    [Test]
    public void OneAxisFullyInfinite_DropsDimension()
    {
      // spec §9.3 degenerate case: y axis has no readable neighbors → 1-D
      var phi_m = new float4(3f, Inf, 4f, Inf);
      var c = new float4(1.25f, Inf, 2f, Inf);
      Assert.AreEqual(3f + 1.25f, CCMath.EikonalSolve(phi_m, c, WMax, WMin), 1e-5f);
    }

    [Test]
    public void AllInfinite_ReturnsInfinity()
    {
      var phi_m = new float4(Inf);
      var c = new float4(1f);
      Assert.IsTrue(float.IsPositiveInfinity(CCMath.EikonalSolve(phi_m, c, WMax, WMin)));
    }

    [Test]
    public void TieOnAxisPicksFirstComponent()
    {
      // repo: phi_mx == phi_m[0] ? C[0] : C[2] — ties select E (and N)
      var phi_m = new float4(5f, 100f, 5f, Inf); // E == W
      var c = new float4(2f, 1f, 3f, 1f);
      // 1-D via discriminant (diff large): min is x axis at 5, cost from E = 2
      Assert.AreEqual(7f, CCMath.EikonalSolve(phi_m, c, WMax, WMin), 1e-5f);
    }

    [Test]
    public void NeverReturnsNaN_RandomizedInputs()
    {
      var rng = new Unity.Mathematics.Random(777);
      for (int i = 0; i < 100000; i++) {
        // random mixture of finite and infinite lanes
        float4 c = default;
        float4 phi_m = default;
        for (int d = 0; d < 4; d++) {
          bool wall = rng.NextFloat() < 0.25f;
          c[d] = wall ? Inf : rng.NextFloat(0.05f, 25f);
          bool unread = wall || rng.NextFloat() < 0.25f;
          phi_m[d] = unread ? Inf : rng.NextFloat(0f, 50f) + c[d];
        }
        float result = CCMath.EikonalSolve(phi_m, c, WMax, WMin);
        Assert.IsFalse(float.IsNaN(result),
          $"NaN from phi_m={phi_m} C={c} — the guard rails must catch this");
      }
    }
  }
}
