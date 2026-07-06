using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Editor-mode assertion tests for the density splat kernel invariants
  /// (spec §2.3/§6.4): the paper's §4.1 requirement that each unit contribute
  /// no less than ρ̄ to its own cell and no more than ρ̄ to any neighboring
  /// cell, plus continuity and scatter/gather agreement.
  /// </summary>
  public class SplatKernelTests
  {
    private const float Lambda = 2f;
    private static readonly float RhoBar = CCMath.RhoBar(Lambda);

    private static float2 RandomPos(ref Random rng) =>
      rng.NextFloat2(new float2(1f, 1f), new float2(63f, 63f));

    [Test]
    public void RhoBarDerivation()
    {
      Assert.AreEqual(0.25f, CCMath.RhoBar(2f), 1e-6f);
      Assert.AreEqual(0.5f, CCMath.RhoBar(1f), 1e-6f);
    }

    [Test]
    public void OwnCellReceivesAtLeastRhoBar()
    {
      var rng = new Random(1234);
      for (int trial = 0; trial < 10000; trial++) {
        var pos = RandomPos(ref rng);
        var ownCell = (int2)math.floor(pos);
        float w = CCMath.SplatWeight(pos, ownCell, Lambda);
        Assert.GreaterOrEqual(w, RhoBar - 1e-5f,
          $"own-cell weight {w} < ρ̄ {RhoBar} at pos {pos}");
      }
    }

    [Test]
    public void NeighborCellsReceiveAtMostRhoBar()
    {
      var rng = new Random(5678);
      for (int trial = 0; trial < 10000; trial++) {
        var pos = RandomPos(ref rng);
        var ownCell = (int2)math.floor(pos);
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            var neighbor = ownCell + new int2(dx, dy);
            float w = CCMath.SplatWeight(pos, neighbor, Lambda);
            Assert.LessOrEqual(w, RhoBar + 1e-5f,
              $"neighbor {neighbor} weight {w} > ρ̄ {RhoBar} at pos {pos}");
          }
        }
      }
    }

    [Test]
    public void DefaultConstantsSatisfyRhoBarInvariant()
    {
      // config-time assert (spec §6.4): f_rhoMin ≥ ρ̄ so an isolated unit
      // always moves at topographical speed
      var c = CCConstants.Defaults;
      Assert.GreaterOrEqual(c.f_rhoMin, c.rhoBar);
      Assert.IsTrue(c.IsValid);
    }

    [Test]
    public void GatherFormMatchesScatterForm()
    {
      var rng = new Random(9012);
      for (int trial = 0; trial < 10000; trial++) {
        var pos = RandomPos(ref rng);
        var baseCell = CCMath.SplatBaseCell(pos);
        var scatter = CCMath.SplatWeights(pos, Lambda);
        // A=(0,0), B=(1,0), C=(1,1), D=(0,1)
        Assert.AreEqual(scatter.x, CCMath.SplatWeight(pos, baseCell, Lambda), 1e-6f);
        Assert.AreEqual(scatter.y, CCMath.SplatWeight(pos, baseCell + new int2(1, 0), Lambda), 1e-6f);
        Assert.AreEqual(scatter.z, CCMath.SplatWeight(pos, baseCell + new int2(1, 1), Lambda), 1e-6f);
        Assert.AreEqual(scatter.w, CCMath.SplatWeight(pos, baseCell + new int2(0, 1), Lambda), 1e-6f);
      }
    }

    [Test]
    public void ZeroOutsideTwoByTwoSupport()
    {
      var rng = new Random(3456);
      for (int trial = 0; trial < 1000; trial++) {
        var pos = RandomPos(ref rng);
        var baseCell = CCMath.SplatBaseCell(pos);
        for (int dy = -2; dy <= 3; dy++) {
          for (int dx = -2; dx <= 3; dx++) {
            bool inSupport = dx >= 0 && dx <= 1 && dy >= 0 && dy <= 1;
            if (inSupport) continue;
            Assert.AreEqual(0f,
              CCMath.SplatWeight(pos, baseCell + new int2(dx, dy), Lambda),
              $"cell offset ({dx},{dy}) outside 2×2 support got nonzero weight");
          }
        }
      }
    }

    [Test]
    public void ContinuousInPosition()
    {
      // continuity requirement (spec §6.4 #1): speed fields must not pop as
      // units cross cell centers/boundaries — sample weight at any fixed cell
      // as the unit takes a small step, including across cell-center seams
      var rng = new Random(7890);
      const float step = 1e-4f;
      // weight function is Lipschitz with slope ≤ λ·max(f)^{λ-1} ≤ λ here
      const float tolerance = step * Lambda * 4f;
      for (int trial = 0; trial < 10000; trial++) {
        var pos = RandomPos(ref rng);
        var dir = math.normalize(rng.NextFloat2Direction());
        var next = pos + dir * step;
        // check all cells in the union of both supports
        var lo = math.min(CCMath.SplatBaseCell(pos), CCMath.SplatBaseCell(next));
        for (int dy = 0; dy <= 2; dy++) {
          for (int dx = 0; dx <= 2; dx++) {
            var cell = lo + new int2(dx, dy);
            float w0 = CCMath.SplatWeight(pos, cell, Lambda);
            float w1 = CCMath.SplatWeight(next, cell, Lambda);
            Assert.LessOrEqual(math.abs(w1 - w0), tolerance,
              $"discontinuity at pos {pos} cell {cell}: {w0} → {w1}");
          }
        }
      }
    }

    [Test]
    public void ExactCellCenterDepositsFullWeightToOwnCell()
    {
      var pos = new float2(10.5f, 20.5f); // exactly the center of cell (10, 20)
      Assert.AreEqual(1f, CCMath.SplatWeight(pos, new int2(10, 20), Lambda), 1e-6f);
      Assert.AreEqual(0f, CCMath.SplatWeight(pos, new int2(11, 20), Lambda), 1e-6f);
      Assert.AreEqual(0f, CCMath.SplatWeight(pos, new int2(10, 21), Lambda), 1e-6f);
      Assert.AreEqual(0f, CCMath.SplatWeight(pos, new int2(11, 21), Lambda), 1e-6f);
    }
  }
}
