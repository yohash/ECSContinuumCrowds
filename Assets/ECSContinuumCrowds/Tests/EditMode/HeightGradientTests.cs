using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  public class HeightGradientTests
  {
    private static NativeArray<float> MakeHeights(GridIndexer gi, System.Func<int, int, float> h)
    {
      var arr = new NativeArray<float>(gi.CellCount, Allocator.Temp);
      for (int y = 0; y < gi.H; y++) {
        for (int x = 0; x < gi.W; x++) {
          arr[gi.Flat(x, y)] = h(x, y);
        }
      }
      return arr;
    }

    [Test]
    public void LinearRampGivesConstantGradient()
    {
      var gi = new GridIndexer(8, 8);
      const float cellSize = 2f;
      const float slope = 0.5f;
      // height rises 'slope' per world unit along +x
      using var heights = MakeHeights(gi, (x, y) => x * cellSize * slope);

      for (int y = 0; y < gi.H; y++) {
        for (int x = 0; x < gi.W; x++) {
          var dh = CCMath.HeightGradient(heights, gi, new int2(x, y), cellSize);
          Assert.AreEqual(slope, dh.x, 1e-5f, $"at ({x},{y})");
          Assert.AreEqual(0f, dh.y, 1e-5f, $"at ({x},{y})");
        }
      }
    }

    [Test]
    public void DiagonalRampGivesBothComponents()
    {
      var gi = new GridIndexer(8, 8);
      using var heights = MakeHeights(gi, (x, y) => x * 0.25f + y * 0.75f);

      var dh = CCMath.HeightGradient(heights, gi, new int2(4, 4), 1f);
      Assert.AreEqual(0.25f, dh.x, 1e-5f);
      Assert.AreEqual(0.75f, dh.y, 1e-5f);
    }

    [Test]
    public void EdgesUseOneSidedDifferences()
    {
      var gi = new GridIndexer(4, 4);
      // quadratic in x: central and one-sided differ, so edges are detectable
      using var heights = MakeHeights(gi, (x, y) => x * x);

      // left edge: (h[1] − h[0]) / 1 = 1
      Assert.AreEqual(1f, CCMath.HeightGradient(heights, gi, new int2(0, 2), 1f).x, 1e-5f);
      // right edge: (h[3] − h[2]) / 1 = 5
      Assert.AreEqual(5f, CCMath.HeightGradient(heights, gi, new int2(3, 2), 1f).x, 1e-5f);
      // interior x=1: central (h[2] − h[0]) / 2 = 2
      Assert.AreEqual(2f, CCMath.HeightGradient(heights, gi, new int2(1, 2), 1f).x, 1e-5f);
    }

    [Test]
    public void DegenerateSingleColumnGridIsZero()
    {
      var gi = new GridIndexer(1, 4);
      using var heights = MakeHeights(gi, (x, y) => y * 3f);
      var dh = CCMath.HeightGradient(heights, gi, new int2(0, 1), 1f);
      Assert.AreEqual(0f, dh.x, 1e-6f);
      Assert.AreEqual(3f, dh.y, 1e-5f);
    }
  }
}
