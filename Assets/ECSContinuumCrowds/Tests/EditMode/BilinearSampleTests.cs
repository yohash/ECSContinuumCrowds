using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  public class BilinearSampleTests
  {
    private static NativeArray<float2> ConstantField(GridIndexer gi, float2 v)
    {
      var arr = new NativeArray<float2>(gi.CellCount, Allocator.Temp);
      for (int i = 0; i < arr.Length; i++) arr[i] = v;
      return arr;
    }

    [Test]
    public void ExactCellCenterReturnsCellValue()
    {
      var gi = new GridIndexer(4, 4);
      using var vel = ConstantField(gi, float2.zero);
      vel[gi.Flat(2, 1)] = new float2(3f, -1f);
      var sampled = CCMath.BilinearSampleVelocity(vel, gi, new float2(2.5f, 1.5f));
      Assert.AreEqual(3f, sampled.x, 1e-5f);
      Assert.AreEqual(-1f, sampled.y, 1e-5f);
    }

    [Test]
    public void MidpointBlendsEqually()
    {
      var gi = new GridIndexer(4, 4);
      using var vel = ConstantField(gi, float2.zero);
      vel[gi.Flat(1, 1)] = new float2(2f, 0f);
      vel[gi.Flat(2, 1)] = new float2(4f, 0f);
      // halfway between the centers of (1,1) and (2,1)
      var sampled = CCMath.BilinearSampleVelocity(vel, gi, new float2(2.0f, 1.5f));
      Assert.AreEqual(3f, sampled.x, 1e-5f);
    }

    [Test]
    public void GridEdgeRenormalizesInsteadOfFadingToZero()
    {
      var gi = new GridIndexer(4, 4);
      using var vel = ConstantField(gi, new float2(5f, 0f));
      // at the very corner, 3 of 4 bilinear corners are out of bounds; the
      // remaining weight must renormalize to the full value (spec §13.1)
      var sampled = CCMath.BilinearSampleVelocity(vel, gi, new float2(0.1f, 0.1f));
      Assert.AreEqual(5f, sampled.x, 1e-4f);
    }

    [Test]
    public void FullyOutsideReturnsZero()
    {
      var gi = new GridIndexer(4, 4);
      using var vel = ConstantField(gi, new float2(5f, 5f));
      var sampled = CCMath.BilinearSampleVelocity(vel, gi, new float2(-3f, -3f));
      Assert.AreEqual(0f, math.length(sampled), 1e-6f);
    }
  }
}
