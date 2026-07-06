using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  public class GridIndexerTests
  {
    [Test]
    public void FlatCoordRoundtrip()
    {
      var gi = new GridIndexer(7, 5);
      for (int y = 0; y < gi.H; y++) {
        for (int x = 0; x < gi.W; x++) {
          int flat = gi.Flat(x, y);
          Assert.AreEqual(new int2(x, y), gi.Coord(flat));
          Assert.AreEqual(flat, gi.Flat(gi.Coord(flat)));
        }
      }
    }

    [Test]
    public void FlatIsRowMajor()
    {
      // spec §2.1: flat index i = y * W + x
      var gi = new GridIndexer(512, 512);
      Assert.AreEqual(0, gi.Flat(0, 0));
      Assert.AreEqual(511, gi.Flat(511, 0));
      Assert.AreEqual(512, gi.Flat(0, 1));
      Assert.AreEqual(512 * 512 - 1, gi.Flat(511, 511));
    }

    [Test]
    public void InBoundsEdgesAndNegatives()
    {
      var gi = new GridIndexer(4, 3);
      Assert.IsTrue(gi.InBounds(0, 0));
      Assert.IsTrue(gi.InBounds(3, 2));
      Assert.IsFalse(gi.InBounds(4, 0));
      Assert.IsFalse(gi.InBounds(0, 3));
      Assert.IsFalse(gi.InBounds(-1, 0));
      Assert.IsFalse(gi.InBounds(0, -1));
      Assert.IsFalse(gi.InBounds(int.MinValue, int.MinValue));
    }

    [Test]
    public void CellCount()
    {
      Assert.AreEqual(512 * 512, new GridIndexer(512, 512).CellCount);
    }
  }
}
