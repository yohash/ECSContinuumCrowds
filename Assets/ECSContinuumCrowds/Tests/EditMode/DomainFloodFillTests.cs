using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Flood-fill domain derivation (spec §8.2, Decision D5) + the cache
  /// trigger matrix (spec §8.4, Decision D6).
  /// </summary>
  public class DomainFloodFillTests
  {
    private const int W = 16;
    private const int H = 16;
    private static readonly GridIndexer Gi = new GridIndexer(W, H);

    private struct Fill : System.IDisposable
    {
      public NativeList<int> Cells;
      public NativeParallelHashMap<int, int> GlobalToLocal;
      public NativeList<int4> Neighbors;

      public static Fill Run(
        NativeArray<byte> walkable, int2[] goals, int2 min, int2 max, int horizon = 999)
      {
        var f = new Fill {
          Cells = new NativeList<int>(64, Allocator.Temp),
          GlobalToLocal = new NativeParallelHashMap<int, int>(Gi.CellCount, Allocator.Temp),
          Neighbors = new NativeList<int4>(64, Allocator.Temp),
        };
        using var goalArr = new NativeArray<int2>(goals, Allocator.Temp);
        CCDomainOps.FloodFill(
          Gi, walkable, goalArr, min, max, horizon,
          f.Cells, f.GlobalToLocal, f.Neighbors);
        return f;
      }

      public bool Contains(int x, int y) => GlobalToLocal.ContainsKey(Gi.Flat(x, y));

      public void Dispose()
      {
        Cells.Dispose();
        GlobalToLocal.Dispose();
        Neighbors.Dispose();
      }
    }

    private static NativeArray<byte> Walkable(System.Func<int, int, bool> pred)
    {
      var w = new NativeArray<byte>(Gi.CellCount, Allocator.Temp);
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          w[Gi.Flat(x, y)] = pred(x, y) ? (byte)1 : (byte)0;
        }
      }
      return w;
    }

    [Test]
    public void OpenGridFillsPaddedBoundsOnly()
    {
      using var walkable = Walkable((x, y) => true);
      using var fill = Fill.Run(
        walkable, new[] { new int2(8, 8) }, new int2(5, 5), new int2(11, 11));
      Assert.AreEqual(7 * 7, fill.Cells.Length, "padded AABB is the spatial bound");
      Assert.IsTrue(fill.Contains(5, 5));
      Assert.IsFalse(fill.Contains(4, 8), "cells outside the pad must be excluded");
    }

    [Test]
    public void WallsAndDisconnectedPocketsExcluded()
    {
      // wall column x=8 with a gap at y=8; pocket at (12..13, 2..3) fully walled
      using var walkable = Walkable((x, y) =>
        !(x == 8 && y != 8)
        && !((x is 11 or 14) && y is >= 1 and <= 4)
        && !((y is 1 or 4) && x is >= 11 and <= 14));
      using var fill = Fill.Run(
        walkable, new[] { new int2(2, 8) }, new int2(0, 0), new int2(15, 15));

      Assert.IsFalse(fill.Contains(8, 2), "wall cells are not in the domain");
      Assert.IsTrue(fill.Contains(8, 8), "the gap is");
      Assert.IsTrue(fill.Contains(12, 8), "far side reachable through the gap");
      Assert.IsFalse(fill.Contains(12, 2), "walled-off pocket interior is unreachable");
      Assert.IsFalse(fill.Contains(13, 3), "walled-off pocket interior is unreachable");
    }

    [Test]
    public void CanyonIsOneDomain_NoLengthwiseSplit()
    {
      // the historical tile failure mode (spec §8.1): a long winding 1-wide
      // canyon must come back as ONE connected domain
      bool InCanyon(int x, int y) =>
        (y == 2 && x >= 1 && x <= 14) || (x == 14 && y >= 2 && y <= 12)
        || (y == 12 && x >= 3 && x <= 14) || (x == 3 && y >= 12 && y <= 14);
      using var walkable = Walkable(InCanyon);
      using var fill = Fill.Run(
        walkable, new[] { new int2(1, 2) }, new int2(0, 0), new int2(15, 15));

      int canyonCells = 0;
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          if (InCanyon(x, y)) {
            canyonCells++;
            Assert.IsTrue(fill.Contains(x, y), $"canyon cell ({x},{y}) missing from domain");
          }
        }
      }
      Assert.AreEqual(canyonCells, fill.Cells.Length, "domain is exactly the canyon — one piece, no seams");
    }

    [Test]
    public void HorizonCellsCapsGraphDistance()
    {
      using var walkable = Walkable((x, y) => true);
      using var fill = Fill.Run(
        walkable, new[] { new int2(8, 8) }, new int2(0, 0), new int2(15, 15), horizon: 3);
      // BFS depth ≤ 3 → Manhattan ball of radius 3 = 25 cells
      Assert.AreEqual(25, fill.Cells.Length);
      Assert.IsTrue(fill.Contains(8, 11));
      Assert.IsFalse(fill.Contains(8, 12));
    }

    [Test]
    public void NeighborTableMatchesAdjacency()
    {
      using var walkable = Walkable((x, y) => x != 8 || y == 8);
      using var fill = Fill.Run(
        walkable, new[] { new int2(2, 8) }, new int2(0, 0), new int2(15, 15));

      for (int local = 0; local < fill.Cells.Length; local++) {
        var c = Gi.Coord(fill.Cells[local]);
        var entry = fill.Neighbors[local];
        for (int d = 0; d < CCMath.NumDirections; d++) {
          var n = c + CCMath.ENSWint(d);
          bool inDomain = Gi.InBounds(n)
            && fill.GlobalToLocal.TryGetValue(Gi.Flat(n), out int expected);
          if (inDomain) {
            fill.GlobalToLocal.TryGetValue(Gi.Flat(n), out int exp);
            Assert.AreEqual(exp, entry[d], $"neighbor table wrong at {c} dir {d}");
          } else {
            Assert.AreEqual(-1, entry[d], $"absent neighbor at {c} dir {d} must be −1");
          }
        }
      }
    }

    [Test]
    public void UnwalkableGoalCellsAreNotSeeded()
    {
      using var walkable = Walkable((x, y) => !(x == 8 && y == 8));
      using var fill = Fill.Run(
        walkable, new[] { new int2(8, 8), new int2(9, 8) }, new int2(0, 0), new int2(15, 15));
      Assert.IsFalse(fill.Contains(8, 8), "invalid (g ≥ 1) goal cell is excluded from the fill");
      Assert.IsTrue(fill.Contains(9, 8), "valid goal cell seeds normally");
      Assert.Greater(fill.Cells.Length, 1);
    }
  }

  /// <summary>Trigger matrix for the domain cache (spec §8.4).</summary>
  public class DomainTriggerTests
  {
    private const float Pad = 16f;

    private static DomainRefreshReason Eval(
      bool valid = true,
      float2 goalCentroid = default, int goalCount = 4,
      float2 cachedGoalCentroid = default, int cachedGoalCount = 4,
      float2 centroid = default, float radius = 5f,
      float2 cachedCentroid = default, float cachedRadius = 5f,
      int walkVersion = 1, int cachedWalkVersion = 1,
      bool escaped = false, bool stalled = false)
      => CCDomainOps.EvaluateTriggers(
        valid, cachedGoalCentroid, cachedGoalCount, goalCentroid, goalCount,
        cachedCentroid, cachedRadius, centroid, radius,
        cachedWalkVersion, walkVersion, escaped, stalled, Pad);

    [Test]
    public void SteadyStateIsACacheHit()
      => Assert.AreEqual(DomainRefreshReason.None, Eval());

    [Test]
    public void NeverBuiltFires()
      => Assert.AreEqual(DomainRefreshReason.NeverBuilt, Eval(valid: false));

    [Test]
    public void HysteresisAbsorbsDriftBelowHalfPad()
      => Assert.AreEqual(DomainRefreshReason.None,
        Eval(centroid: new float2(Pad * 0.5f - 0.5f, 0f)));

    [Test]
    public void CentroidDriftPastHalfPadFires()
      => Assert.AreEqual(DomainRefreshReason.GroupMoved,
        Eval(centroid: new float2(Pad * 0.5f + 0.5f, 0f)));

    [Test]
    public void RadiusGrowthPastHalfPadFires()
      => Assert.AreEqual(DomainRefreshReason.GroupGrew,
        Eval(radius: 5f + Pad * 0.5f + 0.5f));

    [Test]
    public void GoalCentroidMoveFires()
      => Assert.AreEqual(DomainRefreshReason.GoalChanged,
        Eval(goalCentroid: new float2(1.5f, 0f)));

    [Test]
    public void GoalCountChangeFires()
      => Assert.AreEqual(DomainRefreshReason.GoalChanged, Eval(goalCount: 6));

    [Test]
    public void WalkabilityVersionBumpFires()
      => Assert.AreEqual(DomainRefreshReason.WalkabilityEdited, Eval(walkVersion: 2));

    [Test]
    public void EscapeIsAHardTrigger()
      => Assert.AreEqual(DomainRefreshReason.UnitEscaped, Eval(escaped: true));

    [Test]
    public void StallOutranksEverythingElse()
      => Assert.AreEqual(DomainRefreshReason.UnitStalled,
        Eval(escaped: true, stalled: true, walkVersion: 2));
  }
}
