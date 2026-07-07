using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Gather-stamping correctness (spec §15 P2, Decision D3): the gather
  /// formulation (CCStampOps.GatherCell — the exact job code path) must
  /// reproduce the brute-force scatter reference over the full grid, and the
  /// active-cell derivation must cover every cell any unit can touch.
  /// </summary>
  public class GatherStampingTests
  {
    private const int W = 24;
    private const int H = 24;
    private static readonly GridIndexer Gi = new GridIndexer(W, H);
    private const float CellSize = 1f;

    private static NativeParallelMultiHashMap<int, UnitStampData> BuildMap(
      UnitStampData[] stamps, int bucketCells, int2 dims,
      out NativeArray<byte> occupied)
    {
      var map = new NativeParallelMultiHashMap<int, UnitStampData>(
        stamps.Length * 2 + 4, Allocator.Temp);
      occupied = new NativeArray<byte>(dims.x * dims.y, Allocator.Temp);
      foreach (var s in stamps) {
        var b = math.clamp(
          CCStampOps.BucketOf(s.Position, bucketCells), int2.zero, dims - 1);
        int key = CCStampOps.BucketKey(b, dims);
        map.Add(key, s);
        occupied[key] = 1;
      }
      return map;
    }

    private static void RunScatterReference(
      UnitStampData[] stamps, in CCConstants c,
      NativeArray<float> rho, NativeArray<float2> momentum)
    {
      foreach (var s in stamps) {
        CCStampOps.ScatterStatic(s, Gi, c.lambda, rho, momentum);
        CCStampOps.ScatterPredictiveGhosts(s, Gi, c, CellSize, rho, momentum);
      }
    }

    private static UnitStampData[] RandomStaticUnits(int count, uint seed)
    {
      var rng = new Random(seed);
      var stamps = new UnitStampData[count];
      for (int i = 0; i < count; i++) {
        stamps[i] = new UnitStampData {
          Position = rng.NextFloat2(new float2(1f), new float2(W - 1f, H - 1f)),
          Velocity = float2.zero, // below v_dynamicFootprintThreshold
          Mass = rng.NextFloat(0.5f, 3f),
          FootprintSize = 1f,
        };
      }
      return stamps;
    }

    [Test]
    public void BucketSizingFromDefaults()
    {
      // reach = 1.5 (static) + 0 (falloff) + min(8 cap, 20 m/s · 1 s / 1 m) = 9.5 → 10
      Assert.AreEqual(10, CCStampOps.BucketCells(CCConstants.Defaults, 1f));
    }

    [Test]
    public void StaticGatherMatchesScatterReferenceExactly()
    {
      var c = CCConstants.Defaults;
      var stamps = RandomStaticUnits(40, 99);
      int bucketCells = CCStampOps.BucketCells(c, CellSize);
      var dims = CCStampOps.BucketDims(Gi, bucketCells);

      using var rhoRef = new NativeArray<float>(Gi.CellCount, Allocator.Temp);
      using var momRef = new NativeArray<float2>(Gi.CellCount, Allocator.Temp);
      RunScatterReference(stamps, c, rhoRef, momRef);

      using var map = BuildMap(stamps, bucketCells, dims, out var occupied);
      using (occupied) {
        for (int i = 0; i < Gi.CellCount; i++) {
          CCStampOps.GatherCell(
            Gi.Coord(i), map, bucketCells, dims, c, CellSize,
            out var rho, out var mom);
          Assert.AreEqual(rhoRef[i], rho, 1e-4f,
            $"ρ mismatch at {Gi.Coord(i)}");
          Assert.AreEqual(momRef[i].x, mom.x, 1e-4f, $"mom.x mismatch at {Gi.Coord(i)}");
          Assert.AreEqual(momRef[i].y, mom.y, 1e-4f, $"mom.y mismatch at {Gi.Coord(i)}");
        }
      }
    }

    [Test]
    public void MultipleUnitsInOneCellAccumulate()
    {
      var c = CCConstants.Defaults;
      // three units sharing a cell center: ρ must be the sum of masses
      var pos = new float2(5.5f, 5.5f);
      var stamps = new[] {
        new UnitStampData { Position = pos, Mass = 1f, FootprintSize = 1f },
        new UnitStampData { Position = pos, Mass = 2f, FootprintSize = 1f },
        new UnitStampData { Position = pos, Mass = 0.5f, FootprintSize = 1f },
      };
      int bucketCells = CCStampOps.BucketCells(c, CellSize);
      var dims = CCStampOps.BucketDims(Gi, bucketCells);
      using var map = BuildMap(stamps, bucketCells, dims, out var occupied);
      using (occupied) {
        CCStampOps.GatherCell(
          new int2(5, 5), map, bucketCells, dims, c, CellSize,
          out var rho, out _);
        Assert.AreEqual(3.5f, rho, 1e-5f);
      }
    }

    [Test]
    public void ActiveCellsCoverEverythingAnyUnitTouches()
    {
      var c = CCConstants.Defaults;
      var rng = new Random(4242);
      var stamps = new UnitStampData[30];
      for (int i = 0; i < stamps.Length; i++) {
        stamps[i] = new UnitStampData {
          Position = rng.NextFloat2(new float2(1f), new float2(W - 1f, H - 1f)),
          // mix of static and fast movers so predictive reach is exercised
          Velocity = rng.NextFloat2(new float2(-25f), new float2(25f)),
          Mass = 1f,
          FootprintSize = 1f,
        };
      }

      using var rhoRef = new NativeArray<float>(Gi.CellCount, Allocator.Temp);
      using var momRef = new NativeArray<float2>(Gi.CellCount, Allocator.Temp);
      RunScatterReference(stamps, c, rhoRef, momRef);

      int bucketCells = CCStampOps.BucketCells(c, CellSize);
      var dims = CCStampOps.BucketDims(Gi, bucketCells);
      using var map = BuildMap(stamps, bucketCells, dims, out var occupied);
      using (occupied)
      using (var active = new NativeArray<byte>(dims.x * dims.y, Allocator.Temp))
      using (var cells = new NativeList<int>(Gi.CellCount, Allocator.Temp)) {
        CCStampOps.MarkActiveBuckets(occupied, active, dims);
        CCStampOps.EmitActiveCells(active, dims, bucketCells, Gi, cells);

        var inActive = new bool[Gi.CellCount];
        foreach (var flat in cells.AsArray()) {
          inActive[flat] = true;
        }
        for (int i = 0; i < Gi.CellCount; i++) {
          if (rhoRef[i] > 0f) {
            Assert.IsTrue(inActive[i],
              $"cell {Gi.Coord(i)} has scatter density {rhoRef[i]} but is not active");
          }
        }
      }
    }
  }
}
