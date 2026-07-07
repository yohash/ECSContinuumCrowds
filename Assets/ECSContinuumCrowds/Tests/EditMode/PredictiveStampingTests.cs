using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Predictive-velocity stamping (Decision D4, spec §7 + §15 P2 + risk
  /// register): the gather-side closed form validated against the
  /// brute-force ghost-chain scatter reference, plus the invariants the
  /// closed form must satisfy regardless of the approximation: threshold
  /// gating, forward-only deposits, momentum direction, scale falloff,
  /// distance cap, and continuity in unit position.
  /// </summary>
  public class PredictiveStampingTests
  {
    private const int W = 32;
    private const int H = 32;
    private static readonly GridIndexer Gi = new GridIndexer(W, H);
    private const float CellSize = 1f;

    private static float GatherRho(in UnitStampData u, int2 cell, in CCConstants c)
    {
      // single unit: gather = static splat + predictive closed form
      return CCMath.SplatWeight(u.Position, cell, c.lambda) * u.Mass
        + CCStampOps.PredictiveWeight(u, cell, c, CellSize) * u.Mass;
    }

    [Test]
    public void BelowThresholdHasNoPredictiveContribution()
    {
      var c = CCConstants.Defaults;
      var u = new UnitStampData {
        Position = new float2(10.5f, 10.5f),
        Velocity = new float2(0.2f, 0f), // < v_dynamicFootprintThreshold (0.25)
        Mass = 1f,
        FootprintSize = 1f,
      };
      for (int x = 0; x < W; x++) {
        Assert.AreEqual(0f,
          CCStampOps.PredictiveWeight(u, new int2(x, 10), c, CellSize),
          "static-only unit must deposit no ghosts");
      }
    }

    [Test]
    public void GhostsDepositOnlyForward()
    {
      var c = CCConstants.Defaults;
      var u = new UnitStampData {
        Position = new float2(16.5f, 16.5f),
        Velocity = new float2(3f, 0f),
        Mass = 1f,
        FootprintSize = 1f,
      };
      // ahead (outside static support): predictive > 0
      Assert.Greater(CCStampOps.PredictiveWeight(u, new int2(18, 16), c, CellSize), 0f);
      // behind: nothing
      Assert.AreEqual(0f, CCStampOps.PredictiveWeight(u, new int2(14, 16), c, CellSize));
      Assert.AreEqual(0f, CCStampOps.PredictiveWeight(u, new int2(13, 16), c, CellSize));
    }

    [Test]
    public void PureGhostCellAveragesToUnitVelocity()
    {
      // ghost contributions carry the unit's FULL velocity into the momentum
      // accumulator (spec §7.2 — "that is what projects the flow field
      // forward"), so for a single unit v̄ = Σw·m·v / Σw·m = v exactly
      var c = CCConstants.Defaults;
      var u = new UnitStampData {
        Position = new float2(16.5f, 16.5f),
        Velocity = new float2(2.5f, 1.0f),
        Mass = 1.7f,
        FootprintSize = 1f,
      };
      var cell = new int2(18, 17); // downstream, outside the 2×2 static support
      Assert.AreEqual(0f, CCMath.SplatWeight(u.Position, cell, c.lambda),
        "test setup: cell must be outside static support");
      float wp = CCStampOps.PredictiveWeight(u, cell, c, CellSize);
      Assert.Greater(wp, 0f, "test setup: cell must receive a ghost");
      float rho = wp * u.Mass;
      var momentum = wp * u.Mass * u.Velocity;
      var vAve = momentum / rho;
      Assert.AreEqual(u.Velocity.x, vAve.x, 1e-5f);
      Assert.AreEqual(u.Velocity.y, vAve.y, 1e-5f);
    }

    [Test]
    public void ScaleFadesWithLookaheadDistance()
    {
      var c = CCConstants.Defaults;
      var u = new UnitStampData {
        Position = new float2(8.5f, 16.5f),
        Velocity = new float2(8f, 0f), // 8 m/s → 8-cell reach (at the cap boundary)
        Mass = 1f,
        FootprintSize = 1f,
      };
      // sample on-axis cells at increasing lookahead; both are exact ghost
      // centers (t = 2 and t = 7), so SplatWeight is 1 and only scale differs
      float near = CCStampOps.PredictiveWeight(u, new int2(10, 16), c, CellSize);
      float far = CCStampOps.PredictiveWeight(u, new int2(15, 16), c, CellSize);
      Assert.Greater(near, 0f);
      Assert.Greater(far, 0f);
      Assert.Greater(near, far, "ghost scale must fade with lookahead (v_scaleMax → v_scaleMin)");
    }

    [Test]
    public void ExtrapolationDistanceIsCapped()
    {
      var c = CCConstants.Defaults; // cap = 8 cells
      var u = new UnitStampData {
        Position = new float2(4.5f, 16.5f),
        Velocity = new float2(30f, 0f), // uncapped reach would be 30 cells
        Mass = 1f,
        FootprintSize = 1f,
      };
      // within cap: ghosts land
      Assert.Greater(CCStampOps.PredictiveWeight(u, new int2(11, 16), c, CellSize), 0f);
      // beyond cap + splat support (4.5 + 8 + 1.5 = 14): nothing
      Assert.AreEqual(0f, CCStampOps.PredictiveWeight(u, new int2(15, 16), c, CellSize));
      Assert.AreEqual(0f, CCStampOps.PredictiveWeight(u, new int2(20, 16), c, CellSize));
    }

    [Test]
    public void GatherStaysWithinBandOfGhostChainReference()
    {
      // the closed form is an approximation of the discrete ghost chain
      // (spec §7.2: "a closed-form approximation is fine"); this is the risk-
      // register drift test: same support, bounded per-cell magnitude ratio
      var c = CCConstants.Defaults;
      var rng = new Random(31337);
      for (int trial = 0; trial < 50; trial++) {
        var u = new UnitStampData {
          Position = rng.NextFloat2(new float2(10f), new float2(20f)),
          Velocity = rng.NextFloat2Direction() * rng.NextFloat(1f, 12f),
          Mass = 1f,
          FootprintSize = 1f,
        };

        using var rhoRef = new NativeArray<float>(Gi.CellCount, Allocator.Temp);
        using var momRef = new NativeArray<float2>(Gi.CellCount, Allocator.Temp);
        CCStampOps.ScatterPredictiveGhosts(u, Gi, c, CellSize, rhoRef, momRef);

        float totalGather = 0f;
        float totalScatter = 0f;
        for (int i = 0; i < Gi.CellCount; i++) {
          float gather = CCStampOps.PredictiveWeight(u, Gi.Coord(i), c, CellSize);
          totalGather += gather;
          totalScatter += rhoRef[i];

          // support agreement: the gather form may not invent density far
          // from the ghost chain (1 cell of slack for the discrete spacing)
          if (gather > 1e-4f) {
            bool nearChain = false;
            var cc = CCMath.CellCenter(Gi.Coord(i));
            var dir = math.normalize(u.Velocity);
            float len = math.min(
              math.length(u.Velocity) * c.v_predictiveSeconds / CellSize,
              c.v_predictiveDistanceCapCells);
            float t = math.clamp(math.dot(cc - u.Position, dir), 0f, len);
            var closest = u.Position + dir * t;
            nearChain = math.distance(cc, closest) <= CCStampOps.StaticReachCells + 0.01f;
            Assert.IsTrue(nearChain,
              $"gather deposited {gather} at {Gi.Coord(i)}, off the extrapolation path");
          }
        }

        // aggregate magnitude: single-sample projection vs summed ghosts —
        // same order of magnitude, scatter ≥ gather is typical (multiple
        // ghosts overlap each cell). Guard the band so semantic drift trips.
        Assert.Greater(totalGather, 0f, "moving unit must project density forward");
        Assert.Greater(totalScatter, 0f);
        float ratio = totalScatter / totalGather;
        Assert.IsTrue(ratio > 0.5f && ratio < 6f,
          $"gather/scatter aggregate drift: scatter {totalScatter} vs gather {totalGather} (ratio {ratio}) for v={u.Velocity}");
      }
    }

    [Test]
    public void ContinuousInUnitPosition()
    {
      // density continuity (spec §15 P2: no speed-field popping under slow
      // unit drift) — a tiny step in unit position must change any cell's
      // gathered weight by a bounded amount
      var c = CCConstants.Defaults;
      var rng = new Random(777);
      const float step = 1e-3f;
      for (int trial = 0; trial < 2000; trial++) {
        var pos = rng.NextFloat2(new float2(10f), new float2(20f));
        var vel = rng.NextFloat2Direction() * rng.NextFloat(0.5f, 10f);
        var u0 = new UnitStampData { Position = pos, Velocity = vel, Mass = 1f, FootprintSize = 1f };
        var u1 = new UnitStampData {
          Position = pos + rng.NextFloat2Direction() * step,
          Velocity = vel, Mass = 1f, FootprintSize = 1f,
        };
        // probe a handful of cells around the path
        for (int p = 0; p < 6; p++) {
          var cell = (int2)math.floor(pos + math.normalize(vel) * rng.NextFloat(0f, 8f)
            + rng.NextFloat2(new float2(-1.5f), new float2(1.5f)));
          if (!Gi.InBounds(cell)) continue;
          float w0 = GatherRho(u0, cell, c);
          float w1 = GatherRho(u1, cell, c);
          Assert.LessOrEqual(math.abs(w1 - w0), 0.05f,
            $"stamp discontinuity at cell {cell}: {w0} → {w1} for step {step}");
        }
      }
    }
  }
}
