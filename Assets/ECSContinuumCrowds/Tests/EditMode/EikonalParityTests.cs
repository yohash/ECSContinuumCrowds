using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Yohash.ContinuumCrowds;
using RefConstants = Yohash.ContinuumCrowds.Constants;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// φ and velocity parity vs the reference oracle on handcrafted 8×8 grids
  /// — the Phase-1 exit bar (spec §15 validation (a)). Both sides run their
  /// COMPLETE chains from identical raw ρ/g/v̄/∇h inputs: oracle
  /// InitiateTile → EikonalSolver; ours CCFieldOps → CCFmmSolver →
  /// central-gradient velocity.
  /// </summary>
  public class EikonalParityTests
  {
    private RefConstants _oracleConstants;

    [SetUp]
    public void SetUp() => _oracleConstants = ParityHarness.PushOracleConstants();

    [TearDown]
    public void TearDown() => ParityHarness.PopOracleConstants(_oracleConstants);

    private static readonly (string name, ParityHarness.CellInputs inputs, int2[] goals)[]
      Scenarios = {
        ("UniformOpen", ParityHarness.UniformOpen,
          new[] { new int2(1, 1) }),
        ("WallWithGap", ParityHarness.WallWithGap,
          new[] { new int2(6, 2) }),
        ("DiscomfortRamp", ParityHarness.DiscomfortRamp,
          new[] { new int2(6, 6) }),
        ("DensityRegimes", ParityHarness.DensityRegimes,
          new[] { new int2(7, 3) }),
        ("MultiCellGoal", ParityHarness.UniformOpen,
          new[] { new int2(2, 1), new int2(3, 1), new int2(4, 1), new int2(5, 1) }),
        ("UnreachablePocket", ParityHarness.UnreachablePocket,
          new[] { new int2(1, 1) }),
      };

    [Test]
    public void PhiMatchesOracle(
      [Range(0, 5)] int scenarioIndex)
    {
      var (name, inputs, goals) = Scenarios[scenarioIndex];
      RunBoth(inputs, goals,
        out var oraclePhi, out _, out var ourPhi, out _, out var grid);

      using (ourPhi) {
        int infiniteCells = 0;
        for (int x = 0; x < ParityHarness.W; x++) {
          for (int y = 0; y < ParityHarness.H; y++) {
            float ours = ourPhi[grid.Gi.Flat(x, y)];
            float theirs = oraclePhi[x, y];
            Assert.IsTrue(
              ParityHarness.Close(ours, theirs, 2e-3f),
              $"[{name}] φ mismatch at ({x},{y}): ours {ours} oracle {theirs}");
            Assert.IsFalse(float.IsNaN(ours), $"[{name}] NaN in our φ at ({x},{y})");
            if (float.IsInfinity(ours)) infiniteCells++;
          }
        }
        if (name == "UnreachablePocket") {
          // the walled 2×2 interior (plus the walls themselves) must stay ∞
          Assert.GreaterOrEqual(infiniteCells, 4, "pocket interior should be unreachable");
        }
      }
      grid.Dispose();
    }

    [Test]
    public void VelocityMatchesOracle(
      [Range(0, 5)] int scenarioIndex)
    {
      var (name, inputs, goals) = Scenarios[scenarioIndex];
      RunBoth(inputs, goals,
        out _, out var oracleVelocity, out var ourPhi, out var ourF, out var grid);

      using (ourPhi)
      using (ourF)
      using (var ourVelocity = ParityHarness.RunOurVelocity(grid, ourPhi, ourF)) {
        for (int x = 0; x < ParityHarness.W; x++) {
          for (int y = 0; y < ParityHarness.H; y++) {
            var ours = ourVelocity[grid.Gi.Flat(x, y)];
            var theirs = oracleVelocity[x, y];
            Assert.IsTrue(
              ParityHarness.Close(ours.x, theirs.x, 5e-3f)
              && ParityHarness.Close(ours.y, theirs.y, 5e-3f),
              $"[{name}] velocity mismatch at ({x},{y}): ours {ours} oracle ({theirs.x},{theirs.y})");
          }
        }
      }
      grid.Dispose();
    }

    /// <summary>
    /// ⚠ Documented divergence (CCMath.EikonalSolve degenerate-axis note):
    /// in a 1-wide corridor the oracle's float arithmetic produces NaN
    /// proposals that compare false and silently skip the update, leaving
    /// corridor cells at φ = ∞; the spec (§9.3) mandates the 1-D solution
    /// instead, so OUR solver assigns finite φ and units can traverse the
    /// corridor. This test pins BOTH behaviors so the divergence stays
    /// intentional and visible.
    /// </summary>
    [Test]
    public void OneWideCorridor_DivergesFromOracle()
    {
      // walls at y=3 and y=5 make row y=4 a 1-wide E-W corridor
      void Corridor(int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
      {
        rho = 0f;
        g = (y == 3 || y == 5) && x >= 2 ? 1f : 0.001f * x;
        vAve = Vector2.zero;
        dh = Vector2.zero;
      }

      var goals = new[] { new int2(1, 4) };
      RunBoth(Corridor, goals,
        out var oraclePhi, out _, out var ourPhi, out _, out var grid);

      using (ourPhi) {
        // deep inside the corridor, both x-neighbors of the next cell are
        // walls only for cells past x=2; check a mid-corridor cell
        var probe = new int2(5, 4);
        float ours = ourPhi[grid.Gi.Flat(probe)];
        float theirs = oraclePhi[probe.x, probe.y];
        Assert.IsFalse(float.IsInfinity(ours) || float.IsNaN(ours),
          $"our solver must reach the corridor cell (spec §9.3 1-D fallback); got {ours}");
        Assert.IsTrue(float.IsInfinity(theirs),
          $"oracle expected to NaN-skip walled-axis cells and leave φ=∞; got {theirs} — " +
          "if this fails the repo behavior changed and the divergence note should be revisited");
      }
      grid.Dispose();
    }

    private static void RunBoth(
      ParityHarness.CellInputs inputs,
      int2[] goals,
      out float[,] oraclePhi,
      out Vector2[,] oracleVelocity,
      out Unity.Collections.NativeArray<float> ourPhi,
      out Unity.Collections.NativeArray<float4> ourF,
      out ParityHarness.OurGrid grid)
    {
      // oracle chain
      var tile = ParityHarness.BuildOracleTile(ParityHarness.W, ParityHarness.H, inputs);
      ParityHarness.RunOracleFields(tile);
      var goalList = new List<Location>();
      foreach (var gc in goals) {
        goalList.Add(new Location(gc.x, gc.y));
      }
      (oraclePhi, oracleVelocity) = ParityHarness.RunOracleEikonal(tile, goalList);

      // our chain
      grid = ParityHarness.BuildOurGrid(ParityHarness.W, ParityHarness.H, inputs);
      var constants = CCConstants.Defaults;
      var (f, c) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      ourF = f;
      using (c) {
        (ourPhi, _) = ParityHarness.RunOurFmm(grid, c, goals, constants);
      }
    }
  }
}
