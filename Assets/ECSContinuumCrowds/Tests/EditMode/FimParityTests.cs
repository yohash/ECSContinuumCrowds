using NUnit.Framework;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// The standing FMM ↔ FIM agreement test (spec §10.4): both solvers on
  /// identical (domain, C) inputs across the parity scenarios, in BOTH root
  /// modes. Empirical agreement is the acceptance bar.
  ///
  /// Tolerance note: our FMM preserves the repo's accepted-cells-read-as-∞
  /// quirk; FIM converges to the full-information fixed point, so FIM's φ
  /// may sit marginally BELOW FMM's under cost heterogeneity (never above
  /// beyond eps). The relative band below bounds that structural drift; if
  /// it widens on real content, that's exactly what this standing test is
  /// for (triage per spec §10.4).
  /// </summary>
  public class FimParityTests
  {
    private const float RelTol = 1e-2f;

    private static readonly (string name, ParityHarness.CellInputs inputs, int2[] goals)[]
      Scenarios = {
        ("UniformOpen", ParityHarness.UniformOpen, new[] { new int2(1, 1) }),
        ("WallWithGap", ParityHarness.WallWithGap, new[] { new int2(6, 2) }),
        ("DiscomfortRamp", ParityHarness.DiscomfortRamp, new[] { new int2(6, 6) }),
        ("DensityRegimes", ParityHarness.DensityRegimes, new[] { new int2(7, 3) }),
        ("MultiCellGoal", ParityHarness.UniformOpen,
          new[] { new int2(2, 1), new int2(3, 1), new int2(4, 1), new int2(5, 1) }),
        ("UnreachablePocket", ParityHarness.UnreachablePocket, new[] { new int2(1, 1) }),
      };

    [Test]
    public void FimMatchesFmm(
      [Range(0, 5)] int scenarioIndex,
      [Values(FimRootMode.WeightedBlend, FimRootMode.MaxRootWithBlendedPostPass)]
      FimRootMode mode)
    {
      var (name, inputs, goals) = Scenarios[scenarioIndex];
      var constants = CCConstants.Defaults;

      using var grid = ParityHarness.BuildOurGrid(ParityHarness.W, ParityHarness.H, inputs);
      var (f, c) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      using (f)
      using (c) {
        var (phiFmm, stateFmm) = ParityHarness.RunOurFmm(grid, c, goals, constants);
        var (phiFim, sweeps) = ParityHarness.RunOurFim(grid, c, goals, constants, mode);
        using (phiFmm)
        using (stateFmm)
        using (phiFim) {
          Assert.Greater(sweeps, 0, "FIM must have iterated");
          for (int i = 0; i < grid.Gi.CellCount; i++) {
            float a = phiFmm[i];
            float b = phiFim[i];
            Assert.IsFalse(float.IsNaN(b), $"[{name}/{mode}] NaN in FIM φ at {grid.Gi.Coord(i)}");
            // reachability sets must agree exactly
            Assert.AreEqual(float.IsInfinity(a), float.IsInfinity(b),
              $"[{name}/{mode}] reachability mismatch at {grid.Gi.Coord(i)}: FMM {a} FIM {b}");
            if (float.IsInfinity(a)) continue;
            // FIM (full-info fixed point) may be marginally lower, never
            // meaningfully higher (see class doc)
            Assert.LessOrEqual(b, a + RelTol + RelTol * math.abs(a),
              $"[{name}/{mode}] FIM φ above FMM at {grid.Gi.Coord(i)}: FMM {a} FIM {b}");
            Assert.IsTrue(math.abs(a - b) <= RelTol + RelTol * math.abs(a),
              $"[{name}/{mode}] φ drift at {grid.Gi.Coord(i)}: FMM {a} FIM {b}");
          }
        }
      }
    }

    [Test]
    public void GoalCellsStayZeroAndCorridorsSolve()
    {
      // 1-wide corridor: the spec §9.3 degenerate-axis 1-D solution applies
      // to BOTH drivers through the shared EikonalSolve — corridor cells get
      // finite φ from FIM exactly as from FMM
      void Corridor(int x, int y, out float rho, out float g,
        out UnityEngine.Vector2 vAve, out UnityEngine.Vector2 dh)
      {
        rho = 0f;
        g = (y == 3 || y == 5) && x >= 2 ? 1f : 0.001f * x;
        vAve = UnityEngine.Vector2.zero;
        dh = UnityEngine.Vector2.zero;
      }

      var constants = CCConstants.Defaults;
      var goals = new[] { new int2(1, 4) };
      using var grid = ParityHarness.BuildOurGrid(ParityHarness.W, ParityHarness.H, Corridor);
      var (f, c) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      using (f)
      using (c) {
        var (phi, _) = ParityHarness.RunOurFim(grid, c, goals, constants, FimRootMode.WeightedBlend);
        using (phi) {
          Assert.AreEqual(0f, phi[grid.Gi.Flat(1, 4)], 0f, "goal φ must stay exactly 0");
          float corridor = phi[grid.Gi.Flat(5, 4)];
          Assert.IsFalse(float.IsInfinity(corridor) || float.IsNaN(corridor),
            $"FIM must reach 1-wide corridor cells (spec §9.3); got {corridor}");
        }
      }
    }
  }
}
