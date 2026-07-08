using System.Diagnostics;
using System.Text;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// The FMM/FIM crossover benchmark harness — a Phase-4 DELIVERABLE
  /// (spec §10.3/§14.5, "part of the deliverable, not a side quest"): solve
  /// identical domains of graded sizes with both solvers across
  /// representative cost-field heterogeneity (uniform / corridor /
  /// congested), record wall time, and log the table used to fit the
  /// crossover and bake FimThresholdCells for the target hardware class.
  ///
  /// [Explicit]: run it on demand from the Test Runner, not in CI. Two big
  /// caveats to read the numbers correctly:
  /// 1. This measures SINGLE-THREADED wall time for both solvers (the
  ///    edit-mode serial FIM driver) — it captures FIM's total-work overhead
  ///    factor vs FMM. The production win is FIM's parallel width; divide
  ///    FIM's serial time by realistic worker counts (× parallel efficiency)
  ///    when fitting the crossover, then validate in play mode with the
  ///    Profiler timeline.
  /// 2. Editor test code runs WITHOUT Burst on these direct calls; absolute
  ///    times are inflated but the FMM:FIM ratio — what the crossover fit
  ///    needs — is representative.
  /// </summary>
  public class CrossoverBenchmarkHarness
  {
    private enum Field { Uniform, Corridor, Congested }

    [Test]
    [Explicit("Benchmark — run on demand; logs the crossover table (spec §10.3)")]
    public void FmmVsFimCrossoverTable()
    {
      var constants = CCConstants.Defaults;
      var sb = new StringBuilder();
      sb.AppendLine("FMM vs FIM crossover table (serial wall time, ms; ratio = FIM/FMM):");
      sb.AppendLine("size      cells    field      FMM(ms)   FIM(ms)   sweeps  ratio");

      foreach (int side in new[] { 64, 128, 256 }) {
        foreach (Field field in new[] { Field.Uniform, Field.Corridor, Field.Congested }) {
          RunCase(side, field, constants, sb);
        }
      }
      UnityEngine.Debug.Log(sb.ToString());
    }

    private static void RunCase(int side, Field field, in CCConstants constants, StringBuilder sb)
    {
      var gi = new GridIndexer(side, side);
      int cells = gi.CellCount;

      // synthetic inputs per heterogeneity class
      ParityHarness.CellInputs inputs = field switch {
        Field.Uniform => (int x, int y, out float rho, out float g,
            out UnityEngine.Vector2 vAve, out UnityEngine.Vector2 dh) => {
          rho = 0f; g = 0.0005f * x + 0.0007f * y;
          vAve = UnityEngine.Vector2.zero; dh = UnityEngine.Vector2.zero;
        },
        Field.Corridor => (int x, int y, out float rho, out float g,
            out UnityEngine.Vector2 vAve, out UnityEngine.Vector2 dh) => {
          // wall stripes every 16 rows with staggered gaps → long bending characteristics
          bool wall = y % 16 == 8 && (x % 32) != (y / 16 * 8) % 32;
          rho = 0f; g = wall ? 1f : 0.0005f * x;
          vAve = UnityEngine.Vector2.zero; dh = UnityEngine.Vector2.zero;
        },
        _ => (int x, int y, out float rho, out float g,
            out UnityEngine.Vector2 vAve, out UnityEngine.Vector2 dh) => {
          // congested: dense opposing-flow stripes → strong C heterogeneity
          rho = (x / 8) % 2 == 0 ? 0.95f : 0.1f;
          g = 0.3f * math.abs(math.sin(x * 0.37f + y * 0.51f));
          vAve = new UnityEngine.Vector2((y / 8) % 2 == 0 ? 2.5f : -2.5f, 0.4f);
          dh = UnityEngine.Vector2.zero;
        },
      };

      using var grid = ParityHarness.BuildOurGrid(side, side, inputs);
      var (f, c) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      var goals = new[] { new int2(1, 1), new int2(2, 1) };

      using (f)
      using (c) {
        // warm + measure FMM (best of 3)
        double fmmMs = double.MaxValue;
        for (int rep = 0; rep < 3; rep++) {
          var sw = Stopwatch.StartNew();
          var (phi, state) = ParityHarness.RunOurFmm(grid, c, goals, constants);
          sw.Stop();
          phi.Dispose();
          state.Dispose();
          fmmMs = math.min(fmmMs, sw.Elapsed.TotalMilliseconds);
        }

        double fimMs = double.MaxValue;
        int sweeps = 0;
        for (int rep = 0; rep < 3; rep++) {
          var sw = Stopwatch.StartNew();
          var (phi, s) = ParityHarness.RunOurFim(
            grid, c, goals, constants, FimRootMode.WeightedBlend, maxSweeps: 65536);
          sw.Stop();
          phi.Dispose();
          sweeps = s;
          fimMs = math.min(fimMs, sw.Elapsed.TotalMilliseconds);
        }

        sb.AppendLine(
          $"{side}x{side}   {cells,7}  {field,-9}  {fmmMs,8:F2}  {fimMs,8:F2}  {sweeps,6}  {fimMs / math.max(fmmMs, 1e-6):F2}");
      }
    }
  }
}
