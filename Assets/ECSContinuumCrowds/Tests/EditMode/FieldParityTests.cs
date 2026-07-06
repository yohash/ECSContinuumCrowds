using NUnit.Framework;
using RefConstants = Yohash.ContinuumCrowds.Constants;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Speed/cost field parity vs the reference oracle (spec §15 Phase 1
  /// validation (a)): our CCFieldOps.ComputeCell (the exact FieldJob code
  /// path) against DynamicGlobalFields.InitiateTile per cell per ENSW
  /// direction, over inputs covering all three density regimes, walls,
  /// discomfort, and slopes.
  /// </summary>
  public class FieldParityTests
  {
    private RefConstants _oracleConstants;

    [SetUp]
    public void SetUp() => _oracleConstants = ParityHarness.PushOracleConstants();

    [TearDown]
    public void TearDown() => ParityHarness.PopOracleConstants(_oracleConstants);

    [TestCase(nameof(ParityHarness.UniformOpen))]
    [TestCase(nameof(ParityHarness.WallWithGap))]
    [TestCase(nameof(ParityHarness.DiscomfortRamp))]
    [TestCase(nameof(ParityHarness.DensityRegimes))]
    [TestCase(nameof(ParityHarness.UnreachablePocket))]
    public void FieldsMatchOracle(string scenarioName)
    {
      var scenario = Scenario(scenarioName);

      var tile = ParityHarness.BuildOracleTile(ParityHarness.W, ParityHarness.H, scenario);
      ParityHarness.RunOracleFields(tile);

      using var grid = ParityHarness.BuildOurGrid(ParityHarness.W, ParityHarness.H, scenario);
      var constants = CCConstants.Defaults;
      var (f, c) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      using (f)
      using (c) {
        for (int x = 0; x < ParityHarness.W; x++) {
          for (int y = 0; y < ParityHarness.H; y++) {
            int i = grid.Gi.Flat(x, y);
            for (int d = 0; d < CCMath.NumDirections; d++) {
              Assert.IsTrue(
                ParityHarness.Close(f[i][d], tile.f[x, y][d]),
                $"[{scenarioName}] f mismatch at ({x},{y}) dir {d}: ours {f[i][d]} oracle {tile.f[x, y][d]}");
              Assert.IsTrue(
                ParityHarness.Close(c[i][d], tile.C[x, y][d]),
                $"[{scenarioName}] C mismatch at ({x},{y}) dir {d}: ours {c[i][d]} oracle {tile.C[x, y][d]}");
            }
          }
        }
      }
    }

    private static ParityHarness.CellInputs Scenario(string name) => name switch {
      nameof(ParityHarness.UniformOpen) => ParityHarness.UniformOpen,
      nameof(ParityHarness.WallWithGap) => ParityHarness.WallWithGap,
      nameof(ParityHarness.DiscomfortRamp) => ParityHarness.DiscomfortRamp,
      nameof(ParityHarness.DensityRegimes) => ParityHarness.DensityRegimes,
      _ => ParityHarness.UnreachablePocket,
    };
  }
}
