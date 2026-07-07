using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Domain-boundary correctness (spec §8.6): solving over a flood-filled
  /// compact domain must produce the SAME φ and velocity as the full-grid
  /// identity domain whenever the padded region covers the true optimal
  /// paths — treating out-of-domain neighbors as walls is then a no-op.
  /// </summary>
  public class DomainSolveEquivalenceTests
  {
    private const int W = 16;
    private const int H = 16;
    private static readonly GridIndexer Gi = new GridIndexer(W, H);

    // wall column x=8, gap at y=7..8; slight asymmetries to break ties
    private static void Inputs(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      rho = 0.02f * (x % 3);
      g = (x == 8 && y != 7 && y != 8) ? 1f : 0.002f * x + 0.003f * y;
      vAve = Vector2.zero;
      dh = new Vector2(0.01f * x, -0.008f * y);
    }

    [Test]
    public void CompactDomainSolveMatchesFullGridSolve()
    {
      var constants = CCConstants.Defaults;
      var goals = new[] { new int2(13, 8), new int2(13, 7) };

      // ---- full-grid identity solve (the parity-tested baseline) ----
      using var grid = ParityHarness.BuildOurGrid(W, H, Inputs);
      var (fI, cI) = ParityHarness.RunOurFields(grid, constants, 1f, 1f, 1f);
      using (fI)
      using (cI) {
        var (phiI, stateI) = ParityHarness.RunOurFmm(grid, cI, goals, constants);
        using (phiI)
        using (stateI)
        using (var velI = ParityHarness.RunOurVelocity(grid, phiI, fI)) {

          // ---- flood-filled compact domain covering the whole grid ----
          using var walkable = new NativeArray<byte>(Gi.CellCount, Allocator.Temp);
          for (int i = 0; i < Gi.CellCount; i++) {
            walkable[i] = grid.G[i] < 1f ? (byte)1 : (byte)0;
          }
          var cells = new NativeList<int>(64, Allocator.Temp);
          var map = new NativeParallelHashMap<int, int>(Gi.CellCount, Allocator.Temp);
          var neighbors = new NativeList<int4>(64, Allocator.Temp);
          using var goalArr = new NativeArray<int2>(goals, Allocator.Temp);
          CCDomainOps.FloodFill(
            Gi, walkable, goalArr, new int2(0, 0), new int2(W - 1, H - 1),
            W + H, cells, map, neighbors);
          int n = cells.Length;
          Assert.Greater(n, 100, "sanity: domain covers most of the walkable grid");

          // compact field pass + FMM + velocity over the domain
          var fD = new NativeArray<float4>(n, Allocator.Temp);
          var cD = new NativeArray<float4>(n, Allocator.Temp);
          for (int local = 0; local < n; local++) {
            CCFieldOps.ComputeCell(
              local, cells.AsArray(), neighbors.AsArray(), grid.Rho, grid.VAve,
              grid.G, grid.DH, constants, 1f, 1f, 1f, out var fv, out var cv);
            fD[local] = fv;
            cD[local] = cv;
          }
          var phiD = new NativeArray<float>(n, Allocator.Temp);
          var stateD = new NativeArray<byte>(n, Allocator.Temp);
          var heapCells = new NativeArray<int>(n, Allocator.Temp);
          var heapKeys = new NativeArray<float>(n, Allocator.Temp);
          var heapPos = new NativeArray<int>(n, Allocator.Temp);
          CCFmmSolver.Solve(
            n, cells.AsArray(), neighbors.AsArray(), map, Gi, cD, grid.G,
            goalArr, constants.maxWeight, constants.minWeight,
            phiD, stateD, heapCells, heapKeys, heapPos);

          int finiteCells = 0;
          for (int local = 0; local < n; local++) {
            int flat = cells[local];
            float pi = phiI[flat];
            float pd = phiD[local];
            Assert.IsTrue(ParityHarness.Close(pd, pi, 1e-4f),
              $"φ mismatch at {Gi.Coord(flat)}: domain {pd} vs full-grid {pi}");
            if (!float.IsInfinity(pd)) finiteCells++;

            // velocity equivalence through the domain gradient
            var dPhi = CCMath.PotentialGradientCentral(
              phiD, local, Gi.Coord(flat), neighbors[local], Gi);
            var vD = CCMath.VelocityFromGradient(dPhi, fD[local]);
            var vI = velI[flat];
            Assert.IsTrue(
              ParityHarness.Close(vD.x, vI.x, 1e-3f) && ParityHarness.Close(vD.y, vI.y, 1e-3f),
              $"velocity mismatch at {Gi.Coord(flat)}: domain {vD} vs full-grid {vI}");
          }
          Assert.Greater(finiteCells, 100, "sanity: the solve actually propagated");

          fD.Dispose(); cD.Dispose(); phiD.Dispose(); stateD.Dispose();
          heapCells.Dispose(); heapKeys.Dispose(); heapPos.Dispose();
          cells.Dispose(); map.Dispose(); neighbors.Dispose();
        }
      }
    }
  }
}
