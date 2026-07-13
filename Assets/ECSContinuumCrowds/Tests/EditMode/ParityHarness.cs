using System;
using System.Collections.Generic;
using System.Reflection;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Yohash.ContinuumCrowds;
using RefConstants = Yohash.ContinuumCrowds.Constants;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Shared plumbing for the reference-oracle parity suite (spec §15 Phase 1
  /// validation (a)): builds identical inputs for the managed oracle
  /// (yohash/ContinuumCrowds, verbatim in ReferenceOracle/) and for our
  /// Burst implementation, runs both, and compares.
  /// </summary>
  public static class ParityHarness
  {
    /// <summary>Per-cell input sampler: returns (ρ, g, v̄, ∇h) for (x, y).</summary>
    public delegate void CellInputs(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh);

    // -----------------------------------------------------------------
    //  Oracle constants lifecycle (repo Constants is a ScriptableObject
    //  singleton; Values NREs when Instance is unset)
    // -----------------------------------------------------------------
    public static RefConstants PushOracleConstants()
    {
      var c = ScriptableObject.CreateInstance<RefConstants>();
      // field initializers already match CCConstants.Defaults (verified by
      // CCConstantsTests.DefaultsMatchReferenceRepo)
      RefConstants.Instance = c;
      return c;
    }

    public static void PopOracleConstants(RefConstants c)
    {
      RefConstants.Instance = null;
      UnityEngine.Object.DestroyImmediate(c);
    }

    // -----------------------------------------------------------------
    //  Input construction
    // -----------------------------------------------------------------
    public static OracleTile BuildOracleTile(int w, int h, CellInputs inputs)
    {
      var tile = new OracleTile(w, h);
      for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
          inputs(x, y, out var rho, out var g, out var vAve, out var dh);
          tile.rho[x, y] = rho;
          tile.g[x, y] = g;
          tile.vAve[x, y] = vAve;
          tile.dh[x, y] = dh;
        }
      }
      return tile;
    }

    public struct OurGrid : IDisposable
    {
      public GridIndexer Gi;
      public NativeArray<float> Rho;
      public NativeArray<float> G;
      public NativeArray<float2> VAve;
      public NativeArray<float2> DH;

      public void Dispose()
      {
        Rho.Dispose();
        G.Dispose();
        VAve.Dispose();
        DH.Dispose();
      }
    }

    public static OurGrid BuildOurGrid(int w, int h, CellInputs inputs)
    {
      var gi = new GridIndexer(w, h);
      var grid = new OurGrid {
        Gi = gi,
        Rho = new NativeArray<float>(gi.CellCount, Allocator.Temp),
        G = new NativeArray<float>(gi.CellCount, Allocator.Temp),
        VAve = new NativeArray<float2>(gi.CellCount, Allocator.Temp),
        DH = new NativeArray<float2>(gi.CellCount, Allocator.Temp),
      };
      for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
          inputs(x, y, out var rho, out var g, out var vAve, out var dh);
          int i = gi.Flat(x, y);
          grid.Rho[i] = rho;
          grid.G[i] = g;
          grid.VAve[i] = new float2(vAve.x, vAve.y);
          grid.DH[i] = new float2(dh.x, dh.y);
        }
      }
      return grid;
    }

    // -----------------------------------------------------------------
    //  Pipeline runners
    // -----------------------------------------------------------------

    /// <summary>Oracle field pass: InitiateTile computes f and C from the tile's current ρ/v̄/g/∇h.</summary>
    public static void RunOracleFields(OracleTile tile)
    {
      var tiles = tile.AsTiles();
      DynamicGlobalFields.InitiateTile(tile, ref tiles, tile.Hash);
    }

    /// <summary>Our field pass: the exact per-cell function the FieldJob runs.</summary>
    public static (NativeArray<float4> f, NativeArray<float4> c) RunOurFields(
      in OurGrid grid, in CCConstants constants, float alpha, float beta, float gamma)
    {
      var f = new NativeArray<float4>(grid.Gi.CellCount, Allocator.Temp);
      var c = new NativeArray<float4>(grid.Gi.CellCount, Allocator.Temp);
      for (int i = 0; i < grid.Gi.CellCount; i++) {
        CCFieldOps.ComputeCell(
          i, grid.Gi, grid.Rho, grid.VAve, grid.G, grid.DH,
          constants, alpha, beta, gamma, out var fv, out var cv);
        f[i] = fv;
        c[i] = cv;
      }
      return (f, c);
    }

    /// <summary>Oracle eikonal + velocity; φ read via reflection (private field).</summary>
    public static (float[,] phi, Vector2[,] velocity) RunOracleEikonal(
      OracleTile tile, List<Location> goals)
    {
      var solver = new EikonalSolver();
      EikonalSolver done = null;
      solver.Solve(tile, goals, s => done = s);
      var phiField = typeof(EikonalSolver)
        .GetField("phi", BindingFlags.NonPublic | BindingFlags.Instance);
      return ((float[,])phiField.GetValue(done), done.Velocity);
    }

    /// <summary>Our FMM: the exact CCFmmSolver.Solve the eikonal job runs.</summary>
    public static (NativeArray<float> phi, NativeArray<byte> state) RunOurFmm(
      in OurGrid grid, in NativeArray<float4> c, int2[] goals, in CCConstants constants)
    {
      int cells = grid.Gi.CellCount;
      var phi = new NativeArray<float>(cells, Allocator.Temp);
      var state = new NativeArray<byte>(cells, Allocator.Temp);
      var heapCells = new NativeArray<int>(cells, Allocator.Temp);
      var heapKeys = new NativeArray<float>(cells, Allocator.Temp);
      var heapPos = new NativeArray<int>(cells, Allocator.Temp);
      var goalArray = new NativeArray<int2>(goals, Allocator.Temp);
      CCFmmSolver.Solve(
        grid.Gi, c, grid.G, goalArray, constants.maxWeight, constants.minWeight,
        phi, state, heapCells, heapKeys, heapPos);
      return (phi, state);
    }

    /// <summary>Our velocity derivation (central-repo scheme) from φ and f.</summary>
    public static NativeArray<float2> RunOurVelocity(
      in OurGrid grid, in NativeArray<float> phi, in NativeArray<float4> f)
    {
      var velocity = new NativeArray<float2>(grid.Gi.CellCount, Allocator.Temp);
      for (int i = 0; i < grid.Gi.CellCount; i++) {
        var dPhi = CCMath.PotentialGradientCentral(phi, grid.Gi, grid.Gi.Coord(i));
        velocity[i] = CCMath.VelocityFromGradient(dPhi, f[i]);
      }
      return velocity;
    }

    // -----------------------------------------------------------------
    //  Comparison
    // -----------------------------------------------------------------
    public static bool Close(float ours, float theirs, float tol = 1e-3f)
    {
      if (float.IsInfinity(ours) || float.IsInfinity(theirs)) {
        return float.IsInfinity(ours) && float.IsInfinity(theirs);
      }
      return math.abs(ours - theirs) <= tol + tol * math.abs(theirs);
    }

    // -----------------------------------------------------------------
    //  Shared scenarios (spec §15: handcrafted 8×8 grids). Inputs are kept
    //  slightly asymmetric so equal-priority heap ties — whose pop order is
    //  implementation-defined — can't mask real differences.
    // -----------------------------------------------------------------
    public const int W = 8;
    public const int H = 8;

    public static void UniformOpen(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      rho = 0f;
      g = 0.001f * x + 0.0013f * y; // tiny tilt to break symmetry ties
      vAve = Vector2.zero;
      dh = Vector2.zero;
    }

    /// <summary>Wall column at x=4, gap at y=5..6 (no 1-wide corridors).</summary>
    public static void WallWithGap(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      rho = 0f;
      g = (x == 4 && y != 5 && y != 6) ? 1f : 0.001f * x + 0.0017f * y;
      vAve = Vector2.zero;
      dh = Vector2.zero;
    }

    public static void DiscomfortRamp(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      rho = 0f;
      g = 0.05f * x + 0.09f * y; // max 0.98 < 1, everything walkable
      vAve = Vector2.zero;
      dh = new Vector2(0.1f, -0.15f);
    }

    /// <summary>Exercises all three speed-field density regimes + flow clamp.</summary>
    public static void DensityRegimes(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      // stripes: low (topographical), medium (lerp), high (flow) densities
      rho = (x % 3) switch { 0 => 0.1f, 1 => 0.55f, _ => 0.9f } + 0.011f * y;
      g = 0.02f * y;
      vAve = new Vector2(1.5f - 0.2f * y, 0.7f * (x % 2 == 0 ? 1f : -1f));
      dh = new Vector2(0.03f * x, -0.02f * y);
    }

    /// <summary>Fully-walled 2×2 pocket at (5..6, 5..6): φ must stay ∞ inside.</summary>
    public static void UnreachablePocket(
      int x, int y, out float rho, out float g, out Vector2 vAve, out Vector2 dh)
    {
      bool wall =
        (x >= 4 && x <= 7 && (y == 4 || y == 7)) ||
        ((x == 4 || x == 7) && y >= 4 && y <= 7);
      rho = 0f;
      g = wall ? 1f : 0.001f * x + 0.0009f * y;
      vAve = Vector2.zero;
      dh = Vector2.zero;
    }
  }
}
