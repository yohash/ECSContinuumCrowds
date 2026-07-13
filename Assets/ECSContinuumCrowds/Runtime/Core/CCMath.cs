using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Shared Continuum Crowds math (spec §16.4): small, inlined,
  /// Burst-compatible static functions used by jobs AND tests, so no logic
  /// is ever duplicated between passes (or between FMM and FIM).
  ///
  /// Direction convention (preserve exactly — repo data format):
  /// anisotropic per-cell fields are float4(E, N, W, S) = (+x, +y, −x, −y);
  /// the direction tables below are index-aligned with those components.
  /// </summary>
  public static partial class CCMath
  {
    public const int NumDirections = 4;
    public const int DirE = 0; // +x
    public const int DirN = 1; // +y
    public const int DirW = 2; // −x
    public const int DirS = 3; // −y

    /// <summary>float2 direction for ENSW component d (repo table Constants.ENSW).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 ENSW(int d)
    {
      // switch over four inlined branches instead of a managed static array
      // (Burst-friendly; spec §2.1)
      switch (d) {
        case DirE: return new float2(1f, 0f);
        case DirN: return new float2(0f, 1f);
        case DirW: return new float2(-1f, 0f);
        default: return new float2(0f, -1f);
      }
    }

    /// <summary>int2 direction for ENSW component d (repo table Constants.ENSWint).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 ENSWint(int d)
    {
      switch (d) {
        case DirE: return new int2(1, 0);
        case DirN: return new int2(0, 1);
        case DirW: return new int2(-1, 0);
        default: return new int2(0, -1);
      }
    }

    // *************************************************************************
    //    World <-> grid mapping
    // *************************************************************************
    // Grid space: cell (x, y) spans [x, x+1) × [y, y+1), center (x+0.5, y+0.5).
    // World mapping uses the XZ plane; 'origin' is the world-space position of
    // grid corner (0, 0).

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 WorldToGrid(float3 world, float2 origin, float cellSize)
      => (world.xz - origin) / cellSize;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float3 GridToWorld(float2 grid, float2 origin, float cellSize, float y = 0f)
    {
      var xz = origin + grid * cellSize;
      return new float3(xz.x, y, xz.y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 CellCenter(int2 cell) => new float2(cell.x + 0.5f, cell.y + 0.5f);

    // *************************************************************************
    //    Density splat kernel (spec §6.4)
    // *************************************************************************
    // ⚠ NOTE (spec §6.4): this is one place we take the paper's concrete
    // formula (§4.1 splat) to fill a hole the repo deliberately left open —
    // the repo delegates the footprint to the consumer (IContinuumUnit.Footprint)
    // while its interface doc quotes the paper's invariant as the contract:
    //
    //   > ...each person should contribute no less than rho_bar to their own
    //   > grid cell, but no more than rho_bar to any neighboring grid cell.
    //
    // Kernel: let (Δx, Δy) be the unit's fractional position relative to the
    // nearest lower-left cell CENTER; deposit onto that 2×2 cell neighborhood
    // {A=lower-left, B=lower-right, C=upper-right, D=upper-left}:
    //   w_A = min(1−Δx, 1−Δy)^λ    w_B = min(Δx, 1−Δy)^λ
    //   w_C = min(Δx, Δy)^λ        w_D = min(1−Δx, Δy)^λ
    // with ρ̄ = 1/2^λ (λ default 2 ⇒ ρ̄ = 0.25).
    //
    // By construction this is continuous in unit position, deposits ≥ ρ̄ to
    // the unit's own cell and ≤ ρ̄ to any neighbor. Combined with the config
    // assert f_rhoMin ≥ ρ̄, an isolated unit always moves at topographical
    // speed (never self-congests). Editor-mode assertion test: SplatKernelTests.

    /// <summary>
    /// The lower-left cell of the 2×2 splat neighborhood for a unit at grid
    /// position <paramref name="pos"/> — the cell whose CENTER is the nearest
    /// center at or below the position on both axes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 SplatBaseCell(float2 pos) => (int2)math.floor(pos - 0.5f);

    /// <summary>
    /// Fractional offset (Δx, Δy) ∈ [0,1)² of a unit at grid position
    /// <paramref name="pos"/> from the base cell's center.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 SplatDelta(float2 pos) => math.frac(pos - 0.5f);

    /// <summary>
    /// Scatter form: weights for the 2×2 neighborhood as float4(A, B, C, D) =
    /// (lower-left, lower-right, upper-right, upper-left) relative to
    /// <see cref="SplatBaseCell"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float4 SplatWeights(float2 pos, float lambda)
    {
      var d = SplatDelta(pos);
      return new float4(
        math.pow(math.min(1f - d.x, 1f - d.y), lambda),  // A
        math.pow(math.min(d.x, 1f - d.y), lambda),       // B
        math.pow(math.min(d.x, d.y), lambda),            // C
        math.pow(math.min(1f - d.x, d.y), lambda));      // D
    }

    /// <summary>
    /// Gather form (stamping inner loop, spec §6.5): the splat weight a unit
    /// at grid position <paramref name="pos"/> deposits into <paramref name="cell"/>.
    /// Zero if the cell is outside the unit's 2×2 support. Must agree exactly
    /// with <see cref="SplatWeights"/> (tested).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float SplatWeight(float2 pos, int2 cell, float lambda)
    {
      var baseCell = SplatBaseCell(pos);
      var rel = cell - baseCell;
      if ((uint)rel.x > 1u | (uint)rel.y > 1u) {
        return 0f;
      }
      var d = SplatDelta(pos);
      float fx = rel.x == 0 ? 1f - d.x : d.x;
      float fy = rel.y == 0 ? 1f - d.y : d.y;
      return math.pow(math.min(fx, fy), lambda);
    }

    /// <summary>ρ̄ for a given splat exponent λ: ρ̄ = 1/2^λ.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float RhoBar(float lambda) => math.pow(0.5f, lambda);

    // *************************************************************************
    //    Height gradient bake (spec §5.1)
    // *************************************************************************

    /// <summary>
    /// Central-difference height gradient at cell <paramref name="c"/>, with
    /// one-sided differences at grid edges. Heights are per-cell samples;
    /// distances are measured in world units (cellSize between adjacent cells).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float2 HeightGradient(
      in NativeArray<float> height,
      in GridIndexer gi,
      int2 c,
      float cellSize
    )
    {
      int xLo = math.max(c.x - 1, 0);
      int xHi = math.min(c.x + 1, gi.W - 1);
      int yLo = math.max(c.y - 1, 0);
      int yHi = math.min(c.y + 1, gi.H - 1);

      float dx = xHi > xLo
        ? (height[gi.Flat(xHi, c.y)] - height[gi.Flat(xLo, c.y)]) / ((xHi - xLo) * cellSize)
        : 0f;
      float dy = yHi > yLo
        ? (height[gi.Flat(c.x, yHi)] - height[gi.Flat(c.x, yLo)]) / ((yHi - yLo) * cellSize)
        : 0f;

      return new float2(dx, dy);
    }
  }
}
