using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// The single indexing utility for the global grid (spec §3.3, Decision D2).
  ///
  /// ALL grid indexing — field passes, flood fill, eikonal, advection sampling,
  /// debug tools — must go through this struct. It is the seam behind which
  /// paged storage could be introduced in Phase 5+ without touching any
  /// algorithm code. No behavior may ever depend on a page boundary.
  ///
  /// Convention: cell (x, y) with x ∈ [0, W), y ∈ [0, H); flat index i = y * W + x.
  /// Grid space: cell (x, y) spans [x, x+1) × [y, y+1); its center is (x+0.5, y+0.5).
  /// </summary>
  public readonly struct GridIndexer
  {
    public readonly int W;
    public readonly int H;

    public GridIndexer(int w, int h)
    {
      W = w;
      H = h;
    }

    public int CellCount => W * H;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Flat(int2 c) => c.y * W + c.x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Flat(int x, int y) => y * W + x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool InBounds(int2 c) => (uint)c.x < (uint)W & (uint)c.y < (uint)H;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool InBounds(int x, int y) => (uint)x < (uint)W & (uint)y < (uint)H;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int2 Coord(int flat) => new int2(flat % W, flat / W);
  }
}
