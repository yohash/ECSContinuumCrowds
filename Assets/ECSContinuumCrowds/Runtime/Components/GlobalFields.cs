using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// The one global stamped world map (spec §3.3, Decision D2): dense SoA
  /// NativeArrays on a singleton entity, allocated Persistent at init by
  /// <c>GlobalFieldsInitSystem</c>, disposed when that system is destroyed.
  ///
  /// SoA (one array per field, not an interleaved cell struct) because each
  /// pass touches only a subset of fields; SoA keeps bandwidth spent only on
  /// what's read.
  /// </summary>
  public struct GlobalFields : IComponentData
  {
    public int W;
    public int H;
    /// <summary>World units per cell.</summary>
    public float CellSize;
    /// <summary>World-space XZ position of grid corner (0, 0).</summary>
    public float2 Origin;

    /// <summary>Density ρ — written by stamping, read by the speed field.</summary>
    public NativeArray<float> Rho;
    /// <summary>
    /// Momentum accumulator Σ w·m·v, finalized in place to average velocity
    /// v̄ = acc/ρ (ρ > 0) by the stamping finalize pass.
    /// </summary>
    public NativeArray<float2> VAveAcc;
    /// <summary>
    /// Discomfort g. ⚠ DIVERGENCE (repo, kept): g is clamped to [0,1] at use
    /// and g ≥ 1 is absolutely impassable — boundaries are folded into the
    /// discomfort field instead of a separate mask (paper leaves g unbounded).
    /// </summary>
    public NativeArray<float> Discomfort;
    /// <summary>
    /// Height gradient ∇h, baked from the heightmap at init.
    /// ⚠ NOTE (spec §3.3): the paper stores ∇h on cell faces (MAC grid); the
    /// repo stores a per-cell Vector2 sampled at the into-cell. We follow the
    /// repo: per-cell float2, central differences.
    /// </summary>
    public NativeArray<float2> DH;
    /// <summary>Precomputed g &lt; 1 mask (1 = walkable) — flood-fill fast path.</summary>
    public NativeArray<byte> Walkable;

    public GridIndexer Indexer => new GridIndexer(W, H);

    public bool IsCreated => Rho.IsCreated;

    public void Dispose()
    {
      if (Rho.IsCreated) Rho.Dispose();
      if (VAveAcc.IsCreated) VAveAcc.Dispose();
      if (Discomfort.IsCreated) Discomfort.Dispose();
      if (DH.IsCreated) DH.Dispose();
      if (Walkable.IsCreated) Walkable.Dispose();
    }
  }

  /// <summary>
  /// Global walkability version stamp (spec §5.2, §8.4). Any runtime edit to
  /// discomfort that crosses the g = 1 threshold must bump Version; domain
  /// caches key off it (Decision D6).
  /// </summary>
  public struct CCWalkabilityVersion : IComponentData
  {
    public int Version;
  }
}
