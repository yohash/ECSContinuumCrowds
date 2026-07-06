using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Baked, immutable world-bake inputs (spec §5): per-cell height samples
  /// and static authored discomfort. Either array may be empty (length 0)
  /// meaning "flat" / "no discomfort".
  /// </summary>
  public struct CCWorldBakeData
  {
    public BlobArray<float> Height;
    public BlobArray<float> Discomfort;
  }

  /// <summary>
  /// World configuration singleton, baked by <c>CCWorldAuthoring</c>.
  /// Consumed once by <c>GlobalFieldsInitSystem</c> to allocate and bake the
  /// <see cref="GlobalFields"/> singleton.
  /// </summary>
  public struct CCWorldConfig : IComponentData
  {
    public int W;
    public int H;
    public float CellSize;
    /// <summary>World-space XZ position of grid corner (0, 0).</summary>
    public float2 Origin;
    public BlobAssetReference<CCWorldBakeData> Bake;
  }
}
