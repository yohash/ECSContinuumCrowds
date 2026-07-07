using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// The stamping spatial hash + active-cell working set (spec §3.5, §6.2).
  /// Singleton owned (allocated/disposed) by <c>CCSpatialHashSystem</c>;
  /// rebuilt on solve ticks only. This is deliberately a SEPARATE hash from
  /// the min-distance system's per-frame fine hash (spec §13.2: do not
  /// contort one hash to serve both).
  ///
  /// Containers are allocated once with headroom and Clear()ed per rebuild
  /// (clearing is cheap; reallocation is not — spec §3.5). Bucket geometry
  /// is recomputed each tick from CCConstants so editor hot-reload of the
  /// predictive constants resizes correctly.
  /// </summary>
  public struct CCStampingHash : IComponentData
  {
    /// <summary>bucket key (flat bucket index) → unit stamp snapshots.</summary>
    public NativeParallelMultiHashMap<int, UnitStampData> Map;
    /// <summary>1 = bucket contains ≥1 unit this tick.</summary>
    public NativeArray<byte> OccupiedBuckets;
    /// <summary>Occupied dilated by one bucket ring (gather reach ≤ bucket size).</summary>
    public NativeArray<byte> ActiveBuckets;
    /// <summary>Flat indices of cells the gather pass must write this tick.</summary>
    public NativeList<int> ActiveCells;
    public int BucketCells;
    public int2 BucketDims;

    public bool IsCreated => Map.IsCreated;

    public void Dispose()
    {
      if (Map.IsCreated) Map.Dispose();
      if (OccupiedBuckets.IsCreated) OccupiedBuckets.Dispose();
      if (ActiveBuckets.IsCreated) ActiveBuckets.Dispose();
      if (ActiveCells.IsCreated) ActiveCells.Dispose();
    }
  }
}
