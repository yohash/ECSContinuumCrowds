using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Stamping spatial hash rebuild (spec §6.2 Job A + active-cell
  /// determination): on solve ticks, snapshots every unit into
  /// <see cref="CCStampingHash"/> (bucket = ceil(R_max) cells, spec §6.3) and
  /// derives the active-cell list — the cells covered by occupied buckets
  /// dilated by one ring. Cells outside the active set are cleared by the
  /// stamping pass and receive no gather work, so stamping cost scales with
  /// crowd extent, not grid size.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCUnitSpawnSystem))]
  [BurstCompile]
  public partial struct CCSpatialHashSystem : ISystem
  {
    private EntityQuery _unitQuery;
    private EntityQuery _hashQuery;

    public void OnCreate(ref SystemState state)
    {
      _unitQuery = SystemAPI.QueryBuilder()
        .WithAll<UnitTag, CCUnit, UnitVelocity, LocalTransform>().Build();
      _hashQuery = state.GetEntityQuery(ComponentType.ReadWrite<CCStampingHash>());
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveTick>();
      state.RequireForUpdate<CCConstants>();
    }

    public void OnDestroy(ref SystemState state)
    {
      if (!_hashQuery.IsEmptyIgnoreFilter) {
        state.CompleteDependency();
        _hashQuery.GetSingleton<CCStampingHash>().Dispose();
      }
    }

    public void OnUpdate(ref SystemState state)
    {
      if (!SystemAPI.GetSingleton<CCSolveTick>().SolveThisFrame) {
        return;
      }

      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var constants = SystemAPI.GetSingleton<CCConstants>();
      var gi = fields.Indexer;
      int unitCount = _unitQuery.CalculateEntityCount();

      int bucketCells = CCStampOps.BucketCells(constants, fields.CellSize);
      var bucketDims = CCStampOps.BucketDims(gi, bucketCells);
      int bucketCount = bucketDims.x * bucketDims.y;

      // create / grow the persistent containers (spec §3.5: allocate once,
      // Clear per rebuild; reallocation only on population growth or a
      // bucket-geometry change from constants hot-reload)
      if (_hashQuery.IsEmptyIgnoreFilter) {
        state.EntityManager.AddComponentData(
          state.EntityManager.CreateEntity(), Allocate(unitCount, bucketCount, gi));
      }
      var hashRW = SystemAPI.GetSingletonRW<CCStampingHash>();
      if (hashRW.ValueRO.Map.Capacity < unitCount
        || hashRW.ValueRO.OccupiedBuckets.Length != bucketCount) {
        state.CompleteDependency();
        hashRW.ValueRW.Dispose();
        hashRW.ValueRW = Allocate(unitCount, bucketCount, gi);
      }
      hashRW.ValueRW.BucketCells = bucketCells;
      hashRW.ValueRW.BucketDims = bucketDims;
      var hash = hashRW.ValueRO;

      var clear = new ClearHashJob {
        Map = hash.Map,
        Occupied = hash.OccupiedBuckets,
      }.Schedule(state.Dependency);

      var build = new BuildHashJob {
        Origin = fields.Origin,
        CellSize = fields.CellSize,
        BucketCells = bucketCells,
        BucketDims = bucketDims,
        Map = hash.Map.AsParallelWriter(),
        Occupied = hash.OccupiedBuckets,
      }.ScheduleParallel(_unitQuery, clear);

      state.Dependency = new BuildActiveCellsJob {
        Gi = gi,
        BucketCells = bucketCells,
        BucketDims = bucketDims,
        Occupied = hash.OccupiedBuckets,
        Active = hash.ActiveBuckets,
        ActiveCells = hash.ActiveCells,
      }.Schedule(build);
    }

    private static CCStampingHash Allocate(int unitCount, int bucketCount, in GridIndexer gi)
    {
      return new CCStampingHash {
        Map = new NativeParallelMultiHashMap<int, UnitStampData>(
          math.max(1024, unitCount * 2), Allocator.Persistent),
        OccupiedBuckets = new NativeArray<byte>(bucketCount, Allocator.Persistent),
        ActiveBuckets = new NativeArray<byte>(bucketCount, Allocator.Persistent),
        ActiveCells = new NativeList<int>(gi.CellCount, Allocator.Persistent),
      };
    }

    [BurstCompile]
    private struct ClearHashJob : Unity.Jobs.IJob
    {
      public NativeParallelMultiHashMap<int, UnitStampData> Map;
      public NativeArray<byte> Occupied;

      public void Execute()
      {
        Map.Clear();
        for (int i = 0; i < Occupied.Length; i++) {
          Occupied[i] = 0;
        }
      }
    }

    /// <summary>Spec §6.2 Job A: snapshot units into the hash, tracking occupied buckets.</summary>
    [BurstCompile]
    [WithAll(typeof(UnitTag))]
    private partial struct BuildHashJob : IJobEntity
    {
      public float2 Origin;
      public float CellSize;
      public int BucketCells;
      public int2 BucketDims;
      public NativeParallelMultiHashMap<int, UnitStampData>.ParallelWriter Map;
      // concurrent writes all store the identical value 1 — benign by
      // construction; the attribute only waives the ParallelFor range check
      [NativeDisableParallelForRestriction] public NativeArray<byte> Occupied;

      private void Execute(
        in LocalTransform transform, in CCUnit unit, in UnitVelocity velocity)
      {
        var gridPos = CCMath.WorldToGrid(transform.Position, Origin, CellSize);
        var bucket = math.clamp(
          CCStampOps.BucketOf(gridPos, BucketCells), int2.zero, BucketDims - 1);
        int key = CCStampOps.BucketKey(bucket, BucketDims);
        Map.Add(key, new UnitStampData {
          Position = gridPos,
          Velocity = velocity.Value,
          Mass = unit.Mass,
          FootprintSize = unit.FootprintSize,
        });
        Occupied[key] = 1;
      }
    }

    /// <summary>Dilate occupied buckets by one ring and emit the covered cells.</summary>
    [BurstCompile]
    private struct BuildActiveCellsJob : Unity.Jobs.IJob
    {
      public GridIndexer Gi;
      public int BucketCells;
      public int2 BucketDims;
      [ReadOnly] public NativeArray<byte> Occupied;
      public NativeArray<byte> Active;
      public NativeList<int> ActiveCells;

      public void Execute()
      {
        CCStampOps.MarkActiveBuckets(Occupied, Active, BucketDims);
        CCStampOps.EmitActiveCells(Active, BucketDims, BucketCells, Gi, ActiveCells);
      }
    }
  }
}
