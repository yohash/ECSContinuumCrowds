using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Pairwise minimum-distance enforcement (spec §13.2, paper §4.5): the
  /// continuum keeps units apart in aggregate, but grid resolution lets
  /// same-cell units overlap; this pass removes the visual artifact.
  ///
  /// A dedicated fine-grained hash (bucket = 2·maxRadius) rebuilt every
  /// frame — deliberately separate from the Phase-2 stamping hash, which
  /// lives only on solve ticks with coarser buckets (do not contort one
  /// hash to serve both).
  ///
  /// Each unit queries the 9 surrounding buckets; for every neighbor within
  /// minDist = rᵢ + rⱼ it applies the symmetric half-push TO ITSELF ONLY —
  /// both units independently compute the same pair and each moves by half
  /// the overlap, so there are no write races and the result is
  /// deterministic given the hash snapshot.
  ///
  /// Paper caveat (preserve): a single iteration per frame is not strictly
  /// convergent and can push non-moving units; the vast majority of the
  /// time it cleanly removes intersections the grid can't resolve, and it
  /// converges across frames.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCAdvectionSystem))]
  [BurstCompile]
  public partial struct CCMinDistanceSystem : ISystem
  {
    private NativeParallelMultiHashMap<int, Neighbor> _map;
    private EntityQuery _unitQuery;

    public void OnCreate(ref SystemState state)
    {
      _unitQuery = SystemAPI.QueryBuilder()
        .WithAll<UnitTag, CCUnit, LocalTransform>().Build();
      state.RequireForUpdate<CCSolveSettings>();
    }

    public void OnDestroy(ref SystemState state)
    {
      if (_map.IsCreated) {
        state.CompleteDependency();
        _map.Dispose();
      }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      int unitCount = _unitQuery.CalculateEntityCount();
      if (unitCount == 0) {
        return;
      }

      // allocate once with headroom; grow only when the population outgrows
      // it (clearing is cheap; reallocation is not — spec §3.5)
      if (!_map.IsCreated || _map.Capacity < unitCount) {
        state.CompleteDependency();
        if (_map.IsCreated) {
          _map.Dispose();
        }
        _map = new NativeParallelMultiHashMap<int, Neighbor>(
          math.max(1024, unitCount * 2), Allocator.Persistent);
      }

      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      float bucketSize = math.max(2f * settings.MaxUnitRadius, 0.05f);

      var clear = new ClearMapJob { Map = _map }.Schedule(state.Dependency);
      var build = new BuildHashJob {
        BucketSize = bucketSize,
        Map = _map.AsParallelWriter(),
      }.ScheduleParallel(_unitQuery, clear);
      state.Dependency = new PushJob {
        BucketSize = bucketSize,
        Map = _map,
      }.ScheduleParallel(_unitQuery, build);
    }

    private struct Neighbor
    {
      public float2 Position; // world XZ
      public float Radius;
    }

    [BurstCompile]
    private struct ClearMapJob : Unity.Jobs.IJob
    {
      public NativeParallelMultiHashMap<int, Neighbor> Map;

      public void Execute() => Map.Clear();
    }

    [BurstCompile]
    [WithAll(typeof(UnitTag))]
    private partial struct BuildHashJob : IJobEntity
    {
      public float BucketSize;
      public NativeParallelMultiHashMap<int, Neighbor>.ParallelWriter Map;

      private void Execute(in LocalTransform transform, in CCUnit unit)
      {
        var pos = transform.Position.xz;
        Map.Add(Bucket(pos, BucketSize), new Neighbor {
          Position = pos,
          Radius = unit.Radius,
        });
      }
    }

    [BurstCompile]
    [WithAll(typeof(UnitTag))]
    private partial struct PushJob : IJobEntity
    {
      public float BucketSize;
      [ReadOnly] public NativeParallelMultiHashMap<int, Neighbor> Map;

      private void Execute(ref LocalTransform transform, in CCUnit unit)
      {
        var pos = transform.Position.xz;
        var cell = (int2)math.floor(pos / BucketSize);
        float2 push = float2.zero;

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int key = Hash(cell + new int2(dx, dy));
            if (!Map.TryGetFirstValue(key, out var other, out var it)) {
              continue;
            }
            do {
              var delta = pos - other.Position;
              float dist = math.length(delta);
              float minDist = unit.Radius + other.Radius;
              // dist ≈ 0 is the unit's own hash entry (or an exactly
              // coincident pair — no separation axis; skip, next frame's
              // continuum motion breaks the tie)
              if (dist > 1e-4f && dist < minDist) {
                push += delta / dist * ((minDist - dist) * 0.5f);
              }
            } while (Map.TryGetNextValue(out other, ref it));
          }
        }

        transform.Position += new float3(push.x, 0f, push.y);
      }
    }

    private static int Bucket(float2 pos, float bucketSize)
      => Hash((int2)math.floor(pos / bucketSize));

    private static int Hash(int2 cell) => (int)math.hash(cell);
  }
}
