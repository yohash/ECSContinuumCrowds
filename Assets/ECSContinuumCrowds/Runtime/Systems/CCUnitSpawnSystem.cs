using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// One-shot spawner consumer: instantiates Count copies of the baked unit
  /// prefab at random positions in the authored rect, then removes the
  /// spawner component. Structural work happens once at init — never on the
  /// per-frame path.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCGroupInitSystem))]
  [UpdateBefore(typeof(CCSpatialHashSystem))]
  public partial struct CCUnitSpawnSystem : ISystem
  {
    public void OnUpdate(ref SystemState state)
    {
      var em = state.EntityManager;
      var query = SystemAPI.QueryBuilder().WithAll<CCUnitSpawner>().Build();
      if (query.IsEmptyIgnoreFilter) {
        return;
      }

      var spawners = query.ToEntityArray(Allocator.Temp);
      foreach (var spawnerEntity in spawners) {
        var s = em.GetComponentData<CCUnitSpawner>(spawnerEntity);
        if (s.Prefab == Entity.Null || s.Count <= 0) {
          em.RemoveComponent<CCUnitSpawner>(spawnerEntity);
          continue;
        }

        var rng = new Random(s.Seed != 0 ? s.Seed : 1u);
        var units = em.Instantiate(s.Prefab, s.Count, Allocator.Temp);
        foreach (var unit in units) {
          var xz = rng.NextFloat2(s.MinXZ, s.MaxXZ);
          var t = em.GetComponentData<LocalTransform>(unit);
          t.Position = new float3(xz.x, t.Position.y, xz.y);
          em.SetComponentData(unit, t);

          if (s.GroupIdOverride >= 0) {
            var unitData = em.GetComponentData<CCUnit>(unit);
            unitData.GroupId = s.GroupIdOverride;
            em.SetComponentData(unit, unitData);
          }
        }

        em.RemoveComponent<CCUnitSpawner>(spawnerEntity);
      }
    }
  }

  /// <summary>Baked by CCUnitSpawnerAuthoring.</summary>
  public struct CCUnitSpawner : IComponentData
  {
    public Entity Prefab;
    public int Count;
    public float2 MinXZ;
    public float2 MaxXZ;
    public int GroupIdOverride; // −1 = keep the prefab's GroupId
    public uint Seed;
  }
}
