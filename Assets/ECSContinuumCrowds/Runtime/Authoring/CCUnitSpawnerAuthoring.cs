using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Bakes a one-shot unit spawner: Count instances of the unit prefab
  /// scattered uniformly in a world-space XZ rect centered on this
  /// GameObject. Consumed by CCUnitSpawnSystem on the first frame.
  /// </summary>
  public class CCUnitSpawnerAuthoring : MonoBehaviour
  {
    public GameObject unitPrefab;
    [Min(1)] public int count = 100;
    [Min(0.01f)] public Vector2 areaSize = new Vector2(10f, 10f);
    [Tooltip("−1 keeps the prefab's groupId; otherwise overrides it.")]
    public int groupIdOverride = -1;
    public uint seed = 12345;

    private class Baker : Baker<CCUnitSpawnerAuthoring>
    {
      public override void Bake(CCUnitSpawnerAuthoring authoring)
      {
        if (authoring.unitPrefab == null) {
          return;
        }
        var entity = GetEntity(TransformUsageFlags.None);
        var center = new float2(
          authoring.transform.position.x, authoring.transform.position.z);
        var half = new float2(authoring.areaSize.x, authoring.areaSize.y) * 0.5f;
        AddComponent(entity, new CCUnitSpawner {
          Prefab = GetEntity(authoring.unitPrefab, TransformUsageFlags.Dynamic),
          Count = authoring.count,
          MinXZ = center - half,
          MaxXZ = center + half,
          GroupIdOverride = authoring.groupIdOverride,
          Seed = authoring.seed,
        });
      }
    }

    private void OnDrawGizmosSelected()
    {
      Gizmos.color = new Color(0.3f, 0.6f, 1f, 0.9f);
      Gizmos.DrawWireCube(
        transform.position, new Vector3(areaSize.x, 0.1f, areaSize.y));
    }
  }
}
