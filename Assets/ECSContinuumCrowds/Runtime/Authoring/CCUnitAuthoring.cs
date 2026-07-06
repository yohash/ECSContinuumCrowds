using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Bakes a Continuum Crowds unit (spec §3.2): put this on the unit prefab
  /// (with any visual you like) and spawn via CCUnitSpawnerAuthoring or a
  /// plain baked instance.
  /// </summary>
  public class CCUnitAuthoring : MonoBehaviour
  {
    [Min(0.01f)] public float mass = 1f;
    [Tooltip("Physical radius (world units) for the min-distance pass.")]
    [Min(0.01f)] public float radius = 0.4f;
    [Tooltip("Base footprint half-extent in cells; default units use the pure 2×2 splat.")]
    [Min(0f)] public float footprintSize = 1f;
    public int groupId = 0;

    private class Baker : Baker<CCUnitAuthoring>
    {
      public override void Bake(CCUnitAuthoring authoring)
      {
        var entity = GetEntity(TransformUsageFlags.Dynamic);
        AddComponent<UnitTag>(entity);
        AddComponent(entity, new CCUnit {
          Mass = authoring.mass,
          Radius = authoring.radius,
          FootprintSize = authoring.footprintSize,
          GroupId = authoring.groupId,
        });
        AddComponent(entity, new UnitVelocity { Value = float2.zero });
      }
    }
  }
}
