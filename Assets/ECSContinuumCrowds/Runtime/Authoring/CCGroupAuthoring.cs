using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Bakes a solve group (spec §3.4). The goal region is a world-space XZ
  /// rect centered on this GameObject; CCGroupInitSystem converts it to
  /// grid GoalCell entries at runtime (clamped in-bounds).
  /// </summary>
  public class CCGroupAuthoring : MonoBehaviour
  {
    public int groupId = 0;

    [Header("Cost weights (per-group overrides of CCConstants C_alpha/beta/gamma)")]
    public float alpha = 1f;
    public float beta = 1f;
    public float gamma = 1f;

    [Header("Goal region (world XZ rect centered on this transform)")]
    [Min(0.01f)] public Vector2 goalSize = new Vector2(2f, 2f);

    private class Baker : Baker<CCGroupAuthoring>
    {
      public override void Bake(CCGroupAuthoring authoring)
      {
        var entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, new CCGroup {
          GroupId = authoring.groupId,
          Alpha = authoring.alpha,
          Beta = authoring.beta,
          Gamma = authoring.gamma,
        });
        var center = new float2(
          authoring.transform.position.x, authoring.transform.position.z);
        var half = new float2(authoring.goalSize.x, authoring.goalSize.y) * 0.5f;
        AddComponent(entity, new CCGroupGoalRect {
          MinXZ = center - half,
          MaxXZ = center + half,
        });
        AddBuffer<GoalCell>(entity);
      }
    }

    private void OnDrawGizmosSelected()
    {
      Gizmos.color = new Color(1f, 0.85f, 0.1f, 0.9f);
      Gizmos.DrawWireCube(
        transform.position, new Vector3(goalSize.x, 0.1f, goalSize.y));
    }
  }
}
