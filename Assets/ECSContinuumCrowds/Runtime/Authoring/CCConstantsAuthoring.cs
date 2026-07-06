using Unity.Entities;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Bakes the <see cref="CCConstants"/> singleton from a
  /// <see cref="CCConstantsAsset"/> (falls back to repo defaults when no
  /// asset is assigned). Place one in the world SubScene, typically on the
  /// same GameObject as <see cref="CCWorldAuthoring"/>.
  /// </summary>
  public class CCConstantsAuthoring : MonoBehaviour
  {
    [Tooltip("Constants asset; when empty, repo defaults are baked.")]
    public CCConstantsAsset asset;

    private class Baker : Baker<CCConstantsAuthoring>
    {
      public override void Bake(CCConstantsAuthoring authoring)
      {
        var constants = CCConstants.Defaults;
        if (authoring.asset != null) {
          DependsOn(authoring.asset);
          constants = authoring.asset.ToComponent();
        }

        if (!constants.IsValid) {
          Debug.LogError(
            "[ECSContinuumCrowds] CCConstants failed validation at bake " +
            "(check f_rhoMin ≥ ρ̄ and range orderings); baking anyway — fix the asset.",
            authoring);
        }

        var entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, constants);
      }
    }
  }
}
