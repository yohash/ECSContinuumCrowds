#if UNITY_EDITOR
using Unity.Entities;
using UnityEditor;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Editor-only hot reload (spec §5.3): copies the values of the first
  /// <see cref="CCConstantsAsset"/> found in the project into the
  /// <see cref="CCConstants"/> singleton every frame, so tuning in the
  /// Inspector during Play Mode takes effect live. Compiled out of player
  /// builds entirely.
  ///
  /// This is deliberately a managed SystemBase — it touches AssetDatabase
  /// and a ScriptableObject. It never runs in a build; every hot-path system
  /// remains an unmanaged ISystem.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup), OrderFirst = true)]
  public partial class CCConstantsHotReloadSystem : SystemBase
  {
    private CCConstantsAsset _asset;
    private bool _searched;

    protected override void OnCreate()
    {
      RequireForUpdate<CCConstants>();
    }

    protected override void OnUpdate()
    {
      if (_asset == null) {
        if (_searched) return;
        _searched = true;
        var guids = AssetDatabase.FindAssets("t:CCConstantsAsset");
        if (guids.Length == 0) return;
        if (guids.Length > 1) {
          Debug.LogWarning(
            "[ECSContinuumCrowds] Multiple CCConstantsAsset assets found; " +
            $"hot-reloading from '{AssetDatabase.GUIDToAssetPath(guids[0])}'.");
        }
        _asset = AssetDatabase.LoadAssetAtPath<CCConstantsAsset>(
          AssetDatabase.GUIDToAssetPath(guids[0]));
        if (_asset == null) return;
      }

      SystemAPI.SetSingleton(_asset.ToComponent());
    }
  }
}
#endif
