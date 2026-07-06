using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Authors the world grid (spec §5): dimensions, cell size, optional
  /// heightmap and static discomfort textures. Bakes a
  /// <see cref="CCWorldConfig"/> singleton with a blob of per-cell height and
  /// discomfort samples; <c>GlobalFieldsInitSystem</c> turns that into the
  /// live <see cref="GlobalFields"/> at runtime.
  ///
  /// The world origin (grid corner (0,0)) is this GameObject's XZ position at
  /// bake time. Textures must be Read/Write enabled in their import settings;
  /// they are sampled bilinearly at cell centers.
  /// </summary>
  public class CCWorldAuthoring : MonoBehaviour
  {
    [Header("Grid")]
    [Min(1)] public int width = 512;
    [Min(1)] public int height = 512;
    [Min(0.01f)] public float cellSize = 1f;

    [Header("Heightmap (optional — flat world when empty)")]
    [Tooltip("Red channel sampled as height. Must be Read/Write enabled.")]
    public Texture2D heightmap;
    [Tooltip("World-space height corresponding to a heightmap value of 1.")]
    public float heightScale = 1f;

    [Header("Static discomfort (optional)")]
    [Tooltip("Red channel sampled as discomfort g; g ≥ 1 is impassable. Must be Read/Write enabled.")]
    public Texture2D discomfortMap;
    [Tooltip("Multiplier applied to the sampled discomfort value.")]
    public float discomfortScale = 1f;

    private class Baker : Baker<CCWorldAuthoring>
    {
      public override void Bake(CCWorldAuthoring authoring)
      {
        int w = authoring.width;
        int h = authoring.height;
        int cells = w * h;

        var builder = new BlobBuilder(Allocator.Temp);
        ref var root = ref builder.ConstructRoot<CCWorldBakeData>();

        if (authoring.heightmap != null) {
          DependsOn(authoring.heightmap);
          var heights = builder.Allocate(ref root.Height, cells);
          for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
              heights[y * w + x] = authoring.heightmap.GetPixelBilinear(
                (x + 0.5f) / w, (y + 0.5f) / h).r * authoring.heightScale;
            }
          }
        } else {
          builder.Allocate(ref root.Height, 0);
        }

        if (authoring.discomfortMap != null) {
          DependsOn(authoring.discomfortMap);
          var discomfort = builder.Allocate(ref root.Discomfort, cells);
          for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
              discomfort[y * w + x] = authoring.discomfortMap.GetPixelBilinear(
                (x + 0.5f) / w, (y + 0.5f) / h).r * authoring.discomfortScale;
            }
          }
        } else {
          builder.Allocate(ref root.Discomfort, 0);
        }

        var blob = builder.CreateBlobAssetReference<CCWorldBakeData>(Allocator.Persistent);
        builder.Dispose();
        AddBlobAsset(ref blob, out _);

        var entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, new CCWorldConfig {
          W = w,
          H = h,
          CellSize = authoring.cellSize,
          Origin = new float2(
            authoring.transform.position.x,
            authoring.transform.position.z),
          Bake = blob,
        });
      }
    }

    private void OnDrawGizmosSelected()
    {
      var size = new Vector3(width * cellSize, 0f, height * cellSize);
      Gizmos.color = new Color(0.2f, 0.9f, 0.9f, 0.9f);
      Gizmos.DrawWireCube(transform.position + size * 0.5f, size);
    }
  }
}
