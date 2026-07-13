using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Scene-view debug visualizer for the Continuum Crowds fields
  /// (spec §15 Phase 0 — "invest here; every later phase depends on seeing
  /// the fields"). Drop on any GameObject; it draws a window of cells
  /// centered on this transform, so move the GameObject to inspect regions
  /// of a large grid.
  ///
  /// Scalar modes render a heat ramp (dark blue → cyan → yellow → red, with
  /// impassable/∞ cells in black); vector modes render arrows. Per-group
  /// modes (φ, velocity, gradient diff) read the group selected by
  /// <see cref="groupIndex"/>.
  ///
  /// GradientDiff is the shockline diagnostic (Decision D10): it renders the
  /// angular difference between the central-repo and upwind-paper gradients
  /// computed from the SAME live φ — kinks in φ behind obstacles / along
  /// medial axes light up. It doubles as the best available φ-field
  /// debugging tool.
  /// </summary>
  [ExecuteAlways]
  public class CCFieldVisualizer : MonoBehaviour
  {
    public enum FieldMode
    {
      None,
      Density,          // ρ
      AverageVelocity,  // v̄ (finalized) — arrows; accumulator/ρ before finalize
      Discomfort,       // g
      Walkable,         // g < 1 mask
      HeightGradient,   // ∇h — arrows
      Potential,        // φ (per group) — heat ramp, ∞ = black
      GroupVelocity,    // group velocity field — heading-colored arrows
      GradientDiff,     // central-repo vs upwind-paper ∇φ angle — shockline debug
    }

    [Header("What to draw")]
    public FieldMode mode = FieldMode.Discomfort;
    [Tooltip("Which group (index into the CCGroup query) for per-group modes.")]
    [Min(0)] public int groupIndex = 0;

    [Header("Window (cells) around this transform")]
    [Min(1)] public int windowRadius = 48;

    [Header("Scalar ramp")]
    [Tooltip("Auto-normalize the color ramp to the max value in the window.")]
    public bool autoScale = true;
    [Tooltip("Fixed ramp maximum when autoScale is off.")]
    public float fixedMax = 1f;

    [Header("Vector arrows")]
    [Tooltip("World length of an arrow at magnitude 1 (arrows are clamped to cell size).")]
    public float arrowScale = 0.5f;

    [Header("Placement")]
    [Tooltip("Y offset above the world origin plane at which to draw.")]
    public float drawHeight = 0.05f;

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
      if (mode == FieldMode.None) return;
      if (!TryGetFields(out var fields)) return;

      bool needsGroup = mode is FieldMode.Potential or FieldMode.GroupVelocity
        or FieldMode.GradientDiff;
      var groupBuffers = default(GroupFieldBuffers);
      if (needsGroup && !TryGetGroupBuffers(out groupBuffers)) return;

      var gi = fields.Indexer;
      var center = (int2)math.floor(
        CCMath.WorldToGrid(transform.position, fields.Origin, fields.CellSize));
      var lo = math.max(center - windowRadius, int2.zero);
      var hi = math.min(center + windowRadius, new int2(gi.W - 1, gi.H - 1));
      if (math.any(lo > hi)) return;

      float rampMax = autoScale ? WindowMax(fields, groupBuffers, lo, hi) : fixedMax;
      if (rampMax <= 0f) rampMax = 1f;

      for (int y = lo.y; y <= hi.y; y++) {
        for (int x = lo.x; x <= hi.x; x++) {
          int i = gi.Flat(x, y);
          var world = CCMath.GridToWorld(
            CCMath.CellCenter(new int2(x, y)), fields.Origin, fields.CellSize, drawHeight);

          switch (mode) {
            case FieldMode.Density:
              DrawCell(world, fields.CellSize, Ramp(fields.Rho[i] / rampMax));
              break;
            case FieldMode.Discomfort: {
              float g = fields.Discomfort[i];
              DrawCell(world, fields.CellSize, g >= 1f ? Color.black : Ramp(g / rampMax));
              break;
            }
            case FieldMode.Walkable:
              DrawCell(world, fields.CellSize, fields.Walkable[i] != 0
                ? new Color(0.15f, 0.8f, 0.25f, 0.6f)
                : new Color(0.85f, 0.1f, 0.1f, 0.9f));
              break;
            case FieldMode.AverageVelocity: {
              // before the stamping finalize pass VAveAcc holds Σ w·m·v;
              // divide by ρ here so the tool shows v̄ either way
              float rho = fields.Rho[i];
              var v = rho > 0f ? fields.VAveAcc[i] / rho : fields.VAveAcc[i];
              DrawArrow(world, v, fields.CellSize);
              break;
            }
            case FieldMode.HeightGradient:
              DrawArrow(world, fields.DH[i], fields.CellSize);
              break;
            case FieldMode.Potential: {
              float phi = groupBuffers.Phi[i];
              DrawCell(world, fields.CellSize,
                float.IsInfinity(phi) ? Color.black : Ramp(phi / rampMax));
              break;
            }
            case FieldMode.GroupVelocity:
              DrawArrow(world, groupBuffers.Velocity0[i], fields.CellSize);
              break;
            case FieldMode.GradientDiff: {
              var cell = new int2(x, y);
              var central = CCMath.PotentialGradientCentral(groupBuffers.Phi, gi, cell);
              var upwind = CCMath.PotentialGradientUpwind(groupBuffers.Phi, gi, cell);
              float lc = math.length(central);
              float lu = math.length(upwind);
              if (lc < 1e-6f && lu < 1e-6f) break; // no flow either way
              if (lc < 1e-6f || lu < 1e-6f) {
                DrawCell(world, fields.CellSize, Color.magenta); // one-sided flow
                break;
              }
              float angle = math.acos(math.clamp(math.dot(central, upwind), -1f, 1f));
              DrawCell(world, fields.CellSize, Ramp(angle / math.PI));
              break;
            }
          }
        }
      }
    }

    private bool TryGetFields(out GlobalFields fields)
    {
      fields = default;
      var world = World.DefaultGameObjectInjectionWorld;
      if (world == null || !world.IsCreated) return false;
      var em = world.EntityManager;
      var query = em.CreateEntityQuery(ComponentType.ReadOnly<GlobalFields>());
      if (query.CalculateEntityCount() != 1) return false;
      // debug tool, editor only: stall so gizmo reads don't race in-flight jobs
      em.CompleteAllTrackedJobs();
      fields = query.GetSingleton<GlobalFields>();
      return fields.IsCreated;
    }

    private float WindowMax(in GlobalFields fields, in GroupFieldBuffers group, int2 lo, int2 hi)
    {
      var gi = fields.Indexer;
      float max = 0f;
      for (int y = lo.y; y <= hi.y; y++) {
        for (int x = lo.x; x <= hi.x; x++) {
          int i = gi.Flat(x, y);
          float v = mode switch {
            FieldMode.Density => fields.Rho[i],
            FieldMode.Discomfort => fields.Discomfort[i] < 1f ? fields.Discomfort[i] : 0f,
            FieldMode.Potential when group.IsCreated =>
              float.IsInfinity(group.Phi[i]) ? 0f : group.Phi[i],
            _ => 0f,
          };
          if (v > max && !float.IsInfinity(v)) max = v;
        }
      }
      return max;
    }

    private bool TryGetGroupBuffers(out GroupFieldBuffers buffers)
    {
      buffers = default;
      var world = World.DefaultGameObjectInjectionWorld;
      if (world == null || !world.IsCreated) return false;
      var em = world.EntityManager;
      var query = em.CreateEntityQuery(
        ComponentType.ReadOnly<CCGroup>(),
        ComponentType.ReadOnly<GroupFieldBuffers>(),
        ComponentType.ReadOnly<CCGroupInitialized>());
      using var all = query.ToComponentDataArray<GroupFieldBuffers>(Unity.Collections.Allocator.Temp);
      if (groupIndex >= all.Length) return false;
      buffers = all[groupIndex];
      return buffers.IsCreated;
    }

    /// <summary>Heat ramp: 0 → dark blue, ⅓ → cyan, ⅔ → yellow, 1 → red.</summary>
    private static Color Ramp(float t)
    {
      t = Mathf.Clamp01(t);
      if (t < 1f / 3f) return Color.Lerp(new Color(0.05f, 0.05f, 0.35f), Color.cyan, t * 3f);
      if (t < 2f / 3f) return Color.Lerp(Color.cyan, Color.yellow, (t - 1f / 3f) * 3f);
      return Color.Lerp(Color.yellow, Color.red, (t - 2f / 3f) * 3f);
    }

    private static void DrawCell(float3 center, float cellSize, Color color)
    {
      Gizmos.color = color;
      Gizmos.DrawCube(center, new Vector3(cellSize * 0.92f, 0.01f, cellSize * 0.92f));
    }

    private void DrawArrow(float3 origin, float2 v, float cellSize)
    {
      float mag = math.length(v);
      if (mag < 1e-6f) {
        Gizmos.color = new Color(1f, 1f, 1f, 0.15f);
        Gizmos.DrawSphere(origin, cellSize * 0.04f);
        return;
      }
      float len = math.min(mag * arrowScale, cellSize * 0.95f);
      var dir = v / mag;
      var tip = origin + new float3(dir.x, 0f, dir.y) * len;
      // color by heading for quick lane/vortex reading
      Gizmos.color = Color.HSVToRGB(
        (math.atan2(dir.y, dir.x) / (2f * math.PI) + 0.5f) % 1f, 0.9f, 1f);
      Gizmos.DrawLine(origin, tip);
      var left = new float3(-dir.y, 0f, dir.x);
      Gizmos.DrawLine(tip, tip - new float3(dir.x, 0f, dir.y) * len * 0.25f + left * len * 0.15f);
      Gizmos.DrawLine(tip, tip - new float3(dir.x, 0f, dir.y) * len * 0.25f - left * len * 0.15f);
    }
#endif
  }
}
