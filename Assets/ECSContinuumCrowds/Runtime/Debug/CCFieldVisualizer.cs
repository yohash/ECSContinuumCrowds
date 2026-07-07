using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Scene-view debug visualizer for the Continuum Crowds fields
  /// (spec §15 Phase 0 — "invest here"). Drop on any GameObject; it draws a
  /// window of cells centered on this transform.
  ///
  /// Global modes read the stamped map; per-group modes (Potential,
  /// GroupVelocity, GradientDiff, Domain) read the group's compact domain
  /// buffers. Because solve chains are manually chained (Decision D7) and
  /// not tracked by the ECS safety system, this tool first Complete()s the
  /// selected group's chain — a deliberate, editor-only stall.
  ///
  /// GradientDiff is the shockline diagnostic (Decision D10): the angular
  /// difference between the central-repo and upwind-paper gradients on the
  /// SAME live φ — kinks behind obstacles / along medial axes light up.
  /// Domain paints cache membership — watch refreshes/hysteresis live.
  /// </summary>
  [ExecuteAlways]
  public class CCFieldVisualizer : MonoBehaviour
  {
    public enum FieldMode
    {
      None,
      Density,          // ρ
      AverageVelocity,  // v̄ — arrows
      Discomfort,       // g
      Walkable,         // g < 1 mask
      HeightGradient,   // ∇h — arrows
      Potential,        // φ (per group) — heat ramp, ∞/absent = black
      GroupVelocity,    // group FRONT velocity snapshot — heading-colored arrows
      GradientDiff,     // central-repo vs upwind-paper ∇φ angle — shockline debug
      Domain,           // domain-cache membership (per group)
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
        or FieldMode.GradientDiff or FieldMode.Domain;
      var group = default(GroupSnapshot);
      if (needsGroup && !TryGetGroup(out group)) return;

      var gi = fields.Indexer;
      var center = (int2)math.floor(
        CCMath.WorldToGrid(transform.position, fields.Origin, fields.CellSize));
      var lo = math.max(center - windowRadius, int2.zero);
      var hi = math.min(center + windowRadius, new int2(gi.W - 1, gi.H - 1));
      if (math.any(lo > hi)) return;

      float rampMax = autoScale ? WindowMax(fields, group, lo, hi) : fixedMax;
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
            case FieldMode.AverageVelocity:
              DrawArrow(world, fields.VAveAcc[i], fields.CellSize);
              break;
            case FieldMode.HeightGradient:
              DrawArrow(world, fields.DH[i], fields.CellSize);
              break;
            case FieldMode.Potential: {
              if (!TryLocal(group, i, out int local)) {
                DrawCell(world, fields.CellSize, Color.black);
                break;
              }
              float phi = group.Buffers.Phi[local];
              DrawCell(world, fields.CellSize,
                float.IsInfinity(phi) ? Color.black : Ramp(phi / rampMax));
              break;
            }
            case FieldMode.GroupVelocity: {
              int local = group.FrontLookup[i];
              if (local >= 0) {
                DrawArrow(world, group.FrontVelocity[local], fields.CellSize);
              }
              break;
            }
            case FieldMode.GradientDiff: {
              if (!TryLocal(group, i, out int local)) break;
              var neighbors = group.Domain.NeighborLocalIdx[local];
              var central = CCMath.PotentialGradientCentral(
                group.Buffers.Phi, local, new int2(x, y), neighbors, gi);
              var upwind = CCMath.PotentialGradientUpwind(group.Buffers.Phi, local, neighbors);
              float lc = math.length(central);
              float lu = math.length(upwind);
              if (lc < 1e-6f && lu < 1e-6f) break;
              if (lc < 1e-6f || lu < 1e-6f) {
                DrawCell(world, fields.CellSize, Color.magenta); // one-sided flow
                break;
              }
              float angle = math.acos(math.clamp(math.dot(central, upwind), -1f, 1f));
              DrawCell(world, fields.CellSize, Ramp(angle / math.PI));
              break;
            }
            case FieldMode.Domain: {
              bool inDomain = group.Domain.GlobalToLocal.ContainsKey(i);
              bool inFront = group.FrontLookup[i] >= 0;
              // teal = live domain, blue = also in the sampled front snapshot
              if (inDomain || inFront) {
                DrawCell(world, fields.CellSize, inDomain && inFront
                  ? new Color(0.2f, 0.45f, 1f, 0.7f)
                  : inDomain
                    ? new Color(0.1f, 0.8f, 0.75f, 0.6f)
                    : new Color(0.8f, 0.55f, 0.1f, 0.6f)); // front-only (stale snapshot)
              }
              break;
            }
          }
        }
      }
    }

    private struct GroupSnapshot
    {
      public DomainCache Domain;
      public GroupFieldBuffers Buffers;
      public NativeArray<float2> FrontVelocity;
      public NativeArray<int> FrontLookup;
    }

    private static bool TryLocal(in GroupSnapshot g, int flat, out int local)
    {
      if (g.Domain.GlobalToLocal.TryGetValue(flat, out local)
        && local < g.Buffers.DomainLength) {
        return true;
      }
      local = -1;
      return false;
    }

    private bool TryGetFields(out GlobalFields fields)
    {
      fields = default;
      var world = World.DefaultGameObjectInjectionWorld;
      if (world == null || !world.IsCreated) return false;
      var em = world.EntityManager;
      var query = em.CreateEntityQuery(ComponentType.ReadOnly<GlobalFields>());
      if (query.CalculateEntityCount() != 1) return false;
      // debug tool, editor only: stall so gizmo reads don't race tracked jobs
      em.CompleteAllTrackedJobs();
      fields = query.GetSingleton<GlobalFields>();
      return fields.IsCreated;
    }

    private bool TryGetGroup(out GroupSnapshot snapshot)
    {
      snapshot = default;
      var world = World.DefaultGameObjectInjectionWorld;
      if (world == null || !world.IsCreated) return false;
      var em = world.EntityManager;
      var query = em.CreateEntityQuery(
        ComponentType.ReadOnly<CCGroup>(),
        ComponentType.ReadOnly<DomainCache>(),
        ComponentType.ReadOnly<GroupFieldBuffers>(),
        ComponentType.ReadOnly<CCGroupSolveState>(),
        ComponentType.ReadOnly<CCGroupInitialized>());
      using var entities = query.ToEntityArray(Allocator.Temp);
      if (groupIndex >= entities.Length) return false;
      var e = entities[groupIndex];

      // manual solve chains aren't tracked by the safety system — complete
      // this group's chain before touching its containers (editor-only stall)
      em.GetComponentData<CCGroupSolveState>(e).ChainTail.Complete();

      var group = em.GetComponentData<CCGroup>(e);
      var buffers = em.GetComponentData<GroupFieldBuffers>(e);
      snapshot = new GroupSnapshot {
        Domain = em.GetComponentData<DomainCache>(e),
        Buffers = buffers,
        FrontVelocity = group.ActiveBuffer == 0 ? buffers.Velocity0 : buffers.Velocity1,
        FrontLookup = group.ActiveBuffer == 0 ? buffers.LocalIdxLookup0 : buffers.LocalIdxLookup1,
      };
      return snapshot.Domain.IsCreated && buffers.IsCreated;
    }

    private float WindowMax(in GlobalFields fields, in GroupSnapshot group, int2 lo, int2 hi)
    {
      var gi = fields.Indexer;
      float max = 0f;
      for (int y = lo.y; y <= hi.y; y++) {
        for (int x = lo.x; x <= hi.x; x++) {
          int i = gi.Flat(x, y);
          float v = 0f;
          switch (mode) {
            case FieldMode.Density:
              v = fields.Rho[i];
              break;
            case FieldMode.Discomfort:
              v = fields.Discomfort[i] < 1f ? fields.Discomfort[i] : 0f;
              break;
            case FieldMode.Potential:
              if (TryLocal(group, i, out int local)) {
                float phi = group.Buffers.Phi[local];
                v = float.IsInfinity(phi) ? 0f : phi;
              }
              break;
          }
          if (v > max && !float.IsInfinity(v)) max = v;
        }
      }
      return max;
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
