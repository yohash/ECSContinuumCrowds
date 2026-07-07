using Unity.Entities;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>Marks an entity as a Continuum Crowds unit (spec §3.2).</summary>
  public struct UnitTag : IComponentData
  {
  }

  /// <summary>
  /// Unit simulation parameters (spec §3.2).
  ///
  /// GroupId is a plain int, NOT an ISharedComponentData — deliberate:
  /// shared-component grouping would fragment chunks and force a structural
  /// change on every regroup. Regrouping a unit = write GroupId; advection
  /// branches per-entity on it (predictable branch, cheap under Burst).
  /// </summary>
  public struct CCUnit : IComponentData
  {
    /// <summary>Density scale (default 1). ⚠ DIVERGENCE (repo, kept): the
    /// paper has no mass term; the repo scales density by unit mass, which
    /// enables heterogeneous units (vehicles, large creatures) for free.</summary>
    public float Mass;
    /// <summary>Physical radius (world units) for the min-distance pass.</summary>
    public float Radius;
    /// <summary>Base footprint half-extent in cells (spec §6.4); default
    /// units use the pure 2×2 splat.</summary>
    public float FootprintSize;
    /// <summary>Which group's velocity field this unit samples.</summary>
    public int GroupId;
  }

  /// <summary>
  /// Current unit velocity (world units/second, XZ) — written by advection,
  /// read by stamping. StallSeconds accumulates while the sampled speed is
  /// ≈ 0 (and the unit hasn't arrived); past CCSolveSettings.StallSeconds it
  /// raises the group's stall trigger → domain refresh with doubled pad
  /// (spec §8.6).
  /// </summary>
  public struct UnitVelocity : IComponentData
  {
    public float2 Value;
    public float StallSeconds;
  }

  /// <summary>
  /// Arrival event tag (spec §13.1): added via ECB when a unit's cell enters
  /// its group's goal set. Arrived units stop advecting; consumers may react
  /// and remove/repurpose the entity.
  /// </summary>
  public struct UnitArrived : IComponentData
  {
  }
}
