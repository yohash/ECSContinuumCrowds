using Unity.Mathematics;
using UnityEngine;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// ScriptableObject authoring wrapper for <see cref="CCConstants"/>
  /// (spec §2.2/§5.3). Field names and defaults mirror the reference repo's
  /// Constants.cs verbatim; λ is the spec-added splat exponent (§6.4).
  /// ρ̄ is derived (1/2^λ), never authored.
  ///
  /// Editing this asset during Play Mode hot-reloads the singleton each frame
  /// in the Editor (CCConstantsHotReloadSystem).
  /// </summary>
  [CreateAssetMenu(menuName = "ECS Continuum Crowds/Constants", fileName = "CCConstants")]
  public class CCConstantsAsset : ScriptableObject
  {
    [Header("How far a unit's footprint will extend beyond its size")]
    public float u_unitRadialFalloff = 0f;

    [Header("Density splat exponent λ (ρ̄ = 1/2^λ; requires f_rhoMin ≥ ρ̄)")]
    public float lambda = 2f;

    [Header("Speed value over which a dynamic footprint is computed vs. static")]
    public float v_dynamicFootprintThreshold = 0.25f;

    [Header("Number of seconds to extrapolate unit's velocity")]
    public float v_predictiveSeconds = 1f;

    [Header("Max and min scalars to weight unit's extrapolated velocity")]
    public float v_scaleMax = 0.3f;
    public float v_scaleMin = 0.25f;

    [Header("Cap (cells) on predictive extrapolation distance (bounds hash bucket size)")]
    public float v_predictiveDistanceCapCells = 8f;

    [Header("Max and Min slopes to scale topographical speed")]
    public float f_slopeMax = 1f;
    public float f_slopeMin = -1f;

    [Header("Max and min densities to determine flow speed, or topographical speed")]
    public float f_rhoMax = 0.8f;
    public float f_rhoMin = 0.3f;

    [Header("Max and min speed field")]
    public float f_speedMin = 0f;
    public float f_speedMax = 20f;

    [Header("Weights: Path Length / Time (speed inverse) / Discomfort")]
    public float C_alpha = 1f;
    public float C_beta = 1f;
    public float C_gamma = 1f;

    [Header("Weighted average for Eikonal solutions")]
    public float maxWeight = 2.5f;
    public float minWeight = 1f;

    public CCConstants ToComponent()
    {
      var c = new CCConstants {
        u_unitRadialFalloff = u_unitRadialFalloff,
        lambda = lambda,
        v_dynamicFootprintThreshold = v_dynamicFootprintThreshold,
        v_predictiveSeconds = v_predictiveSeconds,
        v_scaleMax = v_scaleMax,
        v_scaleMin = v_scaleMin,
        v_predictiveDistanceCapCells = v_predictiveDistanceCapCells,
        f_slopeMax = f_slopeMax,
        f_slopeMin = f_slopeMin,
        f_rhoMax = f_rhoMax,
        f_rhoMin = f_rhoMin,
        f_speedMin = f_speedMin,
        f_speedMax = f_speedMax,
        C_alpha = C_alpha,
        C_beta = C_beta,
        C_gamma = C_gamma,
        maxWeight = maxWeight,
        minWeight = minWeight,
      };
      c.DeriveRhoBar();
      return c;
    }

    private void OnValidate()
    {
      var rhoBar = math.pow(0.5f, lambda);
      if (f_rhoMin < rhoBar) {
        Debug.LogWarning(
          $"[ECSContinuumCrowds] Config invariant violated: f_rhoMin ({f_rhoMin}) < ρ̄ ({rhoBar}). " +
          "An isolated unit could congest itself (spec §2.3). Raise f_rhoMin or raise λ.",
          this);
      }
    }
  }
}
