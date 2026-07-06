// ---------------------------------------------------------------------------
// REFERENCE ORACLE — verbatim copy from yohash/ContinuumCrowds @ 127cef3
// (Interfaces/IContinuumUnit.cs). MIT License, (c) Jason Heebl.
// Imported unmodified as the managed parity oracle (spec §15 Phase 1).
// DO NOT EDIT — parity tests compare the Burst implementation against this.
// ---------------------------------------------------------------------------
﻿using UnityEngine;

namespace Yohash.ContinuumCrowds
{
  public interface IContinuumUnit
  {
    /// <summary>
    /// The current velocity of this unit
    /// </summary>
    Vector2 Velocity { get; }
    /// <summary>
    /// The mass of this unit
    /// </summary>
    float Mass { get; }
    /// <summary>
    /// The footprint of this unit, from which the
    /// density is calculated. Keeping in mind:
    /// regarding the density computations:
    ///
    ///   > ...each person should contribute no less
    ///   > than rho_bar to their own grid cell, but
    ///   > no more than rho_bar to any neighboring
    ///   > grid cell.
    ///
    /// The computation of this footprint must comply
    /// with these conditions
    /// </summary>
    float[,] Footprint { get; }
    /// <summary>
    /// The corner of this unit's footprint
    /// </summary>
    Vector2Int Corner { get; }
  }
}
