using System.Collections.Generic;
using UnityEngine;
using Yohash.ContinuumCrowds;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Single-tile IContinuumTile implementation for driving the reference
  /// oracle: one tile at corner (0,0) covering the whole test grid, with
  /// directly settable field arrays. The tile hash function maps every
  /// location to this tile, so global reads resolve locally and
  /// out-of-bounds points come back invalid — mirroring our single dense
  /// global grid.
  /// </summary>
  public class OracleTile : IContinuumTile
  {
    private float[,] _rho;
    private Vector2[,] _vAve;
    private Vector2[,] _dh;
    private float[,] _g;
    private Vector4[,] _f;
    private Vector4[,] _c;

    public OracleTile(int w, int h)
    {
      SizeX = w;
      SizeY = h;
      _rho = new float[w, h];
      _vAve = new Vector2[w, h];
      _dh = new Vector2[w, h];
      _g = new float[w, h];
      _f = new Vector4[w, h];
      _c = new Vector4[w, h];
    }

    public int SizeX { get; }
    public int SizeY { get; }
    public Location Corner => new Location(0, 0);

    public IEnumerable<int> ImpactingUnitsIds() => System.Array.Empty<int>();
    public void StoreBaselineFields() { }
    public void ResetToBaseline() { }

    public ref float[,] rho => ref _rho;
    public ref Vector2[,] vAve => ref _vAve;
    public ref Vector2[,] dh => ref _dh;
    public ref float[,] g => ref _g;
    public ref Vector4[,] f => ref _f;
    public ref Vector4[,] C => ref _c;

    /// <summary>Tile dictionary + hash function for DynamicGlobalFields calls.</summary>
    public Dictionary<Location, IContinuumTile> AsTiles()
      => new Dictionary<Location, IContinuumTile> { { Corner, this } };

    public Location Hash(Location _) => Corner;
  }
}
