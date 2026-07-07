using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Snapshot-lookup bilinear sampling (spec §13.1 + §12.2): compact
  /// velocity buffers interpreted through a full-grid localIdxLookup, with
  /// out-of-grid AND out-of-domain corners renormalized away.
  /// </summary>
  public class BilinearSampleTests
  {
    private static (NativeArray<float2> vel, NativeArray<int> lookup) IdentityField(
      GridIndexer gi, float2 v)
    {
      var vel = new NativeArray<float2>(gi.CellCount, Allocator.Temp);
      var lookup = new NativeArray<int>(gi.CellCount, Allocator.Temp);
      for (int i = 0; i < vel.Length; i++) {
        vel[i] = v;
        lookup[i] = i;
      }
      return (vel, lookup);
    }

    [Test]
    public void ExactCellCenterReturnsCellValue()
    {
      var gi = new GridIndexer(4, 4);
      var (vel, lookup) = IdentityField(gi, float2.zero);
      using (vel)
      using (lookup) {
        vel[gi.Flat(2, 1)] = new float2(3f, -1f);
        var sampled = CCMath.BilinearSampleVelocity(vel, lookup, gi, new float2(2.5f, 1.5f));
        Assert.AreEqual(3f, sampled.x, 1e-5f);
        Assert.AreEqual(-1f, sampled.y, 1e-5f);
      }
    }

    [Test]
    public void MidpointBlendsEqually()
    {
      var gi = new GridIndexer(4, 4);
      var (vel, lookup) = IdentityField(gi, float2.zero);
      using (vel)
      using (lookup) {
        vel[gi.Flat(1, 1)] = new float2(2f, 0f);
        vel[gi.Flat(2, 1)] = new float2(4f, 0f);
        var sampled = CCMath.BilinearSampleVelocity(vel, lookup, gi, new float2(2.0f, 1.5f));
        Assert.AreEqual(3f, sampled.x, 1e-5f);
      }
    }

    [Test]
    public void GridEdgeRenormalizesInsteadOfFadingToZero()
    {
      var gi = new GridIndexer(4, 4);
      var (vel, lookup) = IdentityField(gi, new float2(5f, 0f));
      using (vel)
      using (lookup) {
        // at the corner, 3 of 4 bilinear corners are out of bounds; the
        // remaining weight renormalizes to the full value (spec §13.1)
        var sampled = CCMath.BilinearSampleVelocity(vel, lookup, gi, new float2(0.1f, 0.1f));
        Assert.AreEqual(5f, sampled.x, 1e-4f);
      }
    }

    [Test]
    public void DomainFringeRenormalizesOverMissingCorners()
    {
      // out-of-DOMAIN corners (lookup −1) behave exactly like out-of-grid:
      // weight 0 + renormalize, so velocity fades gracefully at the padded
      // fringe rather than snapping toward zero (spec §12.2/§13.1)
      var gi = new GridIndexer(4, 4);
      var (vel, lookup) = IdentityField(gi, new float2(7f, 0f));
      using (vel)
      using (lookup) {
        lookup[gi.Flat(3, 2)] = -1; // hole in the snapshot domain
        lookup[gi.Flat(3, 3)] = -1;
        // sample between columns 2 and 3 at y=2.5: right corners missing
        var sampled = CCMath.BilinearSampleVelocity(vel, lookup, gi, new float2(3.0f, 3.0f));
        Assert.AreEqual(7f, sampled.x, 1e-4f, "remaining corners must renormalize to full value");
      }
    }

    [Test]
    public void FullyOutsideSnapshotReturnsZero()
    {
      var gi = new GridIndexer(4, 4);
      var (vel, lookup) = IdentityField(gi, new float2(5f, 5f));
      using (vel)
      using (lookup) {
        for (int i = 0; i < lookup.Length; i++) lookup[i] = -1;
        var sampled = CCMath.BilinearSampleVelocity(vel, lookup, gi, new float2(2f, 2f));
        Assert.AreEqual(0f, math.length(sampled), 1e-6f);
      }
    }
  }
}
