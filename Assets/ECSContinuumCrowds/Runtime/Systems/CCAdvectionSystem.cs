using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Per-frame advection (spec §13.1, Decision D11): every unit bilinearly
  /// samples its group's FRONT velocity buffer through that buffer's
  /// full-grid localIdxLookup snapshot (spec §12.2 — O(1), no hashing) and
  /// Euler-integrates (paper §4.4: RK made no appreciable difference).
  /// Because solves only ever write the BACK pair, this system runs every
  /// frame with zero dependency on in-flight solve chains (Decision D7).
  ///
  /// Also raises the domain-cache hard triggers (spec §8.4/§8.6):
  /// - escape: the unit's cell is outside the front snapshot domain
  ///   (lookup −1) → DirtyFlags[0]. (⚠ NOTE: the spec words this against
  ///   the "cached live domain"; we test against the front SNAPSHOT — the
  ///   thing actually being sampled — which is safe to read concurrently
  ///   and strictly more conservative: the live domain is always ⊇ current
  ///   needs when valid.)
  /// - stall: sampled speed ≈ 0 for > StallSeconds → DirtyFlags[1]
  ///   (scheduler refreshes with doubled pad and logs it).
  ///
  /// Arrival: cell ∈ the group's GoalSet → UnitArrived via ECB.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCVelocityDerivationSystem))]
  [BurstCompile]
  public partial struct CCAdvectionSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<CCSolveSettings>();
      state.RequireForUpdate<EndSimulationEntityCommandBufferSystem.Singleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var settings = SystemAPI.GetSingleton<CCSolveSettings>();
      var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>()
        .CreateCommandBuffer(state.WorldUnmanaged).AsParallelWriter();
      float dt = SystemAPI.Time.DeltaTime;

      foreach (var (group, domain, buffers) in
        SystemAPI.Query<RefRO<CCGroup>, RefRO<DomainCache>, RefRO<GroupFieldBuffers>>()
          .WithAll<CCGroupInitialized>()) {
        int front = group.ValueRO.ActiveBuffer;
        state.Dependency = new AdvectionJob {
          GroupId = group.ValueRO.GroupId,
          Gi = fields.Indexer,
          Origin = fields.Origin,
          CellSize = fields.CellSize,
          Velocity = front == 0 ? buffers.ValueRO.Velocity0 : buffers.ValueRO.Velocity1,
          Lookup = front == 0 ? buffers.ValueRO.LocalIdxLookup0 : buffers.ValueRO.LocalIdxLookup1,
          GoalSet = domain.ValueRO.GoalSet,
          DirtyFlags = domain.ValueRO.DirtyFlags,
          StallSeconds = settings.StallSeconds,
          Dt = dt,
          Ecb = ecb,
        }.ScheduleParallel(state.Dependency);
      }
    }

    [BurstCompile]
    [WithAll(typeof(UnitTag))]
    [WithNone(typeof(UnitArrived))]
    private partial struct AdvectionJob : IJobEntity
    {
      public int GroupId;
      public GridIndexer Gi;
      public float2 Origin;
      public float CellSize;
      [ReadOnly] public NativeArray<float2> Velocity;
      [ReadOnly] public NativeArray<int> Lookup;
      [ReadOnly] public NativeParallelHashMap<int, byte> GoalSet;
      // identical-value byte stores from many threads — benign by construction
      [NativeDisableParallelForRestriction] public NativeArray<byte> DirtyFlags;
      public float StallSeconds;
      public float Dt;
      public EntityCommandBuffer.ParallelWriter Ecb;

      private void Execute(
        Entity entity,
        [EntityIndexInQuery] int index,
        ref LocalTransform transform,
        ref UnitVelocity unitVelocity,
        in CCUnit unit)
      {
        if (unit.GroupId != GroupId) {
          return;
        }

        var pos = CCMath.WorldToGrid(transform.Position, Origin, CellSize);
        var v = CCMath.BilinearSampleVelocity(Velocity, Lookup, Gi, pos);
        unitVelocity.Value = v;

        // Euler integration; velocity is world-units/sec, position in cells
        pos += v * (Dt / CellSize);
        pos = math.clamp(pos, new float2(0.01f), new float2(Gi.W, Gi.H) - 0.01f);
        transform.Position = CCMath.GridToWorld(pos, Origin, CellSize, transform.Position.y);

        var cell = (int2)math.floor(pos);
        int flat = Gi.Flat(cell);

        if (GoalSet.IsCreated && GoalSet.ContainsKey(flat)) {
          Ecb.AddComponent<UnitArrived>(index, entity);
          return;
        }

        // escape trigger — only meaningful once a snapshot exists
        if (Lookup[flat] < 0 && DirtyFlags.IsCreated) {
          DirtyFlags[0] = 1;
        }

        // stall detector (spec §8.6): sampled speed ≈ 0 while unsolved OR
        // truly blocked; fires the doubled-pad refresh after StallSeconds
        if (math.lengthsq(v) < 1e-4f) {
          unitVelocity.StallSeconds += Dt;
          if (unitVelocity.StallSeconds > StallSeconds) {
            DirtyFlags[1] = 1;
            unitVelocity.StallSeconds = 0f; // re-arm; scheduler logs the refresh
          }
        } else {
          unitVelocity.StallSeconds = 0f;
        }
      }
    }
  }
}
