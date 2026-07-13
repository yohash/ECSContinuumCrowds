using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// Per-frame advection (spec §13.1, Decision D11): every unit bilinearly
  /// samples its group's velocity field at its grid position and
  /// Euler-integrates. Euler is deliberate — paper §4.4: higher-order
  /// integration (RK) made no appreciable difference.
  ///
  /// Velocity conventions: field values are world-units/second in the XZ
  /// plane; grid displacement divides by CellSize. UnitVelocity stores
  /// world-units/second (what stamping's momentum term needs).
  ///
  /// Arrival: when a unit's cell carries the solver's Goal flag, tag it
  /// UnitArrived via ECB — it stops advecting; consumers react to the tag.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup))]
  [UpdateAfter(typeof(CCVelocityDerivationSystem))]
  [BurstCompile]
  public partial struct CCAdvectionSystem : ISystem
  {
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<GlobalFields>();
      state.RequireForUpdate<EndSimulationEntityCommandBufferSystem.Singleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var fields = SystemAPI.GetSingleton<GlobalFields>();
      var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>()
        .CreateCommandBuffer(state.WorldUnmanaged).AsParallelWriter();
      float dt = SystemAPI.Time.DeltaTime;

      // one pass per group (Phase 1: typically one). Each pass touches only
      // its group's units via the GroupId branch — a predictable branch,
      // cheap under Burst (spec §3.2). The Phase-3 scheduler replaces this
      // with per-group buffer tables indexed by GroupId.
      foreach (var (group, buffers) in
        SystemAPI.Query<RefRO<CCGroup>, RefRO<GroupFieldBuffers>>()
          .WithAll<CCGroupInitialized>()) {
        state.Dependency = new AdvectionJob {
          GroupId = group.ValueRO.GroupId,
          Gi = fields.Indexer,
          Origin = fields.Origin,
          CellSize = fields.CellSize,
          Velocity = buffers.ValueRO.Velocity0,
          CellState = buffers.ValueRO.CellState,
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
      [ReadOnly] public NativeArray<byte> CellState;
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
        var v = CCMath.BilinearSampleVelocity(Velocity, Gi, pos);
        unitVelocity.Value = v;

        // Euler integration; velocity is world-units/sec, position in cells
        pos += v * (Dt / CellSize);
        pos = math.clamp(pos, new float2(0.01f), new float2(Gi.W, Gi.H) - 0.01f);
        transform.Position = CCMath.GridToWorld(pos, Origin, CellSize, transform.Position.y);

        var cell = (int2)math.floor(pos);
        if ((CellState[Gi.Flat(cell)] & CCCellState.Goal) != 0) {
          Ecb.AddComponent<UnitArrived>(index, entity);
        }
      }
    }
  }
}
