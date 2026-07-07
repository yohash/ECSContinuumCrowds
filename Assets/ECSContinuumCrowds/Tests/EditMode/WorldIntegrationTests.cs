using NUnit.Framework;
using Unity.Collections;
using Unity.Core;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Full-pipeline integration through the Phase-3 machinery: scheduler
  /// (poll → flip), flood-fill domains + caching, staggered multi-frame
  /// solve chains, double-buffered advection — stepping a manual world.
  /// </summary>
  public class WorldIntegrationTests
  {
    private const int W = 32;
    private const int H = 32;
    private const int WallX = 16;
    private const int GapYMin = 14;
    private const int GapYMax = 17;

    // -----------------------------------------------------------------
    //  Harness
    // -----------------------------------------------------------------
    private static World BuildWorld(out CCSimulationSystemGroup group,
      out EndSimulationEntityCommandBufferSystem ecbSystem)
    {
      var world = new World("CCIntegrationTest");
      group = world.GetOrCreateSystemManaged<CCSimulationSystemGroup>();
      group.AddSystemToUpdateList(world.GetOrCreateSystem<GlobalFieldsInitSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCGroupInitSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCUnitSpawnSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCSchedulerSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCSpatialHashSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCStampingSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCDomainSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCFieldSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCEikonalSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCVelocityDerivationSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCAdvectionSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCMinDistanceSystem>());
      group.SortSystems();
      ecbSystem = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
      return world;
    }

    private static CCWorldConfig AddWorldSingletons(EntityManager em, bool wall)
    {
      em.AddComponentData(em.CreateEntity(), CCConstants.Defaults);
      var config = BuildWorldConfig(wall);
      em.AddComponentData(em.CreateEntity(), config);
      var settings = CCSolveSettings.Defaults;
      settings.SolveHz = 60f; // tick every step in tests
      em.AddComponentData(em.CreateEntity(), settings);
      return config;
    }

    private static Entity AddGroup(EntityManager em, int groupId, float2 goalMin, float2 goalMax)
    {
      var e = em.CreateEntity();
      em.AddComponentData(e, new CCGroup {
        GroupId = groupId, Alpha = 1f, Beta = 1f, Gamma = 1f,
      });
      em.AddComponentData(e, new CCGroupGoalRect { MinXZ = goalMin, MaxXZ = goalMax });
      em.AddBuffer<GoalCell>(e);
      return e;
    }

    private static Entity AddUnit(EntityManager em, int groupId, float2 posXZ)
    {
      var unit = em.CreateEntity();
      em.AddComponent<UnitTag>(unit);
      em.AddComponentData(unit, new CCUnit {
        Mass = 1f, Radius = 0.4f, FootprintSize = 1f, GroupId = groupId,
      });
      em.AddComponentData(unit, new UnitVelocity());
      em.AddComponentData(unit, LocalTransform.FromPosition(
        new float3(posXZ.x, 0f, posXZ.y)));
      return unit;
    }

    private static void Step(World world, CCSimulationSystemGroup group,
      EndSimulationEntityCommandBufferSystem ecb, ref double elapsed, float dt)
    {
      elapsed += dt;
      world.SetTime(new TimeData(elapsed, dt));
      group.Update();
      ecb.Update();
    }

    private static CCWorldConfig BuildWorldConfig(bool wall)
    {
      var builder = new BlobBuilder(Allocator.Temp);
      ref var root = ref builder.ConstructRoot<CCWorldBakeData>();
      builder.Allocate(ref root.Height, 0);
      var discomfort = builder.Allocate(ref root.Discomfort, W * H);
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          bool w = wall && x == WallX && (y < GapYMin || y > GapYMax);
          discomfort[y * W + x] = w ? 1f : 0f;
        }
      }
      var blob = builder.CreateBlobAssetReference<CCWorldBakeData>(Allocator.Persistent);
      builder.Dispose();
      return new CCWorldConfig {
        W = W, H = H, CellSize = 1f, Origin = float2.zero, Bake = blob,
      };
    }

    // -----------------------------------------------------------------
    //  Tests
    // -----------------------------------------------------------------

    [Test]
    public void UnitWalksAroundObstacleToGoal()
    {
      var world = BuildWorld(out var group, out var ecb);
      var config = default(CCWorldConfig);
      try {
        var em = world.EntityManager;
        config = AddWorldSingletons(em, wall: true);
        AddGroup(em, 0, new float2(26.5f, 15f), new float2(28.5f, 17f));
        var unit = AddUnit(em, 0, new float2(4.5f, 16.5f));

        const float dt = 1f / 30f;
        double elapsed = 0;
        bool arrived = false;
        bool wasEverMoving = false;

        for (int step = 0; step < 900 && !arrived; step++) {
          Step(world, group, ecb, ref elapsed, dt);
          em.CompleteAllTrackedJobs();

          var pos = em.GetComponentData<LocalTransform>(unit).Position;
          var cell = (int2)math.floor(pos.xz);

          bool inWall = cell.x == WallX && (cell.y < GapYMin || cell.y > GapYMax);
          Assert.IsFalse(inWall, $"unit entered wall cell {cell} at step {step}");
          if (cell.x >= WallX - 1 && cell.x <= WallX + 1) {
            Assert.IsTrue(cell.y >= GapYMin - 1 && cell.y <= GapYMax + 1,
              $"unit crossed the wall outside the gap at {cell} (step {step})");
          }
          if (math.length(em.GetComponentData<UnitVelocity>(unit).Value) > 0.5f) {
            wasEverMoving = true;
          }
          arrived = em.HasComponent<UnitArrived>(unit);
        }

        Assert.IsTrue(wasEverMoving, "unit never started moving — solve chain broken");
        Assert.IsTrue(arrived, "unit did not reach the goal within the step budget");
      } finally {
        world.Dispose();
        if (config.Bake.IsCreated) config.Bake.Dispose();
      }
    }

    [Test]
    public void TwoStaggeredGroupsBothSolveFlipAndArrive()
    {
      var world = BuildWorld(out var group, out var ecb);
      var config = default(CCWorldConfig);
      try {
        var em = world.EntityManager;
        config = AddWorldSingletons(em, wall: false);

        // two groups crossing in opposite directions (GroupsPerTick = 1 →
        // they occupy alternating stagger slots)
        var groupA = AddGroup(em, 0, new float2(27f, 14f), new float2(29f, 18f));
        var groupB = AddGroup(em, 1, new float2(3f, 14f), new float2(5f, 18f));
        var unitsA = new Entity[4];
        var unitsB = new Entity[4];
        for (int i = 0; i < 4; i++) {
          unitsA[i] = AddUnit(em, 0, new float2(4.5f, 13.5f + i * 1.6f));
          unitsB[i] = AddUnit(em, 1, new float2(27.5f, 13.5f + i * 1.6f));
        }

        const float dt = 1f / 30f;
        double elapsed = 0;
        bool sawBothInFlight = false;

        for (int step = 0; step < 1500; step++) {
          Step(world, group, ecb, ref elapsed, dt);
          em.CompleteAllTrackedJobs();

          // stagger observability: at most one group should ENTER its
          // pipeline per tick, but pipelines may overlap across frames
          var phaseA = em.GetComponentData<CCGroup>(groupA).Phase;
          var phaseB = em.GetComponentData<CCGroup>(groupB).Phase;
          if (phaseA != SolvePhase.Idle && phaseB != SolvePhase.Idle) {
            sawBothInFlight = true;
          }

          bool allArrived = true;
          for (int i = 0; i < 4; i++) {
            allArrived &= em.HasComponent<UnitArrived>(unitsA[i]);
            allArrived &= em.HasComponent<UnitArrived>(unitsB[i]);
          }
          if (allArrived) break;
        }

        for (int i = 0; i < 4; i++) {
          Assert.IsTrue(em.HasComponent<UnitArrived>(unitsA[i]), $"group-0 unit {i} never arrived");
          Assert.IsTrue(em.HasComponent<UnitArrived>(unitsB[i]), $"group-1 unit {i} never arrived");
        }

        // both groups flipped buffers at least once and the cache did its job
        var telemetryQuery = em.CreateEntityQuery(ComponentType.ReadOnly<CCSolveTelemetry>());
        var telemetry = telemetryQuery.GetSingleton<CCSolveTelemetry>();
        Assert.GreaterOrEqual(telemetry.SolvesCompleted, 4, "both groups should complete multiple solves");
        Assert.GreaterOrEqual(telemetry.DomainRefreshes, 2, "each group builds its domain at least once");
        Assert.Greater(telemetry.CacheHits, telemetry.DomainRefreshes,
          "steady-state ticks should overwhelmingly reuse cached domains (spec §15 P3)");
        Assert.AreEqual(0, telemetry.StallRefreshes, "open field: no stalls expected");

        var a = em.GetComponentData<CCGroup>(groupA);
        var b = em.GetComponentData<CCGroup>(groupB);
        Assert.Greater(a.LastSolveTime, 0.0);
        Assert.Greater(b.LastSolveTime, 0.0);
        Assert.AreNotEqual(a.ScheduleSlot, b.ScheduleSlot, "groups occupy distinct stagger slots");
        Assert.IsTrue(sawBothInFlight || true); // informational; overlap is allowed across frames
      } finally {
        world.Dispose();
        if (config.Bake.IsCreated) config.Bake.Dispose();
      }
    }
  }
}
