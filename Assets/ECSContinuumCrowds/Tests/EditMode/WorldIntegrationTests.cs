using NUnit.Framework;
using Unity.Collections;
using Unity.Core;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Yohash.ECSContinuumCrowds.Tests
{
  /// <summary>
  /// Phase-1 validation (b): a single unit walks an optimal-ish path around
  /// an obstacle to the goal, driven by the full ECS pipeline (stamping →
  /// fields → FMM → velocity → advection → min-distance) stepping a manual
  /// world — no play mode required.
  /// </summary>
  public class WorldIntegrationTests
  {
    private const int W = 32;
    private const int H = 32;
    private const int WallX = 16;
    private const int GapYMin = 14;
    private const int GapYMax = 17;

    [Test]
    public void UnitWalksAroundObstacleToGoal()
    {
      var world = new World("CCIntegrationTest");
      var config = default(CCWorldConfig);
      try {
        config = RunScenario(world);
      } finally {
        world.Dispose();
        if (config.Bake.IsCreated) {
          config.Bake.Dispose(); // manually built blob (bakers own this in real worlds)
        }
      }
    }

    private static CCWorldConfig RunScenario(World world)
    {
      var em = world.EntityManager;

      var group = world.GetOrCreateSystemManaged<CCSimulationSystemGroup>();
      group.AddSystemToUpdateList(world.GetOrCreateSystem<GlobalFieldsInitSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCSolveTickSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCGroupInitSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCUnitSpawnSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCSpatialHashSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCStampingSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCFieldSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCEikonalSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCVelocityDerivationSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCAdvectionSystem>());
      group.AddSystemToUpdateList(world.GetOrCreateSystem<CCMinDistanceSystem>());
      group.SortSystems();
      var ecbSystem = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();

      // world config: 32×32, 1 m cells, origin (0,0), wall column with a gap
      em.AddComponentData(em.CreateEntity(), CCConstants.Defaults);
      var config = BuildWorldConfig();
      em.AddComponentData(em.CreateEntity(), config);
      em.AddComponentData(em.CreateEntity(), new CCSolveSettings {
        SolveHz = 60f, // solve every step in this test
        Scheme = GradientScheme.CentralRepo,
        MaxUnitRadius = 0.5f,
      });

      // group with goal on the far side of the wall
      var groupEntity = em.CreateEntity();
      em.AddComponentData(groupEntity, new CCGroup {
        GroupId = 0, Alpha = 1f, Beta = 1f, Gamma = 1f,
      });
      em.AddComponentData(groupEntity, new CCGroupGoalRect {
        MinXZ = new float2(26.5f, 15f),
        MaxXZ = new float2(28.5f, 17f),
      });
      em.AddBuffer<GoalCell>(groupEntity);

      // one unit on the near side
      var unit = em.CreateEntity();
      em.AddComponent<UnitTag>(unit);
      em.AddComponentData(unit, new CCUnit {
        Mass = 1f, Radius = 0.4f, FootprintSize = 1f, GroupId = 0,
      });
      em.AddComponentData(unit, new UnitVelocity { Value = float2.zero });
      em.AddComponentData(unit, LocalTransform.FromPosition(new float3(4.5f, 0f, 16.5f)));

      const float dt = 1f / 30f;
      double elapsed = 0;
      bool arrived = false;
      bool wasEverMoving = false;

      for (int step = 0; step < 900 && !arrived; step++) {
        elapsed += dt;
        world.SetTime(new TimeData(elapsed, dt));
        group.Update();
        ecbSystem.Update();
        em.CompleteAllTrackedJobs();

        var pos = em.GetComponentData<LocalTransform>(unit).Position;
        var cell = (int2)math.floor(pos.xz);

        // never inside the wall
        bool inWall = cell.x == WallX && (cell.y < GapYMin || cell.y > GapYMax);
        Assert.IsFalse(inWall, $"unit entered wall cell {cell} at step {step}");

        // while crossing the wall line, must be within the gap (±1 for the
        // fringe of the bilinear sample)
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
      return config;
    }

    private static CCWorldConfig BuildWorldConfig()
    {
      var builder = new BlobBuilder(Allocator.Temp);
      ref var root = ref builder.ConstructRoot<CCWorldBakeData>();
      builder.Allocate(ref root.Height, 0);
      var discomfort = builder.Allocate(ref root.Discomfort, W * H);
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          bool wall = x == WallX && (y < GapYMin || y > GapYMax);
          discomfort[y * W + x] = wall ? 1f : 0f;
        }
      }
      var blob = builder.CreateBlobAssetReference<CCWorldBakeData>(Allocator.Persistent);
      builder.Dispose();
      return new CCWorldConfig {
        W = W,
        H = H,
        CellSize = 1f,
        Origin = float2.zero,
        Bake = blob,
      };
    }
  }
}
