using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Yohash.ECSContinuumCrowds
{
  /// <summary>
  /// One-shot world initialization (spec §5): allocates the
  /// <see cref="GlobalFields"/> singleton (Persistent), copies authored
  /// discomfort, bakes the height gradient ∇h (central differences, slope
  /// components clamped into the [f_slopeMin, f_slopeMax] domain assumption),
  /// and derives the walkability mask. Runs once when a baked
  /// <see cref="CCWorldConfig"/> and <see cref="CCConstants"/> exist, then
  /// disables itself. Owns disposal of the arrays on system destroy.
  ///
  /// Init-time only: the blocking Complete() here is deliberate and never
  /// occurs on a per-frame path.
  /// </summary>
  [UpdateInGroup(typeof(CCSimulationSystemGroup), OrderFirst = true)]
  public partial struct GlobalFieldsInitSystem : ISystem
  {
    private EntityQuery _fieldsQuery;

    public void OnCreate(ref SystemState state)
    {
      _fieldsQuery = state.GetEntityQuery(ComponentType.ReadWrite<GlobalFields>());
      state.RequireForUpdate<CCWorldConfig>();
      state.RequireForUpdate<CCConstants>();
    }

    public void OnUpdate(ref SystemState state)
    {
      if (!_fieldsQuery.IsEmptyIgnoreFilter) {
        state.Enabled = false;
        return;
      }

      var config = SystemAPI.GetSingleton<CCWorldConfig>();
      var constants = SystemAPI.GetSingleton<CCConstants>();
      int cells = config.W * config.H;

      var fields = new GlobalFields {
        W = config.W,
        H = config.H,
        CellSize = config.CellSize,
        Origin = config.Origin,
        Rho = new NativeArray<float>(cells, Allocator.Persistent),
        VAveAcc = new NativeArray<float2>(cells, Allocator.Persistent),
        Discomfort = new NativeArray<float>(cells, Allocator.Persistent),
        DH = new NativeArray<float2>(cells, Allocator.Persistent),
        Walkable = new NativeArray<byte>(cells, Allocator.Persistent),
      };

      bool hasBake = config.Bake.IsCreated;
      bool hasHeight = hasBake && config.Bake.Value.Height.Length == cells;
      bool hasDiscomfort = hasBake && config.Bake.Value.Discomfort.Length == cells;

      var handle = default(JobHandle);

      if (hasDiscomfort) {
        handle = new CopyDiscomfortJob {
          Bake = config.Bake,
          Discomfort = fields.Discomfort,
        }.Schedule(cells, 4096, handle);
      }

      if (hasHeight) {
        // stage blob heights into a flat array so the gradient math is a
        // plain-array function shared with tests (CCMath.HeightGradient)
        var height = new NativeArray<float>(cells, Allocator.TempJob);
        handle = new CopyHeightJob {
          Bake = config.Bake,
          Height = height,
        }.Schedule(cells, 4096, handle);
        handle = new BakeHeightGradientJob {
          Height = height,
          Indexer = fields.Indexer,
          CellSize = config.CellSize,
          SlopeMin = constants.f_slopeMin,
          SlopeMax = constants.f_slopeMax,
          DH = fields.DH,
        }.Schedule(cells, 1024, handle);
        handle = height.Dispose(handle);
      }

      handle = new BakeWalkableJob {
        Discomfort = fields.Discomfort,
        Walkable = fields.Walkable,
      }.Schedule(cells, 8192, handle);

      handle.Complete();

      var entity = state.EntityManager.CreateEntity();
      state.EntityManager.AddComponentData(entity, fields);
      state.EntityManager.AddComponentData(entity, new CCWalkabilityVersion { Version = 1 });

      state.Enabled = false;
    }

    public void OnDestroy(ref SystemState state)
    {
      if (!_fieldsQuery.IsEmptyIgnoreFilter) {
        _fieldsQuery.GetSingleton<GlobalFields>().Dispose();
      }
    }

    [BurstCompile]
    private struct CopyDiscomfortJob : IJobParallelFor
    {
      [ReadOnly] public BlobAssetReference<CCWorldBakeData> Bake;
      [WriteOnly] public NativeArray<float> Discomfort;

      public void Execute(int i) => Discomfort[i] = Bake.Value.Discomfort[i];
    }

    [BurstCompile]
    private struct CopyHeightJob : IJobParallelFor
    {
      [ReadOnly] public BlobAssetReference<CCWorldBakeData> Bake;
      [WriteOnly] public NativeArray<float> Height;

      public void Execute(int i) => Height[i] = Bake.Value.Height[i];
    }

    /// <summary>
    /// ∇h bake (spec §5.1): central differences, one-sided at edges, each
    /// component clamped to [f_slopeMin, f_slopeMax] so the topographical
    /// speed interpolation (spec §2.4) operates within its assumed domain.
    /// </summary>
    [BurstCompile]
    private struct BakeHeightGradientJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<float> Height;
      public GridIndexer Indexer;
      public float CellSize;
      public float SlopeMin;
      public float SlopeMax;
      [WriteOnly] public NativeArray<float2> DH;

      public void Execute(int i)
      {
        var dh = CCMath.HeightGradient(Height, Indexer, Indexer.Coord(i), CellSize);
        DH[i] = math.clamp(dh, SlopeMin, SlopeMax);
      }
    }

    [BurstCompile]
    private struct BakeWalkableJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<float> Discomfort;
      [WriteOnly] public NativeArray<byte> Walkable;

      public void Execute(int i) => Walkable[i] = Discomfort[i] < 1f ? (byte)1 : (byte)0;
    }
  }
}
