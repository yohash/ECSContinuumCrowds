# ECSContinuumCrowds — Project Context

A faithful, Burst-first Unity DOTS/ECS implementation of Continuum Crowds (Treuille,
Cooper, Popović — SIGGRAPH 2006), reproducing the algorithmic core of the reference
C# implementation `yohash/ContinuumCrowds`, restructured for Burst-compiled,
job-parallel execution.

## Source-of-truth documents

- `docs/reference/TechnicalSpec_extracted.txt` — **the authoritative build spec**
  (v1.0). All architectural decisions are final unless a blocking technical
  contradiction is found (log it, propose an alternative, never silently deviate).
- `docs/reference/ResearchFoundation_extracted.txt` — research foundation backing
  the spec.
- Sibling checkout `/home/user/ContinuumCrowds` (yohash/ContinuumCrowds) — reference
  implementation. **Read-only; never modify.** Key files:
  - `Assets/ContinuumCrowds/Runtime/ContinuumCrowds/Algorithm/EikonalSolver.cs`
  - `Assets/ContinuumCrowds/Runtime/ContinuumCrowds/Algorithm/DynamicGlobalFields.cs`
  - `Assets/ContinuumCrowds/Runtime/ContinuumCrowds/Classes/Constants.cs`

**Precedence when in doubt: Technical Spec → yohash/ContinuumCrowds source → the
paper.** Log any case where the spec appears to contradict the repo instead of
guessing.

## Working rules

- Develop on branch `claude/continuum-crowds-ecs-p85sn0`; all changes in this repo only.
- Every ⚠ DIVERGENCE / ⚠ NOTE marker in the spec must appear as a code comment at the
  corresponding implementation site (incl. the root-selection experiment record §9.4
  and the predictive-velocity rationale §7.1).
- Parity before performance: Phases 1–2 prove reproduction of the repo & the paper's
  phenomena before any Phase-4 optimization.
- All hot-path code `[BurstCompile]`; systems are unmanaged `ISystem`; no managed
  allocations on per-frame/per-solve paths. Never `Entities.ForEach` / `IAspect`
  (obsolete in Entities 1.3).
- Shared math lives in a `CCMath` static class of inlined Burst-compatible functions
  used by jobs AND tests; one `EikonalUpdate` function shared by FMM and FIM.

## Targets

- ~10,000 units, ~5 concurrent groups/solves, 512×512 grid (1 m cells), solve tick
  10 Hz (staggered, multi-frame, double-buffered), zero main-thread stalls, no GC.
- Unity 6 LTS (project: **6000.3.6f1**), com.unity.entities **1.3.x** (1.3.15+),
  burst 1.8.19+, collections 2.5.x, mathematics bundled.
  (Entities packages are NOT yet in `Packages/manifest.json` — add when scaffolding.)

## Locked decisions (spec §1 — do not relitigate)

- **D1** Grid cells are NOT entities. Units & groups are entities; all field data in
  flat native containers (SoA), operated on by Burst jobs.
- **D2** One global stamped map (ρ, v̄, g, ∇h) — dense 512², no tiles. All indexing
  through one `GridIndexer` so paging could be swapped in later (Phase 5+ only).
- **D3** Gather-pattern stamping: spatial hash of units → parallel job over active
  cells; each cell accumulates from nearby units. No write races/atomics.
- **D4** Predictive **velocity**, not predictive discomfort (paper's §3.3 rejected —
  self-avoidance oscillation). Repo semantics: `v_predictiveSeconds`, `v_scaleMax/Min`,
  `v_dynamicFootprintThreshold`.
- **D5** Transient terrain-aware solve domains via BFS flood-fill from goal + group
  extent over walkability. No tiles, no seams; domains overlap freely across groups.
- **D6** Domain caching with invalidation triggers (goal changed; centroid moved >
  PadCells/2; any unit escaped domain; walkability version bump) + hysteresis pad
  (PadCells=16).
- **D7** Low-rate staggered multi-frame solves; double-buffered velocity fields with
  per-buffer domain snapshots; advection reads front buffer every frame; scheduler
  polls `IsCompleted` (never blocking `.Complete()` on solve chains).
- **D8** Hybrid eikonal: FMM (serial per solve, parallel across groups) below a
  domain-size threshold (placeholder 32,768 cells && ≥2 idle workers); FIM
  (parallel within solve) above.
- **D9** Eikonal quadratic roots: **weighted average** of both roots
  (maxWeight=2.5, minWeight=1.0) — repo behavior; keep repo's experiment commentary
  in code (max→diagonal bias; min→cardinals; mean/geomean→still diagonal;
  weighted mean shipped).
- **D10** Velocity gradient: central difference + infinity fallback (repo) is the
  shipping path; paper's upwind differencing preserved behind a toggle
  (`GradientScheme.CentralRepo | UpwindPaper`) — doubles as shockline debug tool.
- **D11** Per-frame passes: bilinear-sampled advection (IJobEntity, Euler
  integration), pairwise min-distance via spatial hash (symmetric half-push, one
  iteration/frame).

## Algorithm essentials (spec §2; repo conventions)

- **ENSW packing**: anisotropic fields (f, C) are `float4(E,N,W,S)` =
  `(+x, +y, −x, −y)`; direction tables `ENSW`/`ENSWint` index-aligned.
- **Into-cell rule (critical)**: speed/cost for cell M in direction d read
  ρ/v̄/g/∇h from `into = M + ENSWint[d]` (the cell being entered), NOT M. Repo:
  `xGlobalInto/yGlobalInto`. This is why a unit never self-obstructs.
- **Density/velocity stamping** (global, couples all groups):
  `ρ[c] = Σ w_i(c)·m_i`; `v̄[c] = ρ>0 ? Σ w_i(c)·m_i·v_i / ρ : 0`.
  ⚠ DIVERGENCE (repo, kept): mass scaling (paper has none).
- **Footprint kernel** (spec fills a hole the repo left open — paper §4.1 splat):
  fractional offset (Δx,Δy) from lower-left cell center, deposit on 2×2:
  `w_A=min(1−Δx,1−Δy)^λ, w_B=min(Δx,1−Δy)^λ, w_C=min(Δx,Δy)^λ, w_D=min(1−Δx,Δy)^λ`,
  ρ̄ = 1/2^λ (λ default 2 ⇒ ρ̄=0.25). Invariant: own cell ≥ ρ̄, any neighbor ≤ ρ̄,
  and config assert `f_rhoMin ≥ ρ̄` (isolated unit always moves at topographical
  speed). Editor-mode assertion test required.
- **Speed field f** per (cell M, dir d), into-cell reads:
  - into invalid (OOB or g≥1) → `f_speedMin`.
  - ρ_into < f_rhoMin → topographical:
    `f_T = f_speedMax + (dot(∇h[into],ENSW[d]) − f_slopeMin)/(f_slopeMax − f_slopeMin) · (f_speedMin − f_speedMax)`.
  - ρ_into > f_rhoMax → flow: `f_v = max(0, dot(v̄[into], ENSW[d]))` —
    **the max(0,·) clamp + directional dot IS lane formation; never remove**
    (repo changelog v0.2.7).
  - else lerp: `f = f_T + (ρ_into − f_rhoMin)/(f_rhoMax − f_rhoMin) · (f_v − f_T)`.
  - final `clamp(f, f_speedMin, f_speedMax)`.
- **Cost field**: `C = C_alpha + C_beta/f + C_gamma·g'/f` with `g' = clamp(g[into],0,1)`;
  `f==0` or invalid into → +∞. ⚠ DIVERGENCE (repo, kept): g clamped to [0,1], g≥1
  = absolutely impassable (boundaries folded into discomfort).
- **Eikonal update** (shared FMM/FIM; repo `EikonalUpdateFormula`): for cell n,
  `phi_m[dd] = phi[neighbor_dd] + C[n][dd]` (C indexed at the cell being updated,
  into-cell convention already baked into C — follow repo exactly);
  `phi_mx = min(E,W)`, `phi_my = min(N,S)`, C_mx/C_my = cost of chosen direction;
  discriminant `valTest = C_mx² + C_my² − 1/(C_mx²·C_my²)` (⚠ repo-specific, not in
  paper — preserve as-is); if `(phi_mx−phi_my)² > valTest` → 1-D solution
  `min + its cost`; else quadratic:
  `radical = sqrt(C_mx²·C_my²·(C_mx²+C_my²−(phi_mx−phi_my)²))`,
  `soln1,2 = (C_my²·phi_mx + C_mx²·phi_my ± radical)/(C_mx²+C_my²)`,
  `phi = (max·maxWeight + min·minWeight)/(maxWeight+minWeight)`.
  NaN guard: radical NaN → 1-D fallback. Never let NaN into φ.
  Repo FMM detail: goal cells seeded into queue at priority 0 (not pre-accepted);
  neighbors that are goal cells contribute `0 + C` (off-tile-goal guard — keep).
  Also: neighbors **in the goal set are skipped as update targets**, and accepted
  cells are never re-updated.
- **Gradient** (shipping): per axis, both neighbors ∞ → 0; one ∞ →
  `sign(phiHi − phiLo)` (±1 one-sided); else central `(phiHi − phiLo)/2`; then
  normalize (zero-safe). Edge cells use one-sided differences (repo enumerates all
  edge/corner cases).
- **Velocity**: `v = −f(direction faces) · normalize(∇φ)`:
  `v.x = dPhi.x>0 ? −f[W]·dPhi.x : −f[E]·dPhi.x`;
  `v.y = dPhi.y>0 ? −f[S]·dPhi.y : −f[N]·dPhi.y`.

## Constants (repo `Constants.cs`, verified against source)

| Name | Default | | Name | Default |
|---|---|---|---|---|
| u_unitRadialFalloff | 0 | | f_rhoMax | 0.8 |
| v_dynamicFootprintThreshold | 0.25 | | f_rhoMin | 0.3 |
| v_predictiveSeconds | 1.0 | | f_speedMin | 0 |
| v_scaleMax | 0.3 | | f_speedMax | 20 |
| v_scaleMin | 0.25 | | C_alpha | 1 |
| f_slopeMax | 1 | | C_beta | 1 |
| f_slopeMin | −1 | | C_gamma | 1 |
| maxWeight | 2.5 | | minWeight | 1.0 |
| λ (splat exponent, spec-added) | 2 (⇒ ρ̄=0.25) | | | |

Implement as `CCConstants : IComponentData` singleton, ScriptableObject-backed
authoring with Editor hot-reload.

## ECS architecture snapshot (spec §3–§4)

- **Unit entity**: `UnitTag`, `CCUnit { Mass, Radius, FootprintSize, GroupId(int, NOT
  shared component) }`, `UnitVelocity { float2 }`, `LocalTransform`. Regroup = write
  GroupId (no structural change).
- **GlobalFields singleton**: SoA `NativeArray`s — Rho(float), VAveAcc(float2),
  Discomfort(float), DH(float2), Walkable(byte); W,H,CellSize; Persistent.
- **Group entity**: `CCGroup { GroupId, Alpha/Beta/Gamma, Phase, ScheduleSlot,
  ActiveBuffer, LastSolveTime }` (JobHandle in managed side-table on scheduler, NOT
  in component), `GoalCell` buffer, `DomainCache { Cells(NativeList<int>),
  GlobalToLocal(hash map), cached centroids/radius, WalkabilityVersion, Valid }`,
  `GroupFieldBuffers { F,C(float4) Phi(float) scratch; Velocity0/1(float2);
  DomainSnapshot0/1 }`. Cleanup-component pattern for disposal;
  `Dispose(JobHandle)` if destroyed mid-solve.
- **Spatial hashes**: TWO — stamping hash (bucket = ceil(R_max) cells, 9-bucket
  query, solve ticks only) and min-distance hash (bucket = 2·maxRadius, every
  frame). `NativeParallelMultiHashMap<int, UnitStampData{Position, Velocity, Mass,
  FootprintSize}>` (32 B payload), allocate once with capacity ≥ unit count,
  Clear() per rebuild.
- **System order** in `CCSimulationSystemGroup`: Scheduler → SpatialHash → Stamping
  → Domain → Field → Eikonal → VelocityDerivation → Advection → MinDistance.
  Per solve tick: hash→stamp (once for all groups scheduled that tick) →
  per-group domain/fields/eikonal/velocity chained via JobHandle. Every frame:
  advection + min-distance.
- **Domain flood fill**: single Burst IJob BFS from goal cells over 4-connected
  walkable, admit if within padded AABB(goal ∪ unitExtent)+PadCells OR
  HorizonCells cap. Output compact Cells list + GlobalToLocal map + **precomputed
  per-cell int4 NeighborLocalIdx table** (E,N,W,S local idx, −1 absent) so hot
  loops do zero hashing. Out-of-domain neighbor = infinite cost; stall detector
  (speed≈0 for >1.5 s) → refresh with doubled pad.
- **Buffers/allocators**: Persistent for global fields + group buffers (1.5× growth
  reuse); WorldUpdateAllocator ONLY for scratch that doesn't outlive the frame
  (multi-frame chains use TempJob or persistent per-group scratch — the default).
- **Advection**: bilinear sample of front buffer via full-grid `localIdxLookup`
  per buffer (1 MB each — ship this, O(1) sampling); out-of-domain corners get
  weight 0 + renormalize; Euler integrate; escape check sets dirty flag; arrival
  events via ECB tag.
- **Scheduler**: SolveHz=10, GroupsPerTick=1; SlotCount=ceil(groups/GroupsPerTick);
  per-group refresh = SolveHz/SlotCount. Flip = poll IsCompleted → Complete() (free)
  → flip ActiveBuffer. Accepted relaxations (document, don't "fix"): staggered groups
  see different stamp snapshots; regrouped units sample stale field ≤ 1 interval.

## FMM / FIM notes

- FMM: indexed binary min-heap over native arrays (heap/key/heapPos, decrease-key)
  — ship first (~120 lines, mirrors repo queue). Lazy-deletion variant acceptable.
  Bucket/Dial's queue = Phase-4 candidate only.
- FIM: per-cell first (Jacobi, double-buffered φ for determinism), block-FIM
  (16×16) as Phase-4 upgrade. eps ≈ 1e-3 (config). Active list: two NativeLists +
  byte state array.
- Parity: FMM vs FIM within eps on identical (domain, C) — standing automated test.
  Known risk: weighted-root blend may destabilize FIM convergence → fallback:
  iterate with pure max root + single weighted-blend post-pass; log which shipped.

## Phased plan (spec §15)

- **Phase 0** Scaffolding: packages, CCConstants authoring, GridIndexer, global
  fields, heightmap/discomfort bake, debug grid visualizer (ρ, g, φ, velocity
  arrows — invest here).
- **Phase 1** Core solve, single group, full-grid domain, naive scatter stamp, FMM,
  central-diff velocity, advection, min-distance. Validate: φ parity vs reference
  repo on handcrafted 8×8 grids (repo imported as managed oracle in editor test
  asm); optimal path around obstacle; gradient-diff visualizer; Burst Inspector
  clean.
- **Phase 2** Gather stamping + splat kernel + ρ̄ assert + predictive velocity
  (validated vs brute-force scatter reference). Validate: lane formation; no
  oscillation (A/B predictive on/off); density continuity.
- **Phase 3** Flood-fill domains + caching + stagger scheduler + double buffering.
  Validate: 4-group vortex; canyon map one domain; cache hit-rate ≫90%; zero
  main-thread stalls; escape/stall triggers.
- **Phase 4** FIM + hybrid + optimizations (block-FIM, bucket queue, job fusion,
  branchless select speed field). Crossover benchmark harness is a deliverable.
- **Phase 5** (as needed) Paged storage behind GridIndexer; region-AABB walkability
  invalidation; agent-integration hooks.

Emergent acceptance criteria (no special-case code): lane formation, 4-group
vortex, smooth congestion avoidance, no local-minima trapping.

## Reference-repo implementation quirks worth remembering

- Repo's `FlatSpeed` helper: `f_speedMax + (−f_slopeMin)/(f_slopeMax−f_slopeMin) ·
  (f_speedMin−f_speedMax)` (speed on flat ground).
- Repo velocity-average: `v /= r` only when `r != 0` (`computeAverageVelocityField`).
- Repo eikonal validity rules (`EikonalSolver.cs`) — reproduce exactly for parity:
  - Update **targets** (`isEikonalLocationValidAsNeighbor`): in-bounds, g<1,
    NOT accepted, NOT a goal cell (goal φ stays 0 forever).
  - `phi_m` **reads** (`isEikonalLocationValidToMoveInto`): in-bounds, g<1, and
    NOT accepted — i.e. previously-accepted cells are treated as ∞ in later
    updates. This deviates from textbook FMM (which reads accepted values).
    It still propagates because `markAccepted(current)` runs AFTER
    `EikonalUpdateFormula(current)`: the cell being finalized is readable by its
    neighbors at that moment, and considered cells (finite tentative φ) are
    readable. A goal cell read as a neighbor contributes `0 + C` via the explicit
    goal-set check. Verify parity on handcrafted 8×8 grids before optimizing.
- Repo unit stamping floors `unit.Corner` and iterates the footprint array; mass
  scales both ρ and momentum.
