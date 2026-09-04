# ARC source walkthrough

This guide follows the current versions of:

- `src/comotion/planning/ARC.h` (558 lines)
- `src/comotion/planning/ARC.cpp` (2226 lines)

Line numbers refer to those files as they existed when this guide was written.
Blank lines, closing braces, and repeated JSON assignments are grouped with the
code they belong to.

## 1. Where ARC sits in CoMotion

```text
apps / benchmark configuration
        |
        v
MultiRobotPlanner (abstract interface and shared result state)
        |
        v
ARC
  |-- reads MultiRobotProblem
  |     |-- RobotModel instances
  |     |-- starts/goals
  |     |-- environment CollisionChecker
  |     `-- resolution and vmax
  |
  |-- creates initial paths with OMPL RRTConnect
  |-- finds robot/robot conflicts with ConflictChecker
  |-- repairs conflicts with
  |     |-- PrioritizedSTRRT
  |     |-- CompositeRRT
  |     `-- bounded AORRTC helpers (optional)
  |-- stores results as vector<Path>
  |
  |-- base class of AOARC
  `-- base class of ParallelARC
```

The most useful call graph is:

```text
ARC::solve
  -> resetArcSolveState
  -> planIndividualPaths
       -> planIndividualPath, once per robot
       -> finishInitialIndividualPaths
  -> initializeConflictScanStarts
  -> loop
       -> conflictScanOptions
       -> ConflictChecker::findConflicts
            -> expandConflictForSubproblem
                 -> subproblemRobotsForConflict
       -> resolveConflictOnPaths
            -> solveSubproblemOnPaths
                 -> validate local endpoints
                 -> PrioritizedSTRRT::solve
                 -> CompositeRRT::solve (fallback)
                 -> nextExpansionWindowAfterAttempt (on failure)
                 -> spliceSolutionIntoPaths (on success)
       -> resetConflictScanStartsForRobots
       -> recordAppliedRepairHistory
```

## 2. `ARC.h`, line by line

### Lines 1-20: header protection and dependencies

- Line 1, `#pragma once`, prevents duplicate inclusion.
- Lines 3-5 import `ConflictChecker`, the abstract `MultiRobotPlanner` base,
  and path-simplification settings.
- Lines 6-18 import standard types used in ARC's public and protected API:
  clocks, callbacks, maps, optional values, sets, strings, pairs, and vectors.
- Line 20 opens namespace `comotion`; all ARC symbols live there.

### Lines 22-24: purpose and inheritance

- Lines 22-23 state the design: independently plan robot paths, then repair
  conflicts through local subproblems.
- Line 24 declares `ARC : public MultiRobotPlanner`. This is the central
  codebase connection. `MultiRobotPlanner` supplies `problem_`,
  `planning_seed_`, `cancel_requested_`, solution metrics, JSON statistics,
  and conversion helpers. ARC must implement `solve`, `getSolutionPaths`, and
  `name`.

### Lines 26-42: public planner identity

- Lines 26-31 define `ExpansionPolicy`: linear, logarithmic, exponential, or a
  caller-provided sequence of width multipliers.
- Lines 33-37 define which local repair planners are enabled: both,
  prioritized only, or composite only.
- Line 38 aliases a no-argument boolean callback for cancellation.
- Line 40 declares the main algorithm, `solve(double timeLimit)`.
- Line 41 declares result retrieval as one `Path` per robot.
- Line 42 identifies this planner as `"ARC"` to app/benchmark code.

### Lines 44-67: main temporal-window schedule

- Line 44 clamps the initial half-window to at least one timestep.
- Lines 45-47 accept a positive finite expansion step and replace invalid input
  with `1.0`.
- Lines 48-59 document the exact growth formulas and global-fallback rules.
- Lines 60-62 select the main expansion policy.
- Lines 63-64 expose the selected policy.
- Line 65 declares validation/storage of custom multipliers.
- Lines 66-68 expose those multipliers without copying.

These settings affect repair attempts only; they do not affect initial
single-robot planning.

### Lines 70-128: endpoint-validity schedule

A local repair cannot be solved if its fixed start or goal composite
configuration already contains a robot/robot collision. ARC therefore has a
separate schedule for finding a window with valid endpoints.

- Lines 70-77 explain inheritance: unless overridden, endpoint-search settings
  use the main schedule.
- Lines 78-89 set, clear, read, and inspect inheritance of the endpoint-search
  expansion step.
- Lines 90-102 do the same for its policy.
- Lines 103-118 do the same for custom multipliers.
- Lines 119-128 control symmetric versus asymmetric endpoint search. Symmetric
  growth expands both ends. Asymmetric growth holds a valid side fixed and
  grows only invalid sides.

Once ARC has found valid endpoints, it locks that interval's midpoint and base
width and never re-enters this phase for the same conflict repair.

### Lines 130-158: local planner controls

- Lines 130-135 document and set the non-global CompositeRRT iteration cap.
- Lines 136-144 set an optional CompositeRRT extension range. Non-positive
  input restores OMPL automatic range selection.
- Lines 145-147 select/read the local solver hierarchy.
- Lines 148-155 set/read the per-robot PrioritizedSTRRT iteration cap.
- Lines 156-158 control local C-space boxes and their margin.
- Line 159 sets a minimum joint-range width, important when an original robot
  segment is nearly stationary.

### Lines 160-200: path simplification

- Lines 160-166 normalize and store initial-path simplification options.
- Lines 167-176 optionally give conflict repairs different options; clearing
  makes them inherit initial options.
- Lines 177-183 provide a shortcut-specific compatibility setter/getter.
- Lines 184-191 enable/read initial-path simplification.
- Lines 192-199 enable/read local-repair simplification.
- Lines 200-204 make one boolean control both phases.

### Lines 206-228: bounded and compatibility settings

- Lines 206-216 set or clear a global makespan bound in native integer
  timesteps.
- Lines 217-222 configure an epsilon removed from non-global bounded local
  repairs, forcing measurable progress toward the bound.
- Lines 224-229 map local window duration to the STRRT time-coordinate upper
  bound; invalid values restore factor `4.0`.
- Lines 231-240 are deprecated no-op methods retained so older callers still
  compile. Local solvers now always receive all remaining global wall time.

### Lines 242-267: summarized statistics structure

`ArcPlannerStatsSummary` is a compact intermediate representation shared with
`ParallelARC`. Its fields describe configuration, conflict/attempt counts,
expansion counts, initial planning time, simplification time, conflict-find
and conflict-resolution time, and number of subproblem batches.

### Lines 269-276: process CPU snapshot

`ProcessTreeCpuUsageSnapshot` stores CPU seconds for this process and its child
processes. This matters mainly for parallel conflict detection.

### Lines 278-306: repair results and history

- Line 278 aliases `steady_clock` as ARC's monotonic clock.
- `RepairWindow` (lines 280-284) stores an interval and IDs of history events
  merged into it.
- `AppliedRepairHistoryEvent` (286-291) records which robots were repaired
  together and where.
- `RepairOutcome` (293-299) returns success, final interval/team, and optionally
  local patch paths.
- `ExpansionScheduleState` (301-313) keeps the two schedule phases and fixed
  integer geometry. Values are doubled (`center_twice`, `half_width_twice`) so
  half-timestep midpoints can be represented exactly without floating-point
  drift.

### Lines 315-349: per-attempt telemetry

- `RepairAttemptPhase` distinguishes the original window, endpoint-validity
  expansions, and main expansions.
- `RepairAttemptEvent` records one trip through the local-repair loop: conflict,
  team, phase/index, interval, endpoint validity, which solvers ran, outcome,
  and fixed main-window geometry.
- `effective_global` means the schedule emitted `[0,max_t]`.
- `temporal_full_window` means solver behavior covers all actual waypoints,
  `[0,max_t-1]`; this distinction explains the two similar booleans.

### Lines 351-362: one initial robot plan result

`IndividualPlanResult` packages success/status, path, arrival time, wall/CPU
measurements, and an error message. This structure is also useful to
`ParallelARC` workers.

### Lines 364-381: initial-path helper declarations

- `resetArcSolveState` clears all state from the previous call.
- `planIndividualPath` plans exactly one robot.
- `recordInitialIndividualPlanStats` accumulates simplification time.
- `finishInitialIndividualPaths` computes sum-of-cost and makespan.
- `planIndividualPaths` loops over robots while recomputing remaining wall time.

### Lines 383-429: local repair and expansion declarations

- `solveSubproblemOnPaths` is the core repair attempt loop. It may directly
  splice a solution or return patch paths to a parallel caller.
- Lines 398-429 declare the schedule helpers. Public-looking simple overloads
  use current settings; stateful overloads preserve the original search
  geometry across retries.
- The convenience overload around lines 420-426 treats start and goal validity
  as one boolean.

### Lines 431-453: splice and repair wrappers

- `symmetricWindowFromGeometry` converts doubled midpoint/half-width geometry
  back to clamped integer bounds.
- `absoluteExpansionWindow` calculates an absolute target from the fixed base,
  rather than repeatedly growing the previous rounded interval.
- The two `spliceSolutionIntoPaths` overloads differ only in whether patches
  are values or pointers.
- `resolveConflictOnPaths` wraps the core repair loop into `RepairOutcome`.

### Lines 455-478: telemetry helpers

These declarations convert ARC's internal counters and detailed attempt records
into stable JSON fields consumed by benchmark tooling. They do not change the
planned paths.

### Lines 480-504: incremental conflict scan

- `initializeConflictScanStarts` allocates one scan frontier per unordered
  robot pair.
- `conflictScanOptions` exports those frontiers to `ConflictChecker`.
- `applyConflictScanProgress` records how far each pair was safely scanned.
- `resetConflictScanStartsForRobots` rewinds only pairs involving changed
  robots.
- `updateDerivedConflictScanStart` keeps a coarse minimum for diagnostics.

### Lines 506-536: repair history and team expansion

- `recordAppliedRepairHistory` stores successful team/window repairs.
- `expandConflictForSubproblem` turns a pairwise `Conflict` into a
  `SubproblemConflict`.
- `subproblemRobotsForConflict` computes transitive closure over overlapping
  old repair windows. This is ARC's adaptive robot-team growth.
- `repairWindowsForRobots` exposes pair history to subclasses/tests.
- `conflictWindowStart` gives the initial left boundary.

### Lines 537-554: persistent configuration and run state

- `solution_paths_`: current global paths and final result.
- `repair_window_schedule_` and `applied_repair_history_events_`: coordination
  history.
- Pair scan frontiers and true arrival timesteps support incremental checking
  and metrics.
- Defaults: initial half-window 20, linear step 20, custom sequence
  `1,1,1,2,2,2,4,8`, symmetric endpoint search, both solvers, five prioritized
  iterations, C-space bounds enabled, simplify initial paths only, STRRT factor
  4.
- Remaining fields are counters and wall/CPU timing vectors reset per solve.

### Lines 556-558: aliases and namespace close

The aliases `ArcLocalSolverMode` and `ArcExpansionPolicy` provide shorter public
names without creating new types.

## 3. `ARC.cpp`, line by line

### Lines 1-23: implementation dependencies

- Line 1 connects declarations to definitions by including `ARC.h`.
- Lines 2-6 import bounded AORRTC adapters, `CompositeRRT`, deterministic seed
  derivation, `PrioritizedSTRRT`, and invariant checks.
- Lines 7-10 import OMPL RNG/state-space/SimpleSetup/RRTConnect APIs used for
  independent paths.
- Lines 11-23 provide algorithms, clocks, math, diagnostics, containers, and
  POSIX CPU accounting.

### Lines 25-137: file-local helpers

The first anonymous namespace hides implementation details from other
translation units.

- Lines 29-36 measure monotonic elapsed nanoseconds.
- Lines 38-43 sum timing vectors.
- Lines 45-65 measure process CPU time with a platform fallback.
- Lines 67-71 convert POSIX `timeval` to seconds.
- Lines 73-83 and 85-98 translate enums to stable JSON strings.
- Lines 100-103 test inclusive repair-window intersection.
- Lines 105-116 compute sum-of-cost (sum of arrivals) and makespan (maximum
  arrival).
- Lines 118-137 subclass OMPL samplers/planners solely to seed their protected
  RNGs deterministically.

### Lines 139-171: multiplier validation

A second anonymous namespace is inside `comotion`.

- `validateArcExpansionMultipliers` rejects empty, non-finite, zero, or negative
  sequences with descriptive exceptions.
- The two setters validate before moving caller vectors into ARC state.

### Lines 174-190: simple schedule entry points

`nextExpansionWindow` and `nextInitialValidExpansionWindow` forward the current
bounds, index, policy, step, and multipliers to the general schedule function.
They are stateless helpers primarily useful in tests.

### Lines 191-207: convert fixed geometry to integer bounds

- Compute `(center-halfWidth)/2` and `(center+halfWidth)/2` using `long double`.
- Floor the left edge, ceil the right edge, and clamp to `[0,max_t]`.
- This guarantees the integer interval contains the desired continuous
  interval.

### Lines 209-296: calculate one absolute expansion

- Lines 216-221 immediately preserve the global sentinel `[0,max_t]`.
- Lines 223-233 sanitize the base half-width and configured step.
- Lines 234-264 apply the selected formula. Exponential growth uses `ldexp` to
  represent multiplication by powers of two; custom growth returns global once
  its sequence ends.
- Lines 266-274 safely ceil/saturate the target and convert it to an interval.
- Lines 275-282 allow repeated custom windows deliberately (for repeated
  independent attempts) but force formula schedules global if rounding/clipping
  produces no growth.
- Lines 284-294 make logarithmic growth jump global when both remaining tails
  are at most 10% of the horizon.

### Lines 298-310: lock main schedule geometry

The first valid-endpoint interval becomes the main schedule's fixed midpoint
and base half-width. The guard prevents later attempts from changing it.

### Lines 312-339: endpoint-validity expansion

- Calculate a symmetric candidate around the original initial-search geometry.
- In symmetric mode both sides may grow.
- In asymmetric mode only an invalid start and/or goal side grows; a valid side
  remains anchored.
- If a non-custom policy cannot grow, return global.

### Lines 341-383: choose endpoint or main schedule

- `nextMainExpansionWindow` uses locked valid-window geometry and main settings.
- `nextExpansionWindowAfterAttempt` initializes original search geometry once.
- If both endpoints are valid, it establishes main geometry.
- Until that point it increments `initial_valid_expansion_index`; afterward it
  increments `main_expansion_index`.
- It records which schedule generated the next interval for counters/telemetry.

### Lines 385-401: stateless expansion convenience function

This derives midpoint/base width from the interval passed by the caller and
forwards to `absoluteExpansionWindow`.

### Lines 403-438: reset one run

`resetArcSolveState` calls the base-class metric reset, clears paths/history,
frontiers, arrivals, attempt records and timing vectors, restores run-local
flags, and zeroes every counter. Configuration such as window size and solver
mode is intentionally retained.

### Lines 440-521: incremental pairwise scan frontiers

- Allocate `N choose 2` frontiers at zero.
- Build validation options with environment checks disabled and one start per
  robot pair.
- Validate returned frontier vector size, safely convert to `int`, and update
  the coarse minimum.
- After a repair, rewind only pairs touching a changed robot to at most the
  repair start. Pairs between unchanged robots retain their proven-safe prefix.

### Lines 523-603: successful-repair history

- Reject malformed intervals/robot IDs and deduplicate the team.
- Assign one history event ID.
- For every ordered robot pair in the team, insert the interval into a sorted
  list, merging overlapping intervals and deduplicating contributing event IDs.
- Storing both `(a,b)` and `(b,a)` makes later team expansion straightforward.

### Lines 605-644: plan all robots independently

- Resize output and arrival arrays.
- For each robot, recompute global remaining wall time.
- Call `planIndividualPath`; any timeout/non-exact result aborts initialization.
- Move each successful path into the working set and store its true arrival.
- Compute initial metrics and timing after all robots succeed.

### Lines 646-785: plan one robot

- Initialize an `IndividualPlanResult` and fetch robot model/DOF.
- Lines 656-704: if bounded mode is enabled, call
  `aorrtc::solveSingleRobotBounded`, require an exact nonempty path, and verify
  start/goal anchors within `1e-6`.
- Lines 706-734: otherwise create this robot's OMPL `SpaceInformation`, optional
  deterministic sampler, `SimpleSetup`, RRTConnect planner, and scoped start/
  goal states.
- Lines 736-751 solve with the entire supplied remaining budget and reject all
  statuses except `EXACT_SOLUTION`.
- Lines 753-760 optionally simplify, interpolate the OMPL geometric path, then
  convert it to CoMotion `Path` timesteps using inherited
  `omplPathToPath`.
- Lines 761-783 store arrival, reject empty results, verify exact anchors, and
  return success/CPU time.

Independent means robot/robot collisions are ignored here. Each robot's
`SpaceInformation` still validates its own model against the environment.

### Lines 787-799: initial statistics and metrics

Accumulate simplification wall time, then compute sum-of-cost and makespan from
the true arrival array. Paths are not artificially padded to equal lengths.

### Lines 801-842: initialize one conflict repair

- Extract the involved robot team and cancellation callback.
- Set `max_t` to the largest involved arrival plus one.
- Clamp the conflict's proposed window into that horizon.
- Initialize two-phase expansion state, a unique repair ID, and attempt phase.
- `recordWindow` exposes the latest interval to the caller even on failure.

### Lines 844-889: start each repair attempt

Every loop iteration:

1. records the interval;
2. increments attempt counters;
3. appends a detailed `RepairAttemptEvent`;
4. identifies a schedule-global interval;
5. recomputes remaining global wall time;
6. stops on cancellation or exhausted budget.

### Lines 891-924: construct the local `MultiRobotProblem`

- Copy environment obstacle spheres/cylinders, velocity, and resolution.
- For each involved global robot, sample its current path at `start_t` and
  `end_t`.
- Add the same robot model to the local problem with those sampled anchors.
- Local robot index `i` corresponds to `involved_robots[i]` globally.

### Lines 925-949: validate local anchors

- Build composite start and goal configuration arrays.
- Ask the local collision checker whether all involved robots can jointly
  occupy each endpoint.
- Record start/goal validity separately.
- On the first attempt where both are valid, lock main expansion geometry.

### Lines 951-1032: local C-space boxes

- Treat `[0,max_t-1]` as the actual full path horizon.
- For non-global attempts, scan each involved robot's old segment for per-joint
  minima/maxima.
- Expand each range by `cspace_bound_margin_`, clamp to physical joint limits,
  and enforce `min_cspace_bound_range_` where possible.
- Install this box on the corresponding local robot.
- Full-horizon repairs use normal full joint limits.

### Lines 1033-1063: derive bounded local makespan

When a global bound exists:

- Find each involved robot's slack to the global bound and keep the minimum.
- Local bound = old window span + minimum slack.
- For non-global attempts subtract the configured epsilon.
- If the raw bound is no larger than epsilon, skip this local solver attempt
  and expand instead.

### Lines 1065-1091: choose enabled local solvers

- A closure recomputes remaining global wall time before each layer.
- Solver calls occur only when both endpoints are valid and bounded epsilon did
  not force a skip.
- Unbounded `Both` means prioritized then composite.
- Bounded mode warns once, disables prioritized repair, and always enables the
  bounded composite helper.

### Lines 1092-1150: layer 1, PrioritizedSTRRT

- Convert window timesteps to seconds and multiply by the STRRT span factor.
- Configure the local planner not to persist at goals and not to equalize path
  lengths, because these paths are patches that leave the local problem.
- Use bounded time, K-nearest rewiring, first exact solution, configured
  iteration cap, planning seed, and cancellation.
- Give it all remaining wall time.
- On exact success, optionally return patches, optionally splice immediately,
  mark telemetry, and return `true`.

### Lines 1152-1247: layer 2, composite repair

- Recompute remaining time and stop on cancellation/exhaustion.
- Resolve conflict-specific simplification settings.
- Bounded mode calls `aorrtc::solveCompositeBounded` with a makespan metric,
  local bound, optional sample/vertex caps, and simplification.
- Unbounded mode configures `CompositeRRT` with seed, simplification, optional
  range, optional makespan metric, cancellation, and iteration cap. The cap is
  removed for a full-horizon attempt.
- On exact success, return/splice patches, mark telemetry, and return `true`.

### Lines 1249-1313: failure and expansion

- Recheck cancellation and global time.
- If schedule-global `[0,max_t]` already failed, return false.
- Save old bounds and schedule indices.
- Ask `nextExpansionWindowAfterAttempt` for endpoint-validity or main growth.
- Update phase/index and expansion counters.
- Repeated custom multipliers are allowed to rerun the same interval.
- Any other no-growth result means the schedule is exhausted.
- Mark the event `expanded` and loop.

### Lines 1314-1324: splice overload adapter

Convert `vector<Path>` to `vector<const Path*>` and call the single actual
implementation. This avoids duplicating splice logic and lets parallel ARC
provide patch pointers.

### Lines 1326-1445: splice patches into global paths

For each involved robot:

- Validate that a corresponding non-null patch exists.
- Clamp window bounds and sample exact old boundary configurations.
- Require patch start/end anchors to match the old path within `1e-6`.
- Build a new sparse path with monotonically increasing timestep metadata.
- Copy the unchanged prefix before `window_start`.
- Map patch-relative timesteps onto the global window start.
- Compute how much shorter/longer the patch is than the replaced interval.
- Shift every unchanged suffix waypoint by that signed difference.
- Replace the global path and update the robot's true arrival.

After all robots, recompute sum-of-cost and makespan.

### Lines 1446-1476: result wrapper

`resolveConflictOnPaths` calls the repair loop, always returns the final attempted
interval/team, and includes patch paths when the caller requested solve-only
rather than immediate application.

### Lines 1477-1584: stable summary JSON

- Copy run counters/timing totals into `ArcPlannerStatsSummary`.
- Convert that summary to JSON.
- Emit both current field names and legacy aliases so existing benchmark scripts
  continue to work.
- Optionally include per-round detection/resolution timing vectors.

### Lines 1586-1661: CPU and conflict-find timing JSON

- On POSIX, sample user+system CPU for this process and completed children.
- Compute a nonnegative delta.
- Serialize main process, build-worker, collision-worker, and critical-worker
  totals and per-round values. This is observability only.

### Lines 1662-1718: serialize every repair attempt

Map the phase enum to text and emit all `RepairAttemptEvent` fields. Missing
indices, geometry, and solver labels become JSON `null` rather than misleading
zero/empty values.

### Lines 1720-1803: aggregate attempts by conflict resolution

Group attempt events by `repair_id` and report wall/CPU time, number of calls to
each solver, whether expansions/global were reached, success, and whether the
first composite call solved the conflict.

### Lines 1805-1912: aggregate by expansion stage

Count validity checks, valid endpoints, solver attempts/invocations, global
attempts, and resolutions for:

- original windows;
- endpoint-validity expansions;
- each indexed main expansion;
- global main windows;
- all main attempts and all attempts.

Custom multipliers are attached to their corresponding index buckets.

### Lines 1913-1976: transitively expand the robot team

Start with the colliding pair in a set/worklist. For each current team robot,
inspect prior repair windows. If a prior pair window intersects the proposed
repair interval, add that teammate. Continue until no robot is added. Optional
trace records exactly which history edge caused every addition.

This is graph reachability where edges are prior pair repairs active in the
current temporal interval.

### Lines 1978-2004: lookup and pair-conflict expansion

- `repairWindowsForRobots` safely looks up stored pair history.
- `expandConflictForSubproblem` creates `[t-initial_window,t+initial_window]`,
  computes the transitive team, and copies pair-conflict details for tracing.

### Lines 2006-2110: begin top-level solve and stats finalizer

- Clear all run state and warn if bounded mode requested prioritized repair.
- Capture the global monotonic start time.
- Define a finalizer that publishes summary/timing JSON, a first-solution event,
  simplification configuration, expansion configuration, detailed repair
  events, and stage counts on every exit path.

### Lines 2112-2117: initial solution

Plan all robots independently. Failure returns `TIMEOUT` after publishing
statistics. Success initializes all pair conflict-scan frontiers at zero.

### Lines 2119-2185: one conflict-detection round

- Stop if the global limit has elapsed.
- Build incremental scan options and a timeout callback.
- Call `ConflictChecker::findConflicts` with environment checking disabled by
  the options, `max_conflicts=1`, `unique=true`, and ARC's conflict-expansion
  callback.
- Save returned per-pair frontiers and detailed wall/CPU instrumentation.
- If time expired, return `TIMEOUT`.
- If no conflict exists, finalize as first exact solution and return
  `EXACT_SOLUTION`.
- Otherwise count the conflict and select `conflicts.front()` (the earliest
  discovered conflict because the checker scans in composite timestep order).

### Lines 2187-2219: repair and repeat

- Time `resolveConflictOnPaths` and store wall/CPU duration.
- Failed repair returns `TIMEOUT`.
- Successful repair rewinds scan frontiers only for changed robots and records
  team/window history for future cascade merging.
- Loop back to conflict detection.
- Falling out through the global-time condition finalizes and returns timeout.

### Lines 2222-2226: result accessor

Return a copy of `solution_paths_`, one timestep-aware `Path` per problem robot.

## 4. Connections to the rest of the codebase

### `MultiRobotPlanner`

File: `src/comotion/planning/MultiRobotPlanner.h`

ARC inherits:

- `setProblem` and protected `problem_`;
- planning seed and cancellation callback;
- `solution_metrics_` and `planner_stats_json_`;
- `resetPlannerRunMetrics`, `setSolutionMetrics`, and conversion helpers.

This is why ARC can call `problem_->...`, `setSolutionMetrics(...)`, and
`omplPathToPath(...)` without defining them in `ARC.h`.

### `MultiRobotProblem`

File: `src/comotion/planning/MultiRobotProblem.h`

It owns robot models/start/goals, obstacles, collision backend, resolution,
velocity, and OMPL state-space creation. ARC reads the global object during
initial planning and creates smaller copies for local repairs.

### `Path`

File: `src/comotion/planning/Path.h`

A `Path` is a vector of joint configurations plus optional sparse integer
waypoint timesteps. ARC relies especially on:

- `config_at_timestep(t)` for interpolated boundary states;
- `timestep_at(i)` for sparse splicing;
- `arrival_timestep()` for metrics/horizon;
- the rule that a completed robot holds its final configuration.

Physical time is `timestep / problem.resolution()` seconds.

### `ConflictChecker`

Files:

- `src/comotion/collision/ConflictChecker.h`
- `src/comotion/collision/ConflictChecker.cpp`
- `src/comotion/collision/ValidationTypes.h`

It scans synchronized paths in timestep order, checks robot pairs, returns pair
scan progress, and invokes ARC's callback to expand pair conflicts into local
teams/windows.

### Local planners

- `src/comotion/planning/PrioritizedSTRRT.h/.cpp`: plans local robots in an
  order through space-time, treating prior paths as dynamic obstacles.
- `src/comotion/planning/CompositeRRT.h/.cpp`: plans all involved joints in one
  composite state space.
- `src/comotion/planning/AORRTCUtils.h/.cpp`: bounded single/composite helpers
  used when ARC has a global makespan bound.

ARC owns orchestration; these classes own the actual local search algorithms.

### App and benchmark factory

File: `apps/benchmark_app_common.hpp`

- `parseArcLocalSolverMode` and `parseArcExpansionPolicy` convert CLI strings.
- `makePlannerBlueprint` creates `std::make_shared<comotion::ARC>()` for
  algorithm `arc` and forwards CLI options through ARC's setters.
- Individual apps construct `MultiRobotProblem`, obtain the planner blueprint,
  inject problem/seed, call `solve`, and consume paths/stats.

### `AOARC`

Files: `src/comotion/planning/AOARC.h/.cpp`

`AOARC : ARC`. It first runs ordinary ARC to get a feasible solution, then
repeatedly creates fresh ARC attempts with the current makespan as a strict
bound. It retains an improved solution and publishes anytime solution events.

### `ParallelARC`

Files: `src/comotion/planning/ParallelARC.h/.cpp`

`ParallelARC : ARC`. It reuses protected ARC primitives—individual planning,
conflict expansion, local solve-only patches, splicing, scan frontiers, history,
and telemetry—but runs independent initial plans/conflict repairs through
worker processes and can process conflict batches.

### Build and public API

`src/CMakeLists.txt` compiles planning `.cpp` files into the `comotion` library
and installs `ARC.h`, `AOARC.h`, `ParallelARC.h`, and their dependencies as
public headers.

### Tests as executable documentation

- `tests/arc_exact_only_regression.cpp` exposes protected ARC helpers and tests
  expansion formulas, endpoint-validity phases, cascade merging, sparse
  splicing, exact-only acceptance, telemetry, and solver fallback.
- `tests/parallel_arc_regression.cpp` tests shared ARC behavior under process
  parallelism and pair-frontier rewinds.
- `tests/aorrtc_makespan_regression.cpp` tests bounded/AOARC integration.
- `tests/public_header_smoke.cpp` verifies the installed public API compiles.

## 5. Recommended debugging/read order

For a first debugger session, set breakpoints in this order:

1. `ARC::solve` at line 2006.
2. `ARC::planIndividualPath` at line 646.
3. `ConflictChecker::findConflicts` in `ConflictChecker.cpp`.
4. `ARC::expandConflictForSubproblem` at line 1988.
5. `ARC::solveSubproblemOnPaths` at line 801.
6. Endpoint validity around lines 925-949.
7. Prioritized solve around line 1121.
8. Composite solve around line 1205.
9. Window expansion around line 1262.
10. `ARC::spliceSolutionIntoPaths` at line 1326.

Watch these values:

```text
solution_paths_
conflict.timestep / conflict.robots
start_t, end_t, max_t
start_composite_ok, goal_composite_ok
expansion_schedule_state
local_paths
true_arrival_timesteps_
pair_conflict_scan_start_t_
repair_window_schedule_
```

That path through the code shows the algorithm; the JSON functions can be read
later as instrumentation rather than control flow.
