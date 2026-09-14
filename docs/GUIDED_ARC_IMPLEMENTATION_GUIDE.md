# GuidedARC Implementation Guide

This document is a code-reading reference for the new GuidedARC implementation.
It explains what each new type and function represents, where it lives, and how
the pieces are intended to interact.

## Implementation status

GuidedARC is being implemented and reviewed in small portions. The status of
the files described here is:

| File | Status | Purpose |
| --- | --- | --- |
| `src/comotion/planning/MRMPSubproblem.h` | Implemented | Describes one local multi-robot planning attempt. |
| `src/comotion/planning/ResolutionHistory.h` | Implemented | Preserves pending and completed conflict-resolution attempts. |
| `src/comotion/planning/GuidedARC.h` | Implemented | Declares the GuidedARC planner interface. |
| `src/comotion/planning/GuidedARC.cpp` | Partially implemented | Contains temporal subproblem construction; dispatch, C-space construction, solving, and the main loop remain planned. |

Declarations in `GuidedARC.h` that do not yet have definitions are documented
as **planned** below. This distinction is important when reading generated code:
temporal construction can compile, but the planner cannot yet be linked and run.

## High-level model

GuidedARC derives from the existing `ARC` planner. It is intended to reuse
ARC's initial single-robot planning, conflict detection, temporal expansion
schedules, local planners, path splicing, metrics, and visualization support.
GuidedARC adds an explicit choice of how a failed local subproblem expands and
a durable history of every attempted resolution.

The intended control flow is:

1. Produce initial paths using inherited ARC behavior.
2. Find a conflict in the current global paths.
3. Start a pending `ConflictResolutionHistory` for that conflict.
4. Construct a fresh `MRMPSubproblem` for the first attempt.
5. Solve the local subproblem and record the attempt, including failures.
6. If it fails, construct a larger fresh subproblem according to the selected
   temporal or configuration-space expansion mode.
7. If it succeeds, splice the local repair into the global paths.
8. Only after the splice succeeds, move the pending history into the resolved
   history collection.
9. Repeat until no conflicts remain, time expires, or expansion is exhausted.

## `MRMPSubproblem.h`

### `RobotCspaceRegion`

One `RobotCspaceRegion` records the configuration-coordinate bounds assigned to
one robot during one attempt.

| Field | Type | Meaning |
| --- | --- | --- |
| `global_robot_index` | `int` | Index of this robot in the original global `MultiRobotProblem`. `-1` means it has not been initialized. |
| `lower` | `std::vector<double>` | Inclusive lower bound for each configuration coordinate. |
| `upper` | `std::vector<double>` | Inclusive upper bound for each configuration coordinate. |

These are configuration-coordinate bounds, not necessarily revolute or
prismatic joint bounds. For a flying sphere, for example, they can represent
Cartesian coordinates.

The vectors should have the same length as the robot configuration. For every
coordinate `d`, the expected invariant is `lower[d] <= upper[d]`.

### `MRMPSubproblem`

`MRMPSubproblem` is the complete description of one local planning attempt.
Each expansion must create a new instance rather than modify one retained in
history.

| Field | Type | Meaning |
| --- | --- | --- |
| `problem` | `std::shared_ptr<MultiRobotProblem>` | The fresh local planning problem containing only participating robots, their local start/goal queries, the environment, and any local bounds. |
| `global_robot_indices` | `std::vector<int>` | Local-to-global robot mapping. Entry `i` gives the global index of local robot `i`. |
| `window_start_t` | `int` | Inclusive global timestep used for each local start configuration. |
| `window_end_t` | `int` | Inclusive global timestep used for each local goal configuration. |
| `global_end_t` | `int` | Inclusive final timestep across the participating global paths. |
| `cspace_regions` | `std::vector<RobotCspaceRegion>` | Bounds used for each participating robot in this attempt. |
| `uses_global_cspace` | `bool` | `true` when no local configuration-space restriction is applied. |

#### `spansGlobalTime()`

```cpp
bool spansGlobalTime() const noexcept;
```

Returns `true` when the attempt covers the complete inclusive temporal interval
from timestep `0` through `global_end_t`. Temporal expansion is exhausted after
such an attempt fails.

`noexcept` promises that this simple comparison does not throw an exception.

#### Mapping example

```text
global_robot_indices = {1, 4}
window_start_t       = 80
window_end_t         = 120
global_end_t         = 300
```

The local problem has two robots. Local robot `0` is global robot `1`, and local
robot `1` is global robot `4`. Their local starts come from global timestep `80`
and their local goals from timestep `120`. This attempt does not span global
time because it covers `[80, 120]`, not `[0, 300]`.

## `ResolutionHistory.h`

### `ResolutionSolver`

Records which local solver handled an attempt.

| Enumerator | Meaning |
| --- | --- |
| `None` | No local solver was used, or a solver has not yet been assigned. This can represent rejection before planning, such as invalid endpoints. |
| `PrioritizedSTRRT` | The inherited prioritized space-time solver was used. |
| `CompositeRRT` | The inherited composite local solver was used. |

### `ResolutionOutcome`

| Enumerator | Meaning |
| --- | --- |
| `Success` | The attempt produced a usable local repair. |
| `Failure` | The attempt did not produce a usable repair. |

### `ResolutionAttempt`

One immutable historical record of a local resolution attempt.

| Field | Type | Meaning |
| --- | --- | --- |
| `conflict` | `Conflict` | The original conflict associated with the attempt. |
| `subproblem` | `std::shared_ptr<const MRMPSubproblem>` | The exact subproblem attempted. The `const` target prevents later code from changing the recorded `MRMPSubproblem`. |
| `solver` | `ResolutionSolver` | Solver selected for this attempt. |
| `outcome` | `ResolutionOutcome` | Whether the attempt succeeded. |

The `MRMPSubproblem` contains a shared pointer to a `MultiRobotProblem`. That
nested problem is historical state and must also be treated as immutable.
Expansion must construct a fresh local problem; it must never change the local
problem retained by an earlier attempt.

### `ConflictResolutionHistory`

Groups all attempts made for one conflict.

| Field | Type | Meaning |
| --- | --- | --- |
| `conflict` | `Conflict` | Conflict being resolved. |
| `attempts` | `std::vector<ResolutionAttempt>` | Attempts in execution order. |

### `ResolutionHistory`

Owns at most one pending conflict plus all completed conflict histories.

#### `beginConflict`

```cpp
void beginConflict(Conflict conflict);
```

- `conflict`: conflict that GuidedARC is about to resolve.
- Creates a pending history with an empty attempt list.
- Throws `std::logic_error` if another conflict is already pending.

#### `recordAttempt`

```cpp
void recordAttempt(MRMPSubproblem subproblem,
                   ResolutionSolver solver,
                   ResolutionOutcome outcome);
```

- `subproblem`: passed by value so it can be moved into immutable history.
- `solver`: records the solver used.
- `outcome`: records success or failure.
- Allocates a `shared_ptr<const MRMPSubproblem>` that preserves this attempt.
- Throws `std::logic_error` if no conflict is pending.

After calling this function, construction of the next expansion must start from
a new `MRMPSubproblem` and a new `MultiRobotProblem`.

#### `markPendingConflictResolved`

```cpp
void markPendingConflictResolved();
```

Moves the pending history into `resolved_` and clears the pending slot. It
throws unless the pending history exists and its last attempt is successful.
GuidedARC must call it only after the successful local repair has been applied
to the global paths.

#### Inspection and reset functions

| Function | Result |
| --- | --- |
| `hasPendingConflict()` | Reports whether a conflict is currently being resolved. |
| `pendingConflict()` | Returns the pending history or throws if none exists. |
| `resolvedConflicts()` | Returns completed histories in resolution order. |
| `clear()` | Removes both pending and resolved history, normally at the start of a new solve. |

#### History transition example

```text
beginConflict(conflict A)
pending: A, attempts = []

recordAttempt(subproblem 1, CompositeRRT, Failure)
pending: A, attempts = [failure]

recordAttempt(subproblem 2, CompositeRRT, Success)
pending: A, attempts = [failure, success]

apply subproblem 2's repair to the global paths
markPendingConflictResolved()
resolved: [A: failure, success]
pending: none

beginConflict(conflict B)
resolved: [A: failure, success]
pending: B, attempts = []
```

Subproblem 1 and subproblem 2 are separate preserved objects. Expanding
subproblem 1 to produce subproblem 2 does not overwrite the first record.

## `GuidedARC.h`

### `GuidedARC`

```cpp
class GuidedARC : public ARC;
```

Inheritance gives GuidedARC access to ARC's protected implementation tools and
all public ARC configuration setters. The baseline `ARC` implementation remains
unchanged.

### `SubproblemExpansionMode`

| Enumerator | Intended behavior |
| --- | --- |
| `Temporal` | Expand the local time window after failure. |
| `CSpace` | Expand allowed configuration-coordinate regions after failure. |

#### `setSubproblemExpansionMode`

```cpp
void setSubproblemExpansionMode(SubproblemExpansionMode mode);
```

Selects which subproblem-construction function the dispatcher will use.

#### `subproblemExpansionMode`

```cpp
SubproblemExpansionMode subproblemExpansionMode() const noexcept;
```

Returns the currently selected mode. The default is `Temporal`.

### Main planner interface

#### `solve` (planned)

```cpp
ompl::base::PlannerStatus solve(double timeLimit) override;
```

- `timeLimit`: total wall-clock budget in seconds for the entire GuidedARC run.
- Will implement the main GuidedARC loop described above.
- Must reset history and inherited per-run state before starting.
- Must respect the total budget across initial planning and every repair.
- Returns an OMPL planner status such as exact solution or timeout/failure.

#### `name`

```cpp
std::string name() const override;
```

Returns `"GuidedARC"` for metrics, logs, and application output.

#### `resolutionHistory`

```cpp
const ResolutionHistory &resolutionHistory() const noexcept;
```

Provides read-only inspection of GuidedARC's history. The returned reference is
owned by the planner and remains valid while the planner exists and the member
is not replaced.

### Subproblem construction

#### `createSubProblem` (planned dispatcher)

```cpp
std::optional<MRMPSubproblem> createSubProblem(
    const SubproblemConflict &conflict,
    const std::vector<Path> &paths,
    const ConflictResolutionHistory &pending_history) const;
```

- `conflict`: expanded ARC conflict containing participating robots and the
  initial repair window.
- `paths`: current global paths from which local endpoints and bounds are
  sampled.
- `pending_history`: previous attempts for this same conflict.
- Dispatches to temporal or C-space construction according to
  `expansion_mode_`.
- Returns `std::nullopt` when no further subproblem can be constructed.

#### `createTemporalSubProblem` (implemented)

Has the same parameters and return type as the dispatcher. It:

1. Use `conflict.robots` as the local-to-global robot mapping.
2. Use the conflict window for the first attempt.
3. Use inherited ARC expansion schedules after failed attempts.
4. Sample local starts and goals at the inclusive window endpoints.
5. Hold shorter paths at their final configuration when paths have unequal
   arrival times.
6. Construct a fresh local problem for every call.
7. Return `std::nullopt` after failure of a full global-time attempt.

Invalid local endpoints are expected to produce a recorded failed attempt
without invoking a planner. The next construction then uses ARC's inherited
initial-valid-window expansion policy.

#### `createCSpaceSubProblem` (planned)

Has the same parameters and return type as the dispatcher. Its exact expansion
policy has not yet been approved. It will construct a fresh local problem and
expand configuration-coordinate bounds instead of the temporal interval.

Do not assume details such as growth factors, coordinate normalization, or the
condition for global C-space until this function is reviewed and implemented.

### Local solving

#### `SubproblemSolveResult`

| Field | Type | Meaning |
| --- | --- | --- |
| `paths` | `std::vector<Path>` | Local repair paths, populated on success. Local index order matches `MRMPSubproblem::global_robot_indices`. |
| `solver` | `ResolutionSolver` | Solver that produced the result, or `None`. |
| `outcome` | `ResolutionOutcome` | Success or failure. |

#### `solveSubProblem` (planned)

```cpp
SubproblemSolveResult solveSubProblem(
    const MRMPSubproblem &subproblem,
    double time_limit) const;
```

- `subproblem`: immutable description of the local attempt.
- `time_limit`: remaining local wall-clock budget in seconds.
- Will validate local endpoints before invoking a planner.
- Will reuse ARC's configured local-solver hierarchy.
- Returns enough information to record the attempt and, on success, splice its
  local paths into the global paths.

### Private state

| Member | Default | Meaning |
| --- | --- | --- |
| `expansion_mode_` | `Temporal` | Active expansion strategy. |
| `resolution_history_` | Empty | Pending and completed attempt history for the current solve. |

## Important inherited ARC parameters

GuidedARC inherits ARC's public setters. The most relevant parameters for the
planned implementation are listed here. Their storage and implementation live
in `src/comotion/planning/ARC.h` and `src/comotion/planning/ARC.cpp`.

| Setter or setting | Parameter meaning | GuidedARC relevance |
| --- | --- | --- |
| `setInitialWindow(int w)` | Initial number of timesteps around a conflict. Values are clamped to at least `1`. | Determines the first temporal repair interval produced by ARC conflict expansion. |
| `setExpansionStep(double e)` | Base temporal growth amount. Invalid values become `1.0`. | Used by the main temporal expansion schedule. |
| `setExpansionPolicy(ExpansionPolicy)` | Selects linear, logarithmic, exponential, or custom multiplied growth. | Controls temporal growth after valid-window solver failures. |
| Initial-valid expansion settings | Optional separate policy, step, multipliers, and symmetric/asymmetric behavior. | Used while searching for collision-free local start and goal endpoints. |
| `setLocalSolverMode(LocalSolverMode)` | Selects prioritized, composite, or configured solver hierarchy. | Controls which inherited local solver GuidedARC tries. |
| `setCspaceBoundMargin(float m)` | Fractional margin added around observed coordinate ranges. | Used when producing local configuration-coordinate bounds. |
| `setMinCspaceBoundRange(double r)` | Minimum allowed width per configuration coordinate. | Prevents local sampling regions from collapsing to an unusably narrow range. |
| `setResolution(size_t)` on the problem | Timesteps per second. | Defines the physical meaning of all integer path timesteps. |
| `setVmax(double)` on the problem | Maximum path speed. | Used by inherited path timing and interpolation behavior. |

Some command-line help currently labels expansion-policy options as
"baseline ARC only." GuidedARC application wiring has not yet been added, so
those labels and supported GuidedARC options must be revisited during app
integration.

## Time and path semantics

- All `window_*_t` values are integer timestep indices, not seconds.
- Windows are inclusive: `[window_start_t, window_end_t]` includes both ends.
- `Path::arrival_timestep()` returns the final inclusive timestep.
- `Path::config_at_timestep(t)` interpolates explicit-timestep paths.
- When `t` is later than a path's arrival, `config_at_timestep(t)` returns the
  final configuration. This is how unequal path lengths are handled.
- `global_end_t` is the maximum arrival timestep among participating paths.
- A temporal attempt spans global time when it covers `[0, global_end_t]`.

## Ownership and mutation rules

These rules are central to understanding and reviewing GuidedARC code:

1. `GuidedARC` owns `resolution_history_`.
2. `ResolutionHistory` owns pending and completed attempt records.
3. Each attempt owns a shared, const `MRMPSubproblem` record.
4. Each `MRMPSubproblem` refers to the fresh local `MultiRobotProblem` built for
   that attempt.
5. Later expansion must not modify an earlier local problem or its bounds.
6. The current global paths may change only after a successful repair is ready
   to be applied.
7. A conflict becomes resolved in history only after that global path update
   succeeds.

## Failure and exhaustion behavior

Expected conditions should be represented deliberately rather than hidden:

| Condition | Expected handling |
| --- | --- |
| No participating robots | Return `std::nullopt` or reject construction. |
| Robot index outside `paths` | Throw an out-of-range error because this indicates an internal contract violation. |
| Participating path is empty | Reject subproblem construction. |
| Invalid local start or goal | Record a failed attempt with solver `None`, then expand according to initial-valid settings. |
| Local solver fails | Record the selected solver and failure, then expand. |
| Full temporal interval already failed | Return `std::nullopt`; temporal expansion is exhausted. |
| No wall-clock budget remains | Stop and return the appropriate non-success planner status. |
| Successful local solve but failed global application | Do not mark the conflict resolved. |

## How to read generated GuidedARC code

For each function or change, check the following:

1. **File and status:** Is this an implemented header, a new `.cpp` definition,
   a test, or application wiring?
2. **Freshness:** Does each attempt construct a new local problem?
3. **Mapping:** Are local path indices translated through
   `global_robot_indices` before touching global paths?
4. **Time units:** Are integers treated as timesteps rather than seconds or
   vector lengths?
5. **Inclusive endpoints:** Are both window endpoints sampled and preserved?
6. **Expansion source:** Is temporal growth using inherited ARC settings rather
   than an unreviewed hard-coded policy?
7. **History order:** Is the attempt recorded before deciding whether to expand
   or finish?
8. **Commit point:** Is `markPendingConflictResolved()` called only after the
   repair is spliced into global paths?
9. **Budget:** Is remaining time derived from the total solve deadline?
10. **Scope:** Does the change stay within the function group currently under
    review?

## Naming glossary

| Name | Meaning |
| --- | --- |
| ARC | Adaptive Robot Coordination baseline planner. |
| GuidedARC | ARC variant that explicitly selects temporal or C-space subproblem expansion and records resolution history. |
| MRMP | Multi-Robot Motion Planning. |
| Global problem | Original problem containing every robot and complete start/goal queries. |
| Local problem | Fresh problem for the robots and bounds in one repair attempt. |
| Global paths | Current complete path set being repaired. |
| Local paths | Paths returned by solving one local subproblem. |
| Pending history | Attempts for the conflict currently being resolved. |
| Resolved history | Completed histories whose successful repair was applied globally. |
| Temporal expansion | Increasing the inclusive timestep window. |
| C-space expansion | Increasing allowed configuration-coordinate regions. |
| Splice | Replace the relevant segment of participating global paths with a successful local repair. |

## Related references

- `docs/arc_subproblem_policy_pseudocode.md`: current ARC subproblem policy
  pseudocode.
- `docs/CreateSubProblem.txt`: subproblem-dispatch pseudocode.
- `docs/CreateTemporalSubProblem.txt`: temporal-construction pseudocode.
- `docs/CreateCSpaceSubProblem.txt`: C-space-construction pseudocode.
- `docs/solveSubproblem.txt`: local solver pseudocode.
- `docs/current_ARC_WALKTHROUGH.md`: detailed baseline ARC walkthrough.

Update this guide whenever a new GuidedARC function is approved or an expansion
policy becomes concrete. In particular, replace the planned status and policy
warnings when `GuidedARC.cpp`, tests, CMake registration, and application wiring
are implemented.