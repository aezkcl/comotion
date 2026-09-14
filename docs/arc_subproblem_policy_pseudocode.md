# ARC Subproblem Policy Pseudocode

## Scope

This document specifies the simplified ARC modification for the current ICRA paper.

The implementation should support:

- a history of resolved conflicts,
- a history of resolution attempts for unresolved conflicts,
- one `CreateSubProblem` function,
- two subproblem modes: `CSPACE` and `TEMPORAL`,
- three implemented robot-selection policies: `ARC_REPAIR_HISTORY`,
    `HISTORICAL_DIRECT_NEIGHBORS`, and `CURRENT_CONFLICT_COMPONENT`,
- existing ARC feasibility solvers through `SolveSubProblem`.

Learned policies remain outside the scope of this implementation.

---

# Data Structures

## MRMP Problem

MRMP_Problem = {
    Environment,
    Robots,
    Queries
}

## MRMP Subproblem

MRMP_SubProblem = {
    Environment,
    Robots,
    Queries
}

## Resolution Attempts

`Resolution_attempts[c]` stores all subproblem attempts made while attempting to resolve conflict `c`.

Each attempt should contain at least:

Attempt = {
    conflict,
    problem,
    solver,
    outcome
}

where:

outcome ∈ {SUCCESS, FAILURE}

## Resolved Conflict History

`Resolved_conflicts` stores the complete resolution-attempt history for conflicts that have been successfully resolved.

For a conflict c:

Resolved_conflicts.append(
    Resolution_attempts[c]
)

## Encountered Direct-Conflict History

For `HISTORICAL_DIRECT_NEIGHBORS`, `Encountered_conflicts` is initialized empty
for each `solve()` call. Each undirected robot pair stores the compact range
covering every collision encountered for that pair:

Encountered_conflicts[i][j] = {
    earliest_t,
    latest_t
}

A vertex collision at timestep `t` covers `[t, t]`. A segment collision with
`alpha > 0` covers `[t, t + 1]`. The current selected conflict is recorded only
after its robot set and initial temporal window have been computed, so selection
uses previously encountered conflicts only.

## Current Conflict Snapshot

For `CURRENT_CONFLICT_COMPONENT`, every inter-robot collision in the current
solution is streamed from timestep zero. The first collision retains its full
configuration data as the selected conflict. All collisions contribute only an
undirected edge and a compact covered-timestep range to `Current_conflicts`:

Current_conflicts[i][j] = {
    earliest_t,
    latest_t
}

The snapshot is complete only when the entire scan finishes. An interrupted or
cancelled scan is not used for robot selection.

---

# Algorithm 1: ARC

Input:
    MRMP_Problem
    selected_expansion_policy ∈ {CSPACE, TEMPORAL}
    selected_robot_policy ∈ {
        ARC_REPAIR_HISTORY,
        HISTORICAL_DIRECT_NEIGHBORS,
        CURRENT_CONFLICT_COMPONENT
    }
    initial_window

Output:
    Paths

P ← ∅
Resolved_conflicts ← ∅
Resolution_attempts ← ∅
Encountered_conflicts ← ∅

for each robot ri with query qi in MRMP_Problem.Robots do

    pi ← MotionPlanning(
        MRMP_Problem.Environment,
        {ri},
        {qi})

    P ← P ∪ {pi}

end for

if selected_robot_policy = CURRENT_CONFLICT_COMPONENT then
    c, Current_conflicts, complete ← StreamCurrentConflicts(P, start_t = 0)
    if not complete then
        return TIMEOUT
    end if
else
    c ← FindFirstConflict(P)
    Current_conflicts ← ∅
end if

while c ≠ ∅ do

    MRMP_SubProblem ← CreateSubProblem(
        c,
        P,
        MRMP_Problem.Environment,
        Resolved_conflicts,
        Resolution_attempts[c],
        selected_expansion_policy,
        selected_robot_policy,
        Encountered_conflicts,
        Current_conflicts,
        initial_window)

    if selected_robot_policy = HISTORICAL_DIRECT_NEIGHBORS then
        RecordDirectConflict(
            Encountered_conflicts,
            c)
    end if

    if MRMP_SubProblem = ∅ then
        return ∅
    end if

    resolution_paths, resolution_attempt ←
        SolveSubProblem(
            MRMP_SubProblem,
            c)

    Resolution_attempts[c].append(
        resolution_attempt)

    if resolution_paths ≠ ∅ then

        UpdateSolution(
            P,
            resolution_paths)

        Resolved_conflicts.append(
            Resolution_attempts[c])

        if selected_robot_policy = CURRENT_CONFLICT_COMPONENT then
            c, Current_conflicts, complete ←
                StreamCurrentConflicts(P, start_t = 0)
            if not complete then
                return TIMEOUT
            end if
        else
            c ← FindFirstConflict(P)
        end if

    end if

end while

return P

---

# Algorithm 2: CreateSubProblem

Input:
    Selected conflict c
    Paths P
    Global environment E
    Resolved_conflicts
    Resolution_attempts[c]
    selected_expansion_policy
    selected_robot_policy
    Encountered_conflicts
    Current_conflicts
    initial_window

Output:
    MRMP_SubProblem

X ← ExtractConflictCharacteristics(
    c,
    P,
    E)

R′, Tpolicy ← SelectRobotSet(
    c,
    X,
    P,
    Resolved_conflicts,
    selected_robot_policy,
    Encountered_conflicts,
    Current_conflicts,
    initial_window)

if selected_expansion_policy = CSPACE then

    E′, Q′ ← CreateCSpaceSubProblem(
        c,
        R′,
        X,
        P,
        E,
        Resolved_conflicts,
        Resolution_attempts[c])

else if selected_expansion_policy = TEMPORAL then

    E′, Q′ ← CreateTemporalSubProblem(
        c,
        R′,
        X,
        P,
        E,
        Resolved_conflicts,
        Resolution_attempts[c],
        Tpolicy)

end if

if E′ = ∅ then
    return ∅
end if

MRMP_SubProblem ← {
    Environment: E′,
    Robots: R′,
    Queries: Q′
}

return MRMP_SubProblem

---

# Algorithm 2a: SelectRobotSet

Input:
    Selected conflict c = (i, j, t, alpha)
    Conflict characteristics X
    Paths P
    Resolved_conflicts
    selected_robot_policy
    Encountered_conflicts
    Current_conflicts
    initial_window

Output:
    Robot set R′
    Optional policy temporal window Tpolicy

if selected_robot_policy = ARC_REPAIR_HISTORY then

    R′ ← ARCRepairHistoryClosure(
        c,
        X,
        P,
        Resolved_conflicts)

    return R′, ∅

else if selected_robot_policy = HISTORICAL_DIRECT_NEIGHBORS then

    R′ ← {i, j}
    earliest_t, latest_t ← CoveredTimesteps(t, alpha)

    for each r in {i, j} do
        for each neighbor k and range [first_t, last_t]
            in Encountered_conflicts[r] do

            R′ ← R′ ∪ {k}
            earliest_t ← min(earliest_t, first_t)
            latest_t ← max(latest_t, last_t)

        end for
    end for

    Tpolicy ← [
        max(0, earliest_t - initial_window),
        latest_t + initial_window
    ]

    return R′, Tpolicy

else if selected_robot_policy = CURRENT_CONFLICT_COMPONENT then

    R′ ← ConnectedComponent(
        Current_conflicts,
        seed_vertices = {i, j})

    earliest_t, latest_t ← CoveredTimesteps(t, alpha)

    for each edge (u, v) with range [first_t, last_t]
        in Current_conflicts do

        if u ∈ R′ and v ∈ R′ then
            earliest_t ← min(earliest_t, first_t)
            latest_t ← max(latest_t, last_t)
        end if

    end for

    Tpolicy ← [
        max(0, earliest_t - initial_window),
        latest_t + initial_window
    ]

    return R′, Tpolicy

end if

Only adjacency entries for `i` and `j` are inspected. Neighbors added to `R′`
are not traversed by `HISTORICAL_DIRECT_NEIGHBORS`, so that policy does not
compute a transitive component. `CURRENT_CONFLICT_COMPONENT` does traverse the
current graph transitively and covers every collision edge inside the selected
component.

---

# Algorithm 3a: CreateTemporalSubProblem

Input:
    Selected conflict c
    Robot set R′
    Conflict characteristics X
    Paths P
    Global environment E
    Resolved_conflicts
    Resolution_attempts[c]
    Optional policy temporal window Tpolicy

Output:
    Local environment E′
    Local queries Q′

if Resolution_attempts[c] = ∅ then

    if Tpolicy ≠ ∅ then
        T ← ClipToGlobalPathHorizon(Tpolicy, P)
    else
        T ← PredictTemporalWindow(
            c,
            R′,
            X,
            Resolved_conflicts,
            P)
    end if

else

    Sprev ← MostRecentAttempt(
        Resolution_attempts[c])

    if Sprev.TemporalWindow
       reaches global temporal limit then

        return ∅

    end if

    T ← ExpandTemporalWindow(
        c,
        R′,
        X,
        Resolved_conflicts,
        P,
        Sprev)

end if

Q′ ← GenerateSubqueries(
    R′,
    T,
    P,
    c)

E′ ← GenerateCSpaceRegion(
    R′,
    Q′,
    E)

return E′, Q′

---

# Algorithm 3b: CreateCSpaceSubProblem

Input:
    Selected conflict c
    Robot set R′
    Conflict characteristics X
    Paths P
    Global environment E
    Resolved_conflicts
    Resolution_attempts[c]

Output:
    Local environment E′
    Local queries Q′

if Resolution_attempts[c] = ∅ then

    E′ ← PredictCSpaceRegion(
        c,
        R′,
        X,
        Resolved_conflicts,
        P)

else

    Sprev ← MostRecentAttempt(
        Resolution_attempts[c])

    if Sprev.LocalEnvironment = E then
        return ∅
    end if

    E′ ← ExpandCSpace(
        c,
        R′,
        X,
        Resolved_conflicts,
        P,
        Sprev)

end if

Q′ ← GenerateSubqueries(
    R′,
    E′,
    P,
    c)

return E′, Q′

---

# Algorithm 4: SolveSubProblem

Input:
    MRMP_SubProblem
    Selected conflict c
    Set of MRMP solvers S

Output:
    Local resolution paths
    Resolution attempt

local_paths ← ∅

for each solver s ∈ S do

    local_paths ← SolveMRMP(
        s,
        MRMP_SubProblem)

    if local_paths ≠ ∅ then

        attempt ← {
            conflict: c,
            problem: MRMP_SubProblem,
            solver: s,
            outcome: SUCCESS
        }

        return local_paths, attempt

    end if

end for

attempt ← {
    conflict: c,
    problem: MRMP_SubProblem,
    outcome: FAILURE
}

return ∅, attempt

---

# Intended Execution Flow

ARC
    ↓
FindFirstConflict
    ↓
CreateSubProblem
    ↓
PredictRobotSet
    ↓
Choose expansion mode

CSPACE:
    E′ → Q′

TEMPORAL:
    T → Q′ → E′

    ↓
SolveSubProblem
    ↓
SUCCESS / FAILURE

FAILURE:
    Resolution_attempts[c] is updated
    ↓
    same conflict remains
    ↓
    CreateSubProblem is called again
    using the previous attempt

SUCCESS:
    Resolution_attempts[c] is updated
    ↓
    UpdateSolution
    ↓
    append complete attempt history
    to Resolved_conflicts
    ↓
    FindFirstConflict again
    