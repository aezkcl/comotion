# ARC Subproblem Policy Pseudocode

## Scope

This document specifies the simplified ARC modification for the current ICRA paper.

The implementation should support:

- a history of resolved conflicts,
- a history of resolution attempts for unresolved conflicts,
- one `CreateSubProblem` function,
- two subproblem construction/expansion modes:
  - `CSPACE`
  - `TEMPORAL`
- existing ARC feasibility solvers through `SolveSubProblem`.

More sophisticated history-guided or learned policies are outside the scope of this implementation.

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

---

# Algorithm 1: ARC

Input:
    MRMP_Problem
    selected_expansion_policy ∈ {CSPACE, TEMPORAL}

Output:
    Paths

P ← ∅
Resolved_conflicts ← ∅
Resolution_attempts ← ∅

for each robot ri with query qi in MRMP_Problem.Robots do

    pi ← MotionPlanning(
        MRMP_Problem.Environment,
        {ri},
        {qi})

    P ← P ∪ {pi}

end for

c ← FindFirstConflict(P)

while c ≠ ∅ do

    MRMP_SubProblem ← CreateSubProblem(
        c,
        P,
        MRMP_Problem.Environment,
        Resolved_conflicts,
        Resolution_attempts[c],
        selected_expansion_policy)

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

        c ← FindFirstConflict(P)

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

Output:
    MRMP_SubProblem

X ← ExtractConflictCharacteristics(
    c,
    P,
    E)

R′ ← PredictRobotSet(
    c,
    X,
    P)

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
        Resolution_attempts[c])

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

# Algorithm 3a: CreateTemporalSubProblem

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

    T ← PredictTemporalWindow(
        c,
        R′,
        X,
        Resolved_conflicts,
        P)

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
    