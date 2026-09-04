# ICRA Implementation Scope

## Implement for this paper

- Existing ARC baseline
- Resolved conflict history
- Resolution attempt history
- One CreateSubProblem dispatcher
- Temporal-guided subproblem expansion
- C-space-guided subproblem expansion
- Existing SolveSubProblem solver hierarchy
- Benchmarking of existing examples
- Logging of planning time, success, number of attempts, and subproblem characteristics

## Do not implement yet

- Learned policy selection
- History-guided policy learning
- Automatic switching between CSPACE and TEMPORAL
- Multiple robot-set prediction policies
- Complex conflict-graph reasoning
- Additional expansion-policy learning

These are possible extensions for subsequent papers.