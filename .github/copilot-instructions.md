# Copilot / AI Agent Instructions — UR Simulator Iteration Process

These rules govern any automated or semi-automated work on this repo. They
exist to keep the multi-round improvement process reviewable.

## Authoritative planning docs

- [`docs/SHORTCOMINGS.md`](../docs/SHORTCOMINGS.md) — what is broken / missing.
  Reference items by their `#N` id.
- [`docs/IMPROVEMENT_PLAN.md`](../docs/IMPROVEMENT_PLAN.md) — rounds.
  Work on **one** round at a time, in order, unless the user reassigns.
- [`docs/ITERATION_LOG.md`](../docs/ITERATION_LOG.md) — append an entry per
  round. Never rewrite history there.

## Round workflow the agent must follow

1. **Announce the round** you are starting (round number + scope in 1 line).
2. **Read the relevant shortcomings** before writing code. Do not invent
   problems outside `SHORTCOMINGS.md`; if you find one, add it there *first*.
3. **Make focused edits.** One round = one theme. Do not refactor unrelated
   files "while you're there".
4. **Run the smoke test** (`scripts/smoke_test.sh`) after changes. If it
   fails, fix the regression before proceeding — do not mark the round done.
5. **Add smoke-test assertions** that would have caught the bug you just
   fixed. A round that promises a controller must fail smoke if that
   controller is absent.
6. **Update docs.** README and `crisp_plan.md` must reflect reality after
   each round that changes user-visible behavior.
7. **Log the round** in `ITERATION_LOG.md` (status `complete` only after
   smoke passes). Include: commit or "uncommitted", smoke result, targets,
   change bullets, follow-ups.
8. **Stop and wait for review** before starting the next round unless the
   user has explicitly authorized batch mode.

## Hard rules

- Never delete or rewrite past entries in `ITERATION_LOG.md`.
- Never mark a round `complete` with a failing or skipped smoke test.
- Never introduce a new top-level config knob without updating
  `config/config.template.yaml`.
- Never change `joint_states` shape, joint order, or topic names without
  explicit user approval — downstream CRISP / MoveIt rely on them.
- Never disable a smoke check to make a round pass; fix the code.

## Preferred conventions

- Align names and YAML keys with
  [`UniversalRobots/Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
  (`humble` branch) and
  [`UniversalRobots/Universal_Robots_ROS2_GZ_Simulation`](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)
  (`humble` branch) whenever we implement a feature they already have.
- `ur_type`-dependent constants go in `config/ur_types/<ur_type>.yaml` once
  Round 4 lands; until then, keep such constants in exactly one place and
  note the TODO.
- Shell scripts: `set -euo pipefail` on all new scripts, portable bash only.
- Python: match surrounding style; do not add type annotations or docstrings
  to code you are not otherwise modifying.

## When to ask the user

- Scope ambiguity between two rounds.
- A shortcoming's fix needs a protocol/topic change that breaks clients.
- Smoke test cannot be made green without relaxing an assertion.

Otherwise: keep moving, log what you did, stop at the end of the round.
