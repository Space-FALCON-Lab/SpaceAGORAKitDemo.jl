# SpaceAGORAKitDemo

A demonstration of the contributor kit of
[SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl): three small GNC
components, built on SpaceAGORA's public interface only, kept in a repository of
their own, and tested with SpaceAGORA's conformance checks and its reusable CI
workflow.

The repository was generated from SpaceAGORA's package starter
(`scripts/new_contributor_repository.jl package`) at the SpaceAGORA commit in
`SpaceAGORASource.toml`; the first commit is the starter exactly as generated.
Every later change is in `src/`, `test/`, `examples/` or this README, except for
two metadata files the starter leaves for its author: `LICENSE` (MIT, as for
SpaceAGORA) and the authors in `CITATION.cff`. No SpaceAGORA code is copied here,
and the code uses only SpaceAGORA's public names (the workflow's scan refuses
internal ones).

## The three parts

| Part | File | What it shows |
|---|---|---|
| `PeriapsisRaisePlanner` | `src/periapsis_raise.jl` | A planner of an existing algorithm family, SpaceAGORA's burn-plan family, run in a simulation through `AlgorithmGuidance`. SpaceAGORA's request driver (`run_algorithm_request!`) builds each request, the planner proposes a burn, and the family validates it and installs it in the burn scheduler. The repository contains no family code and no orchestration of its own; `algorithm_run_report` shows each step. |
| `AttitudeFilter` | `src/attitude_filter.jl` | An estimator. `initialize_navigation!` sets the estimate and its covariance from the filter's configuration at every run's start, and `update_navigation!` fuses the readings (`sensor_measurement`) of a `RateGyro` and a `StarTracker`. |
| `PDPointingController` | `src/pd_pointing.jl` | A feedback controller that ticks and holds: its control law runs at each tick, and the dynamics read only the held torque through the output hook `control_wrench`. It declares the spacecraft it acts on (`spacecraft_scope`), its period, its held command (`held_command`) and a release for an inactive spacecraft (`skip_inactive`, `release_command!`). It reads the filter's `AttitudeEstimate` through a declared link (`Consumes`, with the filter's `Produces`). |

`src/attitude.jl` holds the attitude helpers both use, in SpaceAGORA's quaternion
convention. `demo_configuration` (`src/demo_configuration.jl`) puts the three parts
in one run: a spacecraft near the apoapsis of an Earth orbit raises its periapsis
with one burn while it turns to a commanded attitude. The run needs no native
GRAM, SPICE kernels or private data.

## Who owns what

- **This repository's author owns** the methods (the planner's burn, the filter, the
  control law), their parameters, their scientific validation, and every claim made
  from them.
- **SpaceAGORA owns** scheduling (callbacks, rates, stop times), state ownership
  (run state in the model, deep copies), reproducible randomness (the sensors' and
  the algorithms' seeded streams), component connections (links, the burn-plan
  family, install targets), lifecycle handling (pre-run checks, initialization and
  the reset at each run's start) and the common testing machinery
  (`SpaceAGORA.Conformance` and the contributor workflow).

A conformance pass certifies how the components meet SpaceAGORA's interfaces. It
says nothing about whether the methods are right for a mission.

## Set up and test

You need Julia 1.12 and git. In an empty folder, clone this repository and
SpaceAGORA side by side, check SpaceAGORA out at the commit this repository pins,
and build a separate environment that develops that commit:

```bash
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAKitDemo.jl.git
git clone -c lfs.fetchexclude='*' https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git
git -C SpaceAGORA.jl checkout "$(sed -n 's/^rev = "\(.*\)"/\1/p' SpaceAGORAKitDemo.jl/SpaceAGORASource.toml)"
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl package SpaceAGORAKitDemo.jl SpaceAGORAKitDemo-env --manifest core
```

`-c lfs.fetchexclude='*'` keeps that clone, its checkouts and later pulls from
downloading SpaceAGORA's large data files (Git LFS), which this repository does not
use; without Git LFS installed the setting does nothing, and nothing is downloaded
either. The last command downloads the packages SpaceAGORA's manifest records that
your Julia depot does not hold yet. Then run the tests as CI does, with two threads
so that the conformance checks compare concurrent runs with sequential ones:

```bash
cd SpaceAGORAKitDemo.jl/test
JULIA_NUM_THREADS=2 julia --project=../../SpaceAGORAKitDemo-env --depwarn=error runtests.jl
```

`Pkg.test("SpaceAGORAKitDemo")` in that environment works too, but it runs with
bounds checking, which recompiles every package. The example prints the planner's
run records and what the run did:

```bash
cd SpaceAGORAKitDemo.jl
julia --project=../SpaceAGORAKitDemo-env examples/run_in_simulation.jl
```

## The tests

- `test/runtests.jl` runs SpaceAGORA's conformance checks on each part: the planner on
  the burn-plan family's representative requests and in a short simulation through
  `AlgorithmGuidance` (`test_algorithm`), the filter with its two sensors mounted
  (`test_navigation_model`), and the controller paired with a filter
  (`test_control_model`). They cover the pre-run hook check, repeatable and
  concurrent runs, deep copies, independence from the global random number
  generator, the reset at each run's start and the purity of the output hook.
- `test/method_tests.jl` holds the author's tests of the methods: the attitude
  helpers against SpaceAGORA's convention, the planner against the closed forms it
  uses and the cases it must refuse, and what the filter and the controller do in the
  demonstration run. These check the code against its own equations; they are not a
  validation of the methods.
- `test/simulation_tests.jl` checks the demonstration run itself: the planner's
  request and installed burn in the run report, the propellant the run used against
  the propellant the family validated, two runs of one configuration with
  `isolate_state=false` giving equal trajectories and estimates, and the sensor seeds
  repeating or changing the estimate.

## Moving to another SpaceAGORA commit

The pinned commit appears three times, and the workflow checks that they agree:
`rev` in `SpaceAGORASource.toml`, and the ref after `@` and `spaceagora-rev` in
`.github/workflows/ci.yml`. `version` in `SpaceAGORASource.toml` is SpaceAGORA's
version at that commit, and `[compat]` in `Project.toml` admits one SpaceAGORA
release series. To move to a release, replace the commit in both files, set
`version` to the release, and change the `SpaceAGORA` entry in `[compat]` when the
series changes (for example from `"0.3"` to `"0.4"`).

## Claims

The numbers this repository's runs print are outputs of small demonstration
methods, not validated results.
