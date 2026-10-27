# SpaceAGORAKitDemo

A demonstration of the contributor kit of
[SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl): three small GNC
components, built on SpaceAGORA's public interface only, kept in a repository of
their own, and tested with SpaceAGORA's conformance checks and its reusable CI
workflow.

This repository is pinned to SpaceAGORA v0.7.0 (commit
`bec28fcc348b48c6e807d007f9a5009e253e8340`): that commit is in `SpaceAGORASource.toml`
and in the workflow call in `.github/workflows/ci.yml`, and `[compat]` admits the
0.6 and 0.7 series. SpaceAGORA v0.7.1 has been released since; to use it, or any
other core revision, follow the
[release-update steps](#moving-to-another-spaceagora-release) below.

Start with [setup and the example](#set-up-and-test), then follow the
[source reading path](#read-and-change-the-example). The [test guide](#the-tests)
explains what is checked, and the [release-update steps](#moving-to-another-spaceagora-release)
keep the local environment and CI on the same core revision.

GNC means guidance, navigation and control: here a planner proposes a burn, an
estimator tracks attitude, and a controller uses that estimate to command torque.
The example is for contributors learning how to keep their own methods in a
separate package.

## The three parts

| Part | File | What it shows |
|---|---|---|
| `PeriapsisRaisePlanner` | [`src/periapsis_raise.jl`](src/periapsis_raise.jl) | A planner of an existing algorithm family, SpaceAGORA's burn-plan family, run in a simulation through `AlgorithmGuidance`. SpaceAGORA's request driver (`run_algorithm_request!`) builds each request, the planner proposes a burn, and the family validates it and installs it in the burn scheduler. The repository contains no family code and no orchestration of its own; `algorithm_run_report` shows each step. |
| `AttitudeFilter` | [`src/attitude_filter.jl`](src/attitude_filter.jl) | An estimator. `initialize_navigation!` sets the estimate and its covariance from the filter's configuration at every run's start, and `update_navigation!` fuses the readings (`sensor_measurement`) of a `RateGyro` and a `StarTracker`. |
| `PDPointingController` | [`src/pd_pointing.jl`](src/pd_pointing.jl) | A feedback controller that ticks and holds: its control law runs at each tick, and the dynamics read only the held torque through the output hook `control_wrench`. It declares the spacecraft it acts on (`spacecraft_scope`), its period, its held command (`held_command`) and a release for an inactive spacecraft (`skip_inactive`, `release_command!`). It reads the filter's `AttitudeEstimate` through a declared link (`Consumes`, with the filter's `Produces`). |

`src/attitude.jl` holds the attitude helpers both use, in SpaceAGORA's quaternion
convention. `demo_configuration` (`src/demo_configuration.jl`) puts the three parts
in one run: a spacecraft near the apoapsis of an Earth orbit raises its periapsis
with one burn while it turns to a commanded attitude. The run needs no native
GRAM, SPICE kernels or private data.

## Who owns what

- This repository's author owns the methods (the planner's burn, the filter, the
  control law), their parameters, their scientific validation, and every claim made
  from them.
- SpaceAGORA owns scheduling (callbacks, rates, stop times), state ownership
  (run state in the model, deep copies), reproducible randomness (the sensors' and
  the algorithms' seeded streams), component connections (links, the burn-plan
  family, install targets), lifecycle handling (pre-run checks, initialization and
  the reset at each run's start) and the common testing machinery
  (`SpaceAGORA.Conformance` and the contributor workflow).

A conformance pass checks interface behavior on the supplied fixtures. It does
not establish that the methods are right for a mission.

## Set up and test

You need Julia 1.12 and git. Every command below runs from one folder, the one that
holds the two clones and the environment. In an empty folder, clone this repository
and SpaceAGORA side by side, check SpaceAGORA out at the commit this repository pins,
and build a separate environment that develops that commit:

```bash
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAKitDemo.jl.git
git clone --filter=blob:none -c lfs.fetchexclude="*" https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git
git -C SpaceAGORA.jl checkout "$(sed -n 's/^rev = "\(.*\)"/\1/p' SpaceAGORAKitDemo.jl/SpaceAGORASource.toml)"
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl package SpaceAGORAKitDemo.jl SpaceAGORAKitDemo-env --manifest core
```

`--filter=blob:none` skips the contents of old file versions: Git downloads the
files of a commit when you check it out. `-c lfs.fetchexclude="*"` keeps that clone,
its checkouts and later pulls from downloading SpaceAGORA's large data files (Git
LFS), which this repository does not use; without Git LFS installed the setting does
nothing, and nothing is downloaded either. The third line reads the pinned commit
from `SpaceAGORASource.toml` with `sed`, so it needs bash or zsh (on Windows, Git
Bash). In PowerShell or Command Prompt, name the commit instead: run
`git -C SpaceAGORA.jl checkout` followed by the 40-character `rev` that
`SpaceAGORAKitDemo.jl/SpaceAGORASource.toml` records. Every other command on this
page runs there as written. The last command downloads the packages in
SpaceAGORA's manifest that your Julia depot does not have yet; it refuses an
environment directory that already exists and is not empty.

Then run the tests the way CI does, with two threads so that the conformance
checks can compare concurrent runs with sequential ones. The first run also
compiles SpaceAGORA and its dependencies, which takes a while:

```bash
julia --threads=2 --project=SpaceAGORAKitDemo-env --depwarn=error SpaceAGORAKitDemo.jl/test/runtests.jl
```

`Pkg.test("SpaceAGORAKitDemo")` in that environment also runs the tests, but its
bounds-checking setting can trigger extra compilation. To run the demonstration
itself, use the same environment:

```bash
julia --project=SpaceAGORAKitDemo-env SpaceAGORAKitDemo.jl/examples/run_in_simulation.jl
```

The example prints the planner's request and installed-burn records, the planned
and used propellant, the initial and final periapsis altitudes, and the final
pointing and estimation errors. It keeps the solution in memory and writes no
result files or plots. These numbers describe this demonstration run, not a
validated mission.

## Read and change the example

Read [the executable example](examples/run_in_simulation.jl) first, then
[`demo_configuration`](src/demo_configuration.jl), which connects the three
components, sensors and burn scheduler. Its keyword arguments are the starting
point for changing the target attitude, periapsis target or sensor seeds.

Next, follow one component from its source file in the table above to
[`test/method_tests.jl`](test/method_tests.jl). The planner documents its two-body
and impulsive-burn assumptions and refusal cases. The filter and controller show
which fields are configuration, which fields change during a run, and which
state their checkpoint hooks preserve. Their shared `AttitudeEstimate` connects
one producer to its consumer; [the attitude helpers](src/attitude.jl) document
the quaternion convention used by both.

Before submitting a change, run the tests and the example above. Add method tests
for changed equations or refusal cases, and update the simulation checks if the
demonstration is meant to change. From the folder that holds both clones, two more
commands check the core pin and scan the package for SpaceAGORA's internal names:

```bash
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl check package SpaceAGORAKitDemo.jl
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl scan package SpaceAGORAKitDemo.jl
```

The scan only checks which names the package uses; the tests are still needed to
exercise what the components do.

## The tests

`test/runtests.jl` is the single entry point: it runs the conformance checks and
then includes the two other test files.

- The conformance checks are SpaceAGORA's own, applied to each part: the planner on
  the burn-plan family's representative requests and in a short simulation through
  `AlgorithmGuidance` (`test_algorithm`), the filter with its two sensors mounted
  (`test_navigation_model`), and the controller paired with a filter
  (`test_control_model`). For the filter and the controller they cover the pre-run
  hook check, repeatable and concurrent runs, deep copies, independence from the
  global random number generator, the reset at each run's start, separate state for
  each spacecraft a model is bound to, the purity of the controller's output hook,
  and a run resumed from a checkpoint against an uninterrupted one
  (`checkpoint_round_trip`, required since SpaceAGORA 0.5.0). The controller and the
  filter carry their run state through a checkpoint with `checkpoint_state` and
  `restore_checkpoint_state!`.
  The planner's checks are different: the family resolves, the result follows the
  family's contract, the request is not mutated, runs with the same random stream
  agree, refusals are explicit, and a request makes the round trip through the
  driver and a simulation run. The CI summary lists every check by name.
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

## Moving to another SpaceAGORA release

The pinned commit appears three times, and the workflow checks that they agree:
`rev` in `SpaceAGORASource.toml`, and the ref after `@` and `spaceagora-rev` in
`.github/workflows/ci.yml`. `version` in `SpaceAGORASource.toml` is SpaceAGORA's
version at that commit, and `[compat]` in `Project.toml` admits the supported
SpaceAGORA release series. SpaceAGORA's `repin` command updates these for the
release checked out beside this repository. It changes `[compat]` only when the
range does not admit the selected core version.

The setup command refuses a SpaceAGORA checkout that differs from the pin, so to
use another release you repin first. From the folder that holds both clones,
fetch SpaceAGORA's tags, check out the release and run `repin`. These commands
move the pin to v0.7.1; for another release, put its tag in the second line:

```bash
git -C SpaceAGORA.jl fetch --tags
git -C SpaceAGORA.jl checkout v0.7.1
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl repin package SpaceAGORAKitDemo.jl
```

If the checked-out release is the one already pinned, `repin` says so and changes
nothing. SpaceAGORA's release tags follow the `version` in its `Project.toml`; its
older `v1.0.0` and `v2.0.0` tags predate that numbering and are not newer releases.

`repin` changes the pin metadata, the workflow references and, if needed, the
compatible release series. It does not edit this README, which names the pinned
release at the top of the page. Review the changes, then create a new environment
and test the new pair. The environment path below must be new or empty:

```bash
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl package SpaceAGORAKitDemo.jl SpaceAGORAKitDemo-next-env --manifest core
julia --threads=2 --project=SpaceAGORAKitDemo-next-env --depwarn=error SpaceAGORAKitDemo.jl/test/runtests.jl
julia --project=SpaceAGORAKitDemo-next-env SpaceAGORAKitDemo.jl/examples/run_in_simulation.jl
```

An existing environment develops the sibling SpaceAGORA checkout by path, so
checking out another release also changes the source it loads. Use the new
environment after repinning. `repin` exists from SpaceAGORA 0.4.0 on.

SpaceAGORA's own CI also runs this repository: its contributor-kit demonstration
workflow repins a copy of it to the SpaceAGORA commit under test and runs these
tests, every night against SpaceAGORA's `main` and in each SpaceAGORA release pull
request.

## Repository origin

The repository was generated from SpaceAGORA's package starter
(`scripts/new_contributor_repository.jl package`) at SpaceAGORA 0.3.1 (commit
`c7db8b6fbb2843a3828ccc3003f68da06ca47848`); the first commit is the starter
exactly as generated. Every later change is in `src/`, `test/`, `examples/` or this
README, with three exceptions: the two metadata files the starter leaves for its
author (`LICENSE`, MIT as for SpaceAGORA, and the authors in `CITATION.cff`); the
SpaceAGORA pin, which moves with each release the repository is tested against
(see [Moving to another SpaceAGORA release](#moving-to-another-spaceagora-release));
and the `[compat]` entry for SpaceAGORA in `Project.toml`, widened by hand to
`"0.6, 0.7"` so that the repository admits both series. No SpaceAGORA code is
copied here, and the code uses only SpaceAGORA's public names (the workflow's scan
refuses internal ones).

## Claims

The numbers this repository's runs print are outputs of small demonstration
methods, not validated results.

## Contributors

- [Giusy Falcone](https://github.com/gfalcon2)
