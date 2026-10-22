# SpaceAGORAKitDemo

A demonstration of the contributor kit of
[SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl): three small GNC
components, built on SpaceAGORA's public interface only, kept in a repository of
their own, and tested with SpaceAGORA's conformance checks and its reusable CI
workflow.

This version is developed and tested against **SpaceAGORA v0.5.0** (commit
`41cd1f2ef7bcd4ef51f556487911e797ca411867`): the commit in `SpaceAGORASource.toml`
and in the workflow call in `.github/workflows/ci.yml`, with `[compat]` admitting
the 0.5 release series.

Start with [setup and the example](#set-up-and-test), then follow the
[source reading path](#read-and-change-the-example). The [test guide](#the-tests)
explains what is checked, and the [release-update steps](#moving-to-another-spaceagora-release)
keep the local environment and CI on the same core revision.

GNC means guidance, navigation and control: here a planner proposes a burn, an
estimator tracks attitude, and a controller uses that estimate to command torque.
The example is intended for contributors learning how to put their own methods
in a separate package.

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

- **This repository's author owns** the methods (the planner's burn, the filter, the
  control law), their parameters, their scientific validation, and every claim made
  from them.
- **SpaceAGORA owns** scheduling (callbacks, rates, stop times), state ownership
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
and SpaceAGORA side by side, check SpaceAGORA out at the commit this repository pins
(the v0.5.0 release), and build a separate environment that develops that commit:

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
page runs there as written. The last command downloads
the packages SpaceAGORA's manifest records that your Julia depot does not hold yet, and refuses an environment directory that
already exists and is not empty. Then run the tests as CI does, with two threads so
that the conformance checks compare concurrent runs with sequential ones (the first
run also compiles SpaceAGORA and its dependencies, which takes a while):

```bash
julia --threads=2 --project=SpaceAGORAKitDemo-env --depwarn=error SpaceAGORAKitDemo.jl/test/runtests.jl
```

`Pkg.test("SpaceAGORAKitDemo")` in that environment is another test entry point.
Its bounds-checking settings can require additional compilation. To run the
demonstration itself, use the same environment:

```bash
julia --project=SpaceAGORAKitDemo-env SpaceAGORAKitDemo.jl/examples/run_in_simulation.jl
```

The example prints the planner's request and installed-burn records, the planned
and used propellant, initial and final periapsis altitudes, and final pointing
and estimation errors. It keeps the solution in memory and disables result-file
and plot generation. These outputs describe this demonstration run; they are
not reference values for a validated mission.

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

Before submitting a change, run the tests and example above. Add method tests
for changed equations or refusal cases, and update the simulation checks if the
intended demonstration changes. From the folder containing both clones, these
additional checks verify the core pin and scan the package for internal names:

```bash
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl check package SpaceAGORAKitDemo.jl
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl scan package SpaceAGORAKitDemo.jl
```

The scan is one check on interface use; the tests remain necessary to exercise
the behavior of the components.

## The tests

- `test/runtests.jl` runs SpaceAGORA's conformance checks on each part: the planner on
  the burn-plan family's representative requests and in a short simulation through
  `AlgorithmGuidance` (`test_algorithm`), the filter with its two sensors mounted
  (`test_navigation_model`), and the controller paired with a filter
  (`test_control_model`). They cover the pre-run hook check, repeatable and
  concurrent runs, deep copies, independence from the global random number
  generator, the reset at each run's start, the purity of the output hook, and a
  run resumed from a checkpoint against an uninterrupted one
  (`checkpoint_round_trip`, required from SpaceAGORA 0.5.0). The controller and the
  filter carry their run state through a checkpoint with `checkpoint_state` and
  `restore_checkpoint_state!`.
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
version at that commit, and `[compat]` in `Project.toml` admits one SpaceAGORA
release series. SpaceAGORA's `repin` command rewrites all of them for the release
checked out beside this repository, and changes `[compat]` when the series
changes. From the folder that holds both clones, fetch SpaceAGORA's tags, check
out the release you move to and run `repin`. For the move to SpaceAGORA v0.6.0:

```bash
git -C SpaceAGORA.jl fetch --tags
git -C SpaceAGORA.jl checkout v0.6.0
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl repin package SpaceAGORAKitDemo.jl
```

For a later release, put its tag in the second line. SpaceAGORA's release tags
follow the `version` in its `Project.toml`; its older `v1.0.0` and `v2.0.0` tags
predate that numbering and are not newer releases.

`repin` changes the package's pin metadata, workflow references and compatible
release series. Review those changes, then create a new environment and test the
new pair. The environment path below must be new or empty:

```bash
julia --startup-file=no --project=@stdlib SpaceAGORA.jl/scripts/contributor_environment.jl package SpaceAGORAKitDemo.jl SpaceAGORAKitDemo-next-env --manifest core
julia --threads=2 --project=SpaceAGORAKitDemo-next-env --depwarn=error SpaceAGORAKitDemo.jl/test/runtests.jl
julia --project=SpaceAGORAKitDemo-next-env SpaceAGORAKitDemo.jl/examples/run_in_simulation.jl
```

An existing environment develops the sibling SpaceAGORA checkout by path, so
changing that checkout also changes the source it loads. Use the new environment
after repinning. The repository moved from SpaceAGORA 0.3.1 to its later pins
this way; `repin` exists from SpaceAGORA 0.4.0 on.

## Repository origin

The repository was generated from SpaceAGORA's package starter
(`scripts/new_contributor_repository.jl package`) at SpaceAGORA 0.3.1 (commit
`c7db8b6fbb2843a3828ccc3003f68da06ca47848`); the first commit is the starter
exactly as generated. Every later change is in `src/`, `test/`, `examples/` or this
README, except for two metadata files the starter leaves for its author (`LICENSE`,
MIT as for SpaceAGORA, and the authors in `CITATION.cff`) and the SpaceAGORA pin,
which moves with each release it is tested against (see [Moving to another
SpaceAGORA release](#moving-to-another-spaceagora-release)). No SpaceAGORA code is copied here, and the code uses
only SpaceAGORA's public names (the workflow's scan refuses internal ones).

## Claims

The numbers this repository's runs print are outputs of small demonstration
methods, not validated results.
