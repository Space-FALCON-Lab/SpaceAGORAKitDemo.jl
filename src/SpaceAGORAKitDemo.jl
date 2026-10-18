"""
    SpaceAGORAKitDemo

Three small GNC components built on SpaceAGORA's public interface, as a
demonstration of its contributor kit:

- `PeriapsisRaisePlanner`, a planner of SpaceAGORA's burn-plan family, run in a
  simulation through `AlgorithmGuidance`;
- `AttitudeFilter`, an estimator that fuses a `RateGyro` and a `StarTracker` into an
  `AttitudeEstimate`;
- `PDPointingController`, a proportional-derivative attitude controller that reads
  that estimate through a declared link and holds its torque between ticks.

`demo_configuration` puts the three in one run. The methods are small on purpose:
SpaceAGORA's conformance checks certify how they meet its interfaces, and the
package's own tests check the methods.
"""
module SpaceAGORAKitDemo

using LinearAlgebra: I, cross, diagm, dot, norm, normalize
using SpaceAGORA

export PeriapsisRaisePlanner, AttitudeEstimate, AttitudeFilter, PDPointingController, demo_configuration
export attitude_matrix, compose, rotation_quaternion, attitude_error

include("attitude.jl")
include("periapsis_raise.jl")
include("attitude_filter.jl")
include("pd_pointing.jl")
include("demo_configuration.jl")

end # module SpaceAGORAKitDemo
