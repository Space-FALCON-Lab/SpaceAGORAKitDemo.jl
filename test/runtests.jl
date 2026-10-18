using Test, SpaceAGORA, SpaceAGORAKitDemo
using SpaceAGORA.Conformance: conformance_fixture, test_algorithm, test_control_model, test_navigation_model,
    burn_plan_conformance_requests, burn_plan_conformance_simulation

# The sensors the estimator reads on the conformance fixtures, in its default order
# (gyro first, then star tracker), noisy so that the checks compare seeded streams.
fixture_sensors() = (RateGyro(period_s=1.0, angle_random_walk=1e-4, rate_random_walk=1e-6, seed=7),
                     StarTracker(period_s=5.0, cross_boresight_noise_rad=1e-4, about_boresight_noise_rad=1e-4, seed=8))

@testset "SpaceAGORAKitDemo" begin
    # Interface conformance of each part, with SpaceAGORA's own checks. A pass
    # certifies the interfaces, not the methods.
    @testset "conformance" begin
        # The planner: the burn-plan family's representative requests, and a short
        # simulation through AlgorithmGuidance.
        test_algorithm(PeriapsisRaisePlanner(target_periapsis_altitude_m=450e3), burn_plan_conformance_requests();
                       simulation=burn_plan_conformance_simulation())

        # The estimator, with its two sensors mounted, at the period it declares.
        estimator = AttitudeFilter()
        test_navigation_model(estimator; fixture=conformance_fixture(navigation=(estimator,),
                              sensors=fixture_sensors(), navigation_rate_s=1.0))

        # The controller reads the estimator's estimate: check it on a fixture that
        # holds the pair, so the link has its producer.
        estimate = AttitudeEstimate()
        controller = PDPointingController(estimate=estimate, target_q=Tuple(rotation_quaternion([0.1, 0.0, 0.0])))
        fixture = conformance_fixture(navigation=(AttitudeFilter(estimate=estimate),), control=(controller,),
                                      sensors=fixture_sensors(), navigation_rate_s=1.0, control_rate_s=1.0)
        test_control_model(controller; fixture=fixture)
    end

    include("method_tests.jl")
    include("simulation_tests.jl")
end
