# The demonstration run as a whole: the planner's path through SpaceAGORA's
# AlgorithmGuidance and the burn-plan family, as the run report shows it, and the
# run's reproducibility.
using Test, SpaceAGORA, SpaceAGORAKitDemo

estimate_of(configuration) = only(configuration.navigation_model.navigation_effectors).estimate

# One run of `configuration` with isolate_state=false, so that the configuration's own
# models hold what the run left: its saved times and states, the estimate and the
# algorithm run report, copied before the next run resets them.
function demo_run(configuration)
    solution = run_simulation(configuration; isolate_state=false, return_solution=true)
    return (t=copy(solution.t), u=deepcopy(solution.u), estimate=deepcopy(estimate_of(configuration)),
            report=algorithm_run_report(solution), propellant_used_kg=150.0 - solution.u[end].sc[1].mass)
end

same_estimate(a, b) = all(isequal(getfield(a, name), getfield(b, name)) for name in fieldnames(AttitudeEstimate))

@testset "the demonstration run" begin
    configuration = demo_configuration()
    first_run, second_run = demo_run(configuration), demo_run(configuration)

    @testset "planner path" begin
        # One request at the run's start, proposed by the planner, validated by the
        # family and installed: the binding's records show each step.
        binding = only(first_run.report.bindings)
        @test binding.algorithm == string(PeriapsisRaisePlanner)
        @test [record.event for record in binding.records] == [:request, :installed]
        installed = last(binding.records)
        plan = installed.product
        @test plan isa PropulsiveBurnPlan
        @test installed.validation.reason === :accepted
        @test 0 < plan.start_burn_s < plan.stop_burn_s < last(first_run.t)
        # The burn scheduler executed the installed plan: the propellant the run used
        # is the propellant the family validated.
        @test first_run.propellant_used_kg ≈ plan.propellant_required_kg rtol=1e-6
    end

    @testset "reproducibility" begin
        # Two runs of one configuration with isolate_state=false: every model resets
        # at the run's start, so the trajectories and the estimates are equal.
        @test isequal(first_run.t, second_run.t)
        @test isequal(first_run.u, second_run.u)
        @test same_estimate(first_run.estimate, second_run.estimate)
        @test isequal(first_run.report, second_run.report)
        # The same sensor seeds in a new configuration repeat the estimate; other
        # seeds change it.
        @test same_estimate(demo_run(demo_configuration(gyro_seed=1, star_tracker_seed=2)).estimate, first_run.estimate)
        other = demo_run(demo_configuration(gyro_seed=3, star_tracker_seed=4)).estimate
        @test !isequal(other.q, first_run.estimate.q)
        @test !isequal(other.bias, first_run.estimate.bias)
    end
end
