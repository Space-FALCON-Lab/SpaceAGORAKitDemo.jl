# The package's own tests of its methods: the attitude conventions, the planner
# against closed forms, and what the estimator and the controller do in the
# demonstration run. The conformance checks in runtests.jl do not test the methods;
# these tests do. They check the code against its own equations and SpaceAGORA's
# conventions, which is not a validation of the methods.
using Test, SpaceAGORA, SpaceAGORAKitDemo
using LinearAlgebra: I, diag, norm
using Random: MersenneTwister
using SpaceAGORA.Conformance: burn_plan_conformance_requests

@testset "the methods" begin
    @testset "attitude helpers" begin
        rng = MersenneTwister(3)
        random_rotation(angle) = (axis = randn(rng, 3); rotation_quaternion(angle * axis / norm(axis)))
        for _ in 1:20
            p, q = random_rotation(π * rand(rng)), random_rotation(π * rand(rng))
            @test attitude_matrix(compose(p, q)) ≈ attitude_matrix(p) * attitude_matrix(q)
            @test attitude_matrix(q) * attitude_matrix(q)' ≈ I
            axis = randn(rng, 3)
            θ = 0.9π * rand(rng) * axis / norm(axis)
            @test attitude_error(compose(rotation_quaternion(θ), q), q) ≈ θ
        end
        # A small turn θ of the body axes changes the attitude matrix by -[θ×], to first order.
        θ = [1e-6, -2e-6, 3e-6]
        @test attitude_matrix(rotation_quaternion(θ)) ≈ I - SpaceAGORAKitDemo.skew(θ) atol=1e-10
    end

    @testset "periapsis-raise planner" begin
        planner = PeriapsisRaisePlanner(target_periapsis_altitude_m=450e3)
        request = first(burn_plan_conformance_requests())      # at the periapsis of its orbit
        result = propose!(nothing, planner, request, MersenneTwister(1))
        @test result.status === :candidate
        plan = result.product
        μ, rp, ra = request.mu_m3ps2, result.diagnostics.periapsis_m, result.diagnostics.apoapsis_m
        target_m = request.planet_radius_m + 450e3
        # The planned Δv, added at apoapsis, puts the periapsis on the target (vis-viva).
        speed_after = sqrt(2μ * rp / (ra * (ra + rp))) + plan.delta_v_mps
        @test 2 / (2 / ra - speed_after^2 / μ) - ra ≈ target_m rtol=1e-12
        # From periapsis, the next apoapsis is half an orbit away; the window is centred on it.
        period_s = 2π * sqrt(((ra + rp) / 2)^3 / μ)
        @test (plan.start_burn_s + plan.stop_burn_s) / 2 ≈ request.time_s + period_s / 2 rtol=1e-9
        @test plan.stop_burn_s - plan.start_burn_s ≈ burn_plan_sizing(request.mass_kg, plan.delta_v_mps, 0.0,
            request.thrust_n, request.isp_s).burn_duration_s rtol=1e-12
        # The cases it refuses.
        refused(planner, request) = propose!(nothing, planner, request, MersenneTwister(1)).termination
        @test refused(PeriapsisRaisePlanner(target_periapsis_altitude_m=300e3), request) === :periapsis_at_or_above_target
        @test refused(PeriapsisRaisePlanner(target_periapsis_altitude_m=5000e3), request) === :target_above_apoapsis
        short = BurnPlanRequest(request_id=1, reason=:initial, spacecraft_index=1, time_s=0.0,
            position_m=request.position_m, velocity_mps=request.velocity_mps, mass_kg=request.mass_kg,
            available_propellant_kg=0.01, thrust_n=request.thrust_n, isp_s=request.isp_s, mu_m3ps2=μ,
            planet_radius_m=request.planet_radius_m, entry_interface_altitude_m=request.entry_interface_altitude_m)
        @test refused(planner, short) === :insufficient_propellant
        escaping = BurnPlanRequest(request_id=1, reason=:initial, spacecraft_index=1, time_s=0.0,
            position_m=request.position_m, velocity_mps=(0.0, 12000.0, 0.0), mass_kg=request.mass_kg,
            thrust_n=request.thrust_n, isp_s=request.isp_s, mu_m3ps2=μ, planet_radius_m=request.planet_radius_m,
            entry_interface_altitude_m=request.entry_interface_altitude_m)
        @test refused(planner, escaping) === :not_elliptic
    end

    @testset "estimator and controller in the demonstration run" begin
        configuration = demo_configuration()
        solution = run_simulation(configuration; isolate_state=false, return_solution=true)
        truth = collect(solution.u[end].sc[1].q)
        estimate = only(configuration.navigation_model.navigation_effectors).estimate
        controller = last(configuration.control_model.control_effectors)
        # The estimate's attitude error is within five standard deviations of its own
        # covariance; a mismatch with SpaceAGORA's attitude convention would put it near
        # the size of the slew (0.28 rad).
        @test norm(attitude_error(estimate.q, truth)) < 5 * sqrt(maximum(diag(estimate.covariance)[1:3]))
        # The controller has turned the spacecraft from its initial attitude to the target.
        @test norm(attitude_error(collect(solution.u[1].sc[1].q), controller.target_q)) > 0.2
        @test norm(attitude_error(truth, controller.target_q)) < 0.01
        # The burn raised the periapsis toward the target (a finite burn, so not onto it).
        planet = make_no_gram_planet(:earth)
        orbit(k) = SpaceAGORAKitDemo.two_body_orbit(solution.u[k].sc[1].pos, solution.u[k].sc[1].vel, planet.μ)
        @test orbit(1).periapsis_m - planet.Rp_e < 401e3
        @test abs(orbit(length(solution.u)).periapsis_m - planet.Rp_e - 450e3) < 1e3
    end
end
