# The demonstration run: the periapsis-raise planner through SpaceAGORA's
# AlgorithmGuidance, the attitude filter and the PD pointing controller in one
# simulation (see `demo_configuration`). It prints the planner's run records and what
# the run did. The workflow runs it after the tests. The numbers are this run's
# output, not validated results.
using SpaceAGORA, SpaceAGORAKitDemo
using LinearAlgebra: diag, norm

configuration = demo_configuration()
solution = run_simulation(configuration; isolate_state=false, return_solution=true)

# The planner's path: SpaceAGORA's request driver built the request, the planner
# proposed, the burn-plan family validated and installed the burn.
binding = only(algorithm_run_report(solution).bindings)
println("Planner: ", binding.algorithm)
for record in binding.records
    println("  t = ", record.time_s, " s: ", record.event, " (request ", record.request_id, ", ", record.reason, ")")
end
plan = only(filter(record -> record.event === :installed, binding.records)).product
println("Installed burn: Δv ", round(plan.delta_v_mps, digits=3), " m/s from ", round(plan.start_burn_s, digits=2),
        " s to ", round(plan.stop_burn_s, digits=2), " s; propellant planned ", round(plan.propellant_required_kg, digits=4),
        " kg, used ", round(150.0 - solution.u[end].sc[1].mass, digits=4), " kg")

planet = make_no_gram_planet(:earth)
periapsis_km(k) = (SpaceAGORAKitDemo.two_body_orbit(solution.u[k].sc[1].pos, solution.u[k].sc[1].vel, planet.μ).periapsis_m -
                   planet.Rp_e) / 1e3
println("Periapsis altitude (two-body): ", round(periapsis_km(1), digits=2), " km at the start, ",
        round(periapsis_km(length(solution.u)), digits=2), " km at the end")

# The attitude: the controller's pointing and the filter's estimate at the end.
estimate = only(configuration.navigation_model.navigation_effectors).estimate
controller = last(configuration.control_model.control_effectors)
truth = collect(solution.u[end].sc[1].q)
println("Pointing error at the end: ", round(norm(attitude_error(truth, controller.target_q)), sigdigits=3), " rad")
println("Estimate error at the end: ", round(norm(attitude_error(estimate.q, truth)), sigdigits=3), " rad (filter's ",
        "one-sigma ", round(sqrt(maximum(diag(estimate.covariance)[1:3])), sigdigits=3), " rad)")
