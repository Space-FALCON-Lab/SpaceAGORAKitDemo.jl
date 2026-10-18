"""
    PeriapsisRaisePlanner(; target_periapsis_altitude_m)

A planner of SpaceAGORA's burn-plan family: one prograde burn, centred on the next
apoapsis, that raises the periapsis to `target_periapsis_altitude_m` (m above the
planet's equatorial radius).

The method, from the request's position and velocity: the two-body orbit; the
impulsive Δv at apoapsis that puts the periapsis at the target (vis-viva); the
burn's duration at the request's thrust and specific impulse, from the family's own
size check (`burn_plan_sizing`); and a window of that duration centred on the time
of the next apoapsis (Kepler's equation), taken one orbit later when the next
apoapsis is less than half a burn away. A finite burn is not impulsive, so the
periapsis it reaches is near the target, not on it.

It refuses (`:infeasible`) a request whose orbit is not elliptic (`:not_elliptic`)
or is nearly circular (`:nearly_circular`), whose periapsis is already at or above
the target (`:periapsis_at_or_above_target`), whose target is above the apoapsis
(`:target_above_apoapsis`), and a burn the size check refuses (its reason, such as
`:insufficient_propellant`). It draws no random numbers.
"""
struct PeriapsisRaisePlanner <: AbstractBurnPlanner
    target_periapsis_altitude_m::Float64
    function PeriapsisRaisePlanner(; target_periapsis_altitude_m::Real)
        (isfinite(target_periapsis_altitude_m) && target_periapsis_altitude_m > 0) ||
            throw(ArgumentError("target_periapsis_altitude_m must be finite and positive."))
        return new(Float64(target_periapsis_altitude_m))
    end
end

# Deterministic: the conformance checks then require the random stream to stay untouched.
SpaceAGORA.consumes_rng(::PeriapsisRaisePlanner) = false

"""
    two_body_orbit(position_m, velocity_mps, μ) -> NamedTuple or Symbol

The two-body ellipse through `position_m` with `velocity_mps` about a body of
gravitational parameter `μ` (m³/s²): its eccentricity, periapsis and apoapsis radii
(m), period (s) and the time until the next apoapsis (s). `:not_elliptic` for an
orbit that is not an ellipse, and `:nearly_circular` for one whose apsides are not
defined (eccentricity below 1e-9).
"""
function two_body_orbit(position_m, velocity_mps, μ)
    r, v = collect(Float64, position_m), collect(Float64, velocity_mps)
    radius = norm(r)
    energy = dot(v, v) / 2 - μ / radius
    energy < 0 || return :not_elliptic
    a = -μ / (2energy)
    e = norm(cross(v, cross(r, v)) / μ - r / radius)
    e < 1 || return :not_elliptic
    e < 1e-9 && return :nearly_circular
    # The eccentric anomaly from the radius and the radial velocity.
    E = atan(dot(r, v) / (e * sqrt(μ * a)), (1 - radius / a) / e)
    M = mod(E - e * sin(E), 2π)
    n = sqrt(μ / a^3)
    return (; eccentricity=e, periapsis_m=a * (1 - e), apoapsis_m=a * (1 + e), period_s=2π / n,
            time_to_apoapsis_s=mod(π - M, 2π) / n)
end

# The speed at apoapsis `ra` of the ellipse with periapsis `rp` (vis-viva).
apoapsis_speed(ra, rp, μ) = sqrt(2μ * rp / (ra * (ra + rp)))

function SpaceAGORA.propose!(state, planner::PeriapsisRaisePlanner, request::BurnPlanRequest, rng)
    refuse(why) = AlgorithmResult(request_id=request.request_id, status=:infeasible, termination=why)
    μ = request.mu_m3ps2
    orbit = two_body_orbit(request.position_m, request.velocity_mps, μ)
    orbit isa Symbol && return refuse(orbit)
    target_m = request.planet_radius_m + planner.target_periapsis_altitude_m
    target_m > orbit.periapsis_m || return refuse(:periapsis_at_or_above_target)
    target_m < orbit.apoapsis_m || return refuse(:target_above_apoapsis)
    delta_v_mps = apoapsis_speed(orbit.apoapsis_m, target_m, μ) - apoapsis_speed(orbit.apoapsis_m, orbit.periapsis_m, μ)
    # The family's own size check: the propellant, impulse and duration it validates.
    sizing = burn_plan_sizing(request.mass_kg, delta_v_mps, 0.0, request.thrust_n, request.isp_s;
                              available_propellant_kg=request.available_propellant_kg)
    sizing.reason === :accepted || return refuse(sizing.reason)
    half_burn_s = sizing.burn_duration_s / 2
    to_apoapsis_s = orbit.time_to_apoapsis_s
    to_apoapsis_s < half_burn_s && (to_apoapsis_s += orbit.period_s)
    start_s = request.time_s + to_apoapsis_s - half_burn_s
    plan = PropulsiveBurnPlan(valid=true, delta_v_mps=delta_v_mps, direction_rad=0.0, source_orbit=-1,
        start_burn_s=start_s, stop_burn_s=start_s + sizing.burn_duration_s,
        thrust_n=request.thrust_n, isp_s=request.isp_s,
        commanded_impulse_n_s=sizing.commanded_impulse_n_s, propellant_required_kg=sizing.propellant_required_kg)
    return AlgorithmResult(request_id=request.request_id, status=:candidate, termination=:planned, product=plan,
        diagnostics=(periapsis_m=orbit.periapsis_m, apoapsis_m=orbit.apoapsis_m, target_periapsis_m=target_m,
                     time_to_apoapsis_s=to_apoapsis_s))
end
