"""
    AttitudeEstimate()

The attitude estimate an `AttitudeFilter` writes and a `PDPointingController` reads:
one shared object, declared by both as a GNC link (`Produces` and `Consumes`), so
SpaceAGORA checks the pairing before a run and keeps it in the run's copies.

- `q`: the estimated attitude, inertial to body, scalar last;
- `ω_body`: the estimated body rate (rad/s), the latest gyro reading less the
  estimated bias;
- `bias`: the estimated gyro bias (rad/s);
- `covariance`: 6×6, of the attitude error (rad, body axes) and then the bias error;
- `time_s`: the time the estimate refers to (s into the run);
- `star_tracker_time_s`: the time of the last star-tracker reading fused, `-Inf`
  before the first.

The filter sets every field when each run starts (`initialize_navigation!`).
"""
mutable struct AttitudeEstimate
    q::Vector{Float64}
    ω_body::Vector{Float64}
    bias::Vector{Float64}
    covariance::Matrix{Float64}
    time_s::Float64
    star_tracker_time_s::Float64
end
AttitudeEstimate() = AttitudeEstimate([0.0, 0.0, 0.0, 1.0], zeros(3), zeros(3), zeros(6, 6), NaN, -Inf)

# One filter writes the estimate; any number of controllers read it.
SpaceAGORA.link_cardinality(::AttitudeEstimate) = (producers=1:1, consumers=0:typemax(Int))

"""
    AttitudeFilter(; spacecraft_index=1, gyro=1, star_tracker=2, period_s=1.0, initial_q=(0, 0, 0, 1),
                   initial_bias_rps=(0, 0, 0), initial_attitude_sigma_rad=0.01, initial_bias_sigma_rps=1e-3,
                   gyro_noise=1e-4, gyro_bias_walk=1e-6, star_tracker_sigma_rad=1e-4,
                   estimate=AttitudeEstimate())

A navigation model that estimates the attitude and the gyro bias of spacecraft
`spacecraft_index` from two of its mounted sensors, a `RateGyro` and a
`StarTracker`, found by their positions among the spacecraft's sensors (`gyro`,
`star_tracker`, in mounting order). It ticks every `period_s` seconds.

The method is a multiplicative extended Kalman filter with six states, the attitude
error and the gyro bias. Between readings it turns the estimate with the latest gyro
rate less the estimated bias, and grows the covariance by the gyro's white noise
`gyro_noise` (rad/√s) and bias random walk `gyro_bias_walk` (rad/s/√s); for each new
star-tracker reading, at that reading's time, it corrects the attitude and the bias
with the reading's noise `star_tracker_sigma_rad` (rad, per axis).

At each run's start the estimate is reset from the configuration: the attitude
`initial_q`, the bias `initial_bias_rps` (rad/s), and a diagonal covariance with
standard deviations `initial_attitude_sigma_rad` and `initial_bias_sigma_rps`.
"""
Base.@kwdef struct AttitudeFilter <: SpaceAGORA.AbstractNavigationModel
    spacecraft_index::Int = 1
    gyro::Int = 1
    star_tracker::Int = 2
    period_s::Float64 = 1.0
    initial_q::NTuple{4, Float64} = (0.0, 0.0, 0.0, 1.0)
    initial_bias_rps::NTuple{3, Float64} = (0.0, 0.0, 0.0)
    initial_attitude_sigma_rad::Float64 = 0.01
    initial_bias_sigma_rps::Float64 = 1e-3
    gyro_noise::Float64 = 1e-4
    gyro_bias_walk::Float64 = 1e-6
    star_tracker_sigma_rad::Float64 = 1e-4
    estimate::AttitudeEstimate = AttitudeEstimate()
end

# Declarations: the spacecraft it serves, the period it assumes, the estimate it writes.
SpaceAGORA.spacecraft_scope(f::AttitudeFilter) = BoundSpacecraft((f.spacecraft_index,))
SpaceAGORA.gnc_period_s(f::AttitudeFilter) = f.period_s
SpaceAGORA.gnc_links(f::AttitudeFilter) = (Produces(f.estimate),)

# At each run's start: the configured initial estimate, so that two runs of one
# configuration start equal.
function SpaceAGORA.initialize_navigation!(f::AttitudeFilter, u, p, t)
    e = f.estimate
    e.q = normalize(collect(f.initial_q))
    e.bias = collect(f.initial_bias_rps)
    e.ω_body = zeros(3)
    e.covariance = diagm([fill(f.initial_attitude_sigma_rad^2, 3); fill(f.initial_bias_sigma_rps^2, 3)])
    e.time_s = Float64(t)
    e.star_tracker_time_s = -Inf
    return nothing
end

# A checkpoint saves the estimate the filter writes (it owns the shared
# `AttitudeEstimate`) as plain data, and a run resumed from the checkpoint puts it
# back after the reset above, so the filter and its controllers go on from the
# estimate the uninterrupted run had.
function SpaceAGORA.checkpoint_state(f::AttitudeFilter)
    e = f.estimate
    return (q=copy(e.q), ω_body=copy(e.ω_body), bias=copy(e.bias), covariance=copy(e.covariance),
            time_s=e.time_s, star_tracker_time_s=e.star_tracker_time_s)
end

function SpaceAGORA.restore_checkpoint_state!(f::AttitudeFilter, data)
    e = f.estimate
    e.q = collect(Float64, data.q)
    e.ω_body = collect(Float64, data.ω_body)
    e.bias = collect(Float64, data.bias)
    e.covariance = Matrix{Float64}(data.covariance)
    e.time_s = Float64(data.time_s)
    e.star_tracker_time_s = Float64(data.star_tracker_time_s)
    return nothing
end

# The tick: fuse a star-tracker reading taken since the last tick at its own time,
# then bring the estimate to the tick's time. Sensors sample before navigation
# ticks at the same time.
function SpaceAGORA.update_navigation!(f::AttitudeFilter, u, p, t, spacecraft_index)
    spacecraft = p.args.dynamics_model.spacecraft[spacecraft_index]
    e = f.estimate
    rate = collect(Float64, sensor_measurement(spacecraft, f.gyro))
    all(isfinite, rate) || (rate = e.ω_body + e.bias)
    reading_time = sensor_measurement_time(spacecraft, f.star_tracker)
    if isfinite(reading_time) && reading_time > e.star_tracker_time_s
        reading = collect(Float64, sensor_measurement(spacecraft, f.star_tracker))
        if all(isfinite, reading)
            propagate!(f, rate, reading_time)
            correct!(f, reading)
        end
        e.star_tracker_time_s = reading_time
    end
    propagate!(f, rate, Float64(t))
    e.ω_body = rate - e.bias
    return nothing
end

symmetric(P) = (P + P') / 2

# Turn the estimate with the measured rate less the bias, up to time `t`.
function propagate!(f::AttitudeFilter, rate, t)
    e = f.estimate
    Δt = t - e.time_s
    Δt > 0 || return nothing
    ω = rate - e.bias
    e.q = propagate_attitude(e.q, ω, Δt)
    Z, I3 = zeros(3, 3), Matrix{Float64}(I, 3, 3)
    Φ = [I3 - skew(ω) * Δt -I3 * Δt; Z I3]
    σv², σu² = f.gyro_noise^2, f.gyro_bias_walk^2
    Q = [(σv² * Δt + σu² * Δt^3 / 3) * I3 (-σu² * Δt^2 / 2) * I3; (-σu² * Δt^2 / 2) * I3 (σu² * Δt) * I3]
    e.covariance = symmetric(Φ * e.covariance * Φ' + Q)
    e.time_s = t
    return nothing
end

# Correct the attitude and the bias with a star-tracker reading of the attitude.
function correct!(f::AttitudeFilter, reading)
    e = f.estimate
    P = e.covariance
    R = f.star_tracker_sigma_rad^2 * Matrix{Float64}(I, 3, 3)
    H = [Matrix{Float64}(I, 3, 3) zeros(3, 3)]
    K = (P * H') / (H * P * H' + R)
    correction = K * attitude_error(reading, e.q)
    e.q = normalize(compose(rotation_quaternion(correction[1:3]), e.q))
    e.bias = e.bias + correction[4:6]
    A = Matrix{Float64}(I, 6, 6) - K * H
    e.covariance = symmetric(A * P * A' + K * R * K')
    return nothing
end
