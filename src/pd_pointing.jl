"""
    PDPointingController(; estimate, spacecraft_index=1, target_q=(0, 0, 0, 1), kp=0.5, kd=4.0,
                         max_torque_nm=0.1, period_s=1.0)

A proportional-derivative attitude controller for spacecraft `spacecraft_index`. At
each tick, every `period_s` seconds, it reads the `AttitudeEstimate` `estimate`
(through a declared link, written by an `AttitudeFilter`) and computes the body
torque

    τ = -kp * attitude_error(q̂, target_q) - kd * ω̂

(N·m, with `kp` in N·m/rad and `kd` in N·m·s/rad), limited to `max_torque_nm` on each
body axis. It holds that torque until the next tick: the dynamics read the held
torque (`control_wrench`), never the control law. `target_q` is the commanded
attitude, inertial to body, scalar last. For an inactive spacecraft the controller
releases its command (zero torque) instead of ticking.
"""
Base.@kwdef mutable struct PDPointingController <: SpaceAGORA.AbstractControlEffectorModel
    estimate::AttitudeEstimate
    spacecraft_index::Int = 1
    target_q::NTuple{4, Float64} = (0.0, 0.0, 0.0, 1.0)
    kp::Float64 = 0.5
    kd::Float64 = 4.0
    max_torque_nm::Float64 = 0.1
    period_s::Float64 = 1.0
    # Run state: the held torque (N·m, body axes) and the time of the tick that set it.
    torque_body::NTuple{3, Float64} = (0.0, 0.0, 0.0)
    tick_s::Float64 = NaN
end

const ZERO3 = (0.0, 0.0, 0.0)

# Declarations: the spacecraft it acts on, the period its gains assume, the estimate it
# reads, and that an inactive spacecraft gets a release instead of a tick.
SpaceAGORA.spacecraft_scope(c::PDPointingController) = BoundSpacecraft((c.spacecraft_index,))
SpaceAGORA.gnc_period_s(c::PDPointingController) = c.period_s
SpaceAGORA.gnc_links(c::PDPointingController) = (Consumes(c.estimate),)
SpaceAGORA.skip_inactive(::PDPointingController) = true

# The held command and the tick that produced it.
SpaceAGORA.held_command(c::PDPointingController, spacecraft_index) = c.torque_body
SpaceAGORA.held_command_time(c::PDPointingController, spacecraft_index) = c.tick_s
function SpaceAGORA.release_command!(c::PDPointingController, t, spacecraft_index)
    c.torque_body, c.tick_s = ZERO3, Float64(t)
    return nothing
end

# At each run's start: no command held yet.
function SpaceAGORA.initialize_control!(c::PDPointingController, u, p, t)
    c.torque_body, c.tick_s = ZERO3, NaN
    return nothing
end

# A checkpoint saves the held command and its tick time as plain data, and a run
# resumed from the checkpoint puts them back, so it holds the same torque until its
# next tick as the uninterrupted run does.
SpaceAGORA.checkpoint_state(c::PDPointingController) = (torque_body=c.torque_body, tick_s=c.tick_s)
function SpaceAGORA.restore_checkpoint_state!(c::PDPointingController, data)
    c.torque_body = NTuple{3, Float64}(data.torque_body)
    c.tick_s = Float64(data.tick_s)
    return nothing
end

# The tick: the control law on the estimate, limited, then held.
function SpaceAGORA.update_control!(c::PDPointingController, u, p, t, spacecraft_index)
    e = c.estimate
    τ = -c.kp * attitude_error(e.q, c.target_q) - c.kd * e.ω_body
    limited = clamp.(τ, -c.max_torque_nm, c.max_torque_nm)
    c.torque_body = (limited[1], limited[2], limited[3])
    c.tick_s = Float64(t)
    return nothing
end

# The output, at every evaluation of the equations of motion: the held torque and no
# force. The dynamics call it for every spacecraft.
SpaceAGORA.control_wrench(c::PDPointingController, x::StateSample, p, t) =
    x.spacecraft_index == c.spacecraft_index ? (ZERO3, c.torque_body) : (ZERO3, ZERO3)
