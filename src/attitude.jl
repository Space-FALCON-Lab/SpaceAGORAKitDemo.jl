# Attitude helpers shared by the estimator and the controller.
#
# A quaternion is four numbers (x, y, z, w), scalar last, describing the rotation
# from the inertial frame to the body frame, as SpaceAGORA's spacecraft states and
# its StarTracker report it: `attitude_matrix(q)` maps an inertial vector into body
# axes, and a body rate ω turns the attitude as d/dt attitude_matrix(q) =
# -[ω×] attitude_matrix(q).

# The cross-product matrix: skew(a) * b == cross(a, b).
skew(v) = [0.0 -v[3] v[2]; v[3] 0.0 -v[1]; -v[2] v[1] 0.0]

vector_part(q) = [Float64(q[1]), Float64(q[2]), Float64(q[3])]

"""
    attitude_matrix(q) -> Matrix{Float64}

The 3×3 matrix that maps inertial vectors into the body axes of attitude `q`.
"""
function attitude_matrix(q)
    v, w = vector_part(q), Float64(q[4])
    return (w^2 - dot(v, v)) * Matrix{Float64}(I, 3, 3) + 2 * v * v' - 2w * skew(v)
end

"""
    compose(p, q) -> Vector{Float64}

The attitude reached by rotation `q` followed by rotation `p`:
`attitude_matrix(compose(p, q)) ≈ attitude_matrix(p) * attitude_matrix(q)`.
"""
function compose(p, q)
    pv, qv = vector_part(p), vector_part(q)
    v = p[4] * qv + q[4] * pv - cross(pv, qv)
    return [v; p[4] * q[4] - dot(pv, qv)]
end

"""The opposite rotation of `q`."""
conjugate(q) = [-Float64(q[1]), -Float64(q[2]), -Float64(q[3]), Float64(q[4])]

"""
    rotation_quaternion(θ) -> Vector{Float64}

The rotation by the rotation vector `θ` (rad): an angle `norm(θ)` about `θ`. Applied
with `compose(rotation_quaternion(θ), q)`, it turns the body axes of `q` by `θ`.
"""
function rotation_quaternion(θ)
    angle = norm(θ)
    angle == 0 && return [0.0, 0.0, 0.0, 1.0]
    s = sin(angle / 2) / angle
    return [s * θ[1], s * θ[2], s * θ[3], cos(angle / 2)]
end

"""
    attitude_error(q, reference) -> Vector{Float64}

The rotation vector (rad, in body axes) that turns attitude `reference` into
attitude `q`, the shorter way round: `compose(rotation_quaternion(e), reference)`
is `q` up to sign.
"""
function attitude_error(q, reference)
    δ = compose(q, conjugate(reference))
    δ[4] < 0 && (δ = -δ)
    v = δ[1:3]
    s = norm(v)
    s == 0 && return zeros(3)
    return (2 * atan(s, δ[4]) / s) * v
end

"""The attitude `q` turned by the constant body rate `ω` (rad/s) for `Δt` seconds."""
propagate_attitude(q, ω, Δt) = normalize(compose(rotation_quaternion(ω * Δt), q))
