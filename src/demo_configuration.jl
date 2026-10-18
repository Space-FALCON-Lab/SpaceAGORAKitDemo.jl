"""
    demo_configuration(; mission_time_s=400.0, target_periapsis_altitude_m=450e3,
                       target_q=rotation_quaternion([0.2, 0.2, 0.0]), gyro_seed=1, star_tracker_seed=2,
                       gyro_noise=1e-4, gyro_bias_walk=1e-6, gyro_bias_rps=(2e-4, -1e-4, 5e-5),
                       star_tracker_sigma_rad=1e-4) -> SimulationConfiguration

The demonstration's run, with the three components together. One 150 kg spacecraft
(100 kg dry, 50 kg of propellant, a 1 m cube) starts 10° of true anomaly before the
apoapsis of a 400 km by 2000 km Earth orbit at 30° inclination, at rest in the
inertial frame, with point-mass gravity and no atmosphere, for `mission_time_s`
seconds with attitude propagation. It needs no native GRAM, SPICE kernels or
private data.

- Guidance: an `AlgorithmGuidance` runs a `PeriapsisRaisePlanner` with
  `target_periapsis_altitude_m`, with one request (reason `:initial`) at the run's
  start. SpaceAGORA's burn-plan family validates the proposed burn and installs it
  in the burn scheduler, a `BaseThrusterModel` (50 N, 300 s specific impulse) that
  plans no burn of its own.
- Navigation: an `AttitudeFilter`, ticking every second, reads a `RateGyro`
  (sampled every second, with white noise `gyro_noise`, a bias that starts at
  `gyro_bias_rps` and walks with `gyro_bias_walk`, seed `gyro_seed`) and a
  `StarTracker` (every 5 s, noise `star_tracker_sigma_rad` on each axis, seed
  `star_tracker_seed`), both on the spacecraft's body. Its initial estimate is the
  true initial attitude with no bias.
- Control: a `PDPointingController`, ticking every second, turns the spacecraft to
  `target_q` using the filter's estimate.

The filter and the controller share one `AttitudeEstimate`; read it from
`only(configuration.navigation_model.navigation_effectors).estimate` after a run
with `isolate_state=false`.
"""
function demo_configuration(; mission_time_s::Real=400.0, target_periapsis_altitude_m::Real=450e3,
        target_q=rotation_quaternion([0.2, 0.2, 0.0]), gyro_seed::Integer=1, star_tracker_seed::Integer=2,
        gyro_noise::Real=1e-4, gyro_bias_walk::Real=1e-6, gyro_bias_rps=(2e-4, -1e-4, 5e-5),
        star_tracker_sigma_rad::Real=1e-4)
    planet = make_no_gram_planet(:earth)
    root = Link(root=true, m=100.0, ref_area=2.0, dims=[1.0, 1.0, 1.0])
    orbit = InitialCondition(ra=planet.Rp_e + 2000e3, rp=planet.Rp_e + 400e3, i=30.0, ω=0.0, Ω=0.0, ν=170.0)
    spacecraft = SpacecraftModel(joints=Joint[], links=[root], root=root, prop_mass=50.0,
                                 inertia_tensor=root.inertia, initial_condition=orbit, id=1)
    add_sensor!(spacecraft, root, RateGyro(period_s=1.0, angle_random_walk=gyro_noise, rate_random_walk=gyro_bias_walk,
                                           initial_bias=gyro_bias_rps, seed=gyro_seed))
    add_sensor!(spacecraft, root, StarTracker(period_s=5.0, cross_boresight_noise_rad=star_tracker_sigma_rad,
                                              about_boresight_noise_rad=star_tracker_sigma_rad, seed=star_tracker_seed))

    estimate = AttitudeEstimate()
    attitude_filter = AttitudeFilter(estimate=estimate, gyro=1, star_tracker=2, period_s=1.0, gyro_noise=gyro_noise,
                                     gyro_bias_walk=gyro_bias_walk, star_tracker_sigma_rad=star_tracker_sigma_rad)
    controller = PDPointingController(estimate=estimate, target_q=Tuple(target_q), period_s=1.0)
    scheduler = BaseThrusterModel(thrust=[50.0], direction=[0.0], Δv=[0.0], start_burn_time=[-1.0],
                                  stop_burn_time=[-1.0], Isp=[300.0])
    planner = AlgorithmGuidance(algorithm=PeriapsisRaisePlanner(target_periapsis_altitude_m=target_periapsis_altitude_m),
                                spacecraft_index=1, events=[AlgorithmEvent(0.0; reason=:initial)])

    return SimulationConfiguration(
        simulation_settings=SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false),
        mission_configuration=MissionConfiguration(mission_type=MissionTime, mission_time=Float64(mission_time_s),
            number_of_orbits=1, keplerian=true, orientation_sim=true, num_steps_to_save=round(Int, mission_time_s),
            data_rate=1.0),
        environment_model=EnvironmentModel(planet=planet, EI=120.0, density_model=NoAtmosphereModel(),
            ephemerides_model=SimpleEphemeridesModel(),
            thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet), topography=false, wind=false),
        dynamics_model=DynamicsModel([spacecraft], (InverseSquaredGravityModel(),)),
        guidance_model=GuidanceModel(guidance_effectors=(planner,), guidance_rates=[60.0]),
        navigation_model=NavigationModel(navigation_effectors=(attitude_filter,), navigation_rates=[attitude_filter.period_s]),
        control_model=ControlModel(control_effectors=(scheduler, controller), control_rates=[1.0, controller.period_s]),
        initial_time=InitialTime(year=2024, month=1, day=1, hour=0, minute=0, second=0.0),
        integration_tolerances=IntegrationTolerances())
end
