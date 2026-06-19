from mowgli_tools.drive_pid_math import (
    DrivePidParams,
    SpeedSample,
    compute_settling_time,
    compute_trial_metrics,
    detect_oscillation,
    recommend_drive_pid_params,
    recommend_pid_only_params,
)


def _params() -> DrivePidParams:
    return DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=18.0,
        wheel_pid_ki=700.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=30.0,
        wheel_pid_pwm_per_mps=550.0,
    )


def _early_pid_params(kp: float = 0.0) -> DrivePidParams:
    return DrivePidParams(
        ticks_per_meter=533.0,
        wheel_pid_kp=kp,
        wheel_pid_ki=0.0,
        wheel_pid_kd=0.0,
        wheel_pid_integral_limit=0.0,
        wheel_pid_pwm_per_mps=550.0,
    )


def test_compute_settling_time_returns_first_stable_point() -> None:
    samples = [
        SpeedSample(0.0, 0.00),
        SpeedSample(0.5, 0.12),
        SpeedSample(1.0, 0.19),
        SpeedSample(1.5, 0.20),
        SpeedSample(2.0, 0.20),
        SpeedSample(2.5, 0.20),
    ]
    settling = compute_settling_time(samples, target_speed=0.20, tolerance_mps=0.02, min_hold_s=1.0)
    assert settling == 1.0


def test_detect_oscillation_flags_repeated_crossings() -> None:
    samples = [
        SpeedSample(0.0, 0.10),
        SpeedSample(0.2, 0.28),
        SpeedSample(0.4, 0.12),
        SpeedSample(0.6, 0.29),
        SpeedSample(0.8, 0.11),
        SpeedSample(1.0, 0.30),
    ]
    assert detect_oscillation(samples, target_speed=0.20)


def test_recommend_drive_pid_params_increases_feedforward_when_robot_is_slow() -> None:
    base_params = _params()
    feedforward_trial = compute_trial_metrics(
        name="ff",
        phase="feedforward",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.18),
            SpeedSample(0.5, 0.20),
            SpeedSample(1.0, 0.21),
            SpeedSample(1.5, 0.22),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=0.21,
        odom_distance_m=1.10,
        rtk_distance_m=1.45,
        notes=(),
    )
    response_trial = compute_trial_metrics(
        name="resp",
        phase="response",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.20),
            SpeedSample(0.5, 0.22),
            SpeedSample(1.0, 0.24),
            SpeedSample(1.5, 0.25),
        ],
        response_samples=None,
        ticks_seen=140,
        left_ticks_seen=138,
        right_ticks_seen=142,
        params_used=base_params,
        ground_speed_mean=0.23,
        odom_distance_m=1.20,
        rtk_distance_m=1.50,
        notes=(),
    )
    recommended, reasons = recommend_drive_pid_params(base_params, [feedforward_trial], [response_trial])
    assert recommended.wheel_pid_pwm_per_mps > base_params.wheel_pid_pwm_per_mps
    assert recommended.wheel_pid_kp >= base_params.wheel_pid_kp
    assert recommended.ticks_per_meter < base_params.ticks_per_meter
    assert reasons


def test_recommend_drive_pid_params_softens_oscillatory_response() -> None:
    base_params = _params()
    feedforward_trial = compute_trial_metrics(
        name="ff",
        phase="feedforward",
        target_speed=0.20,
        speed_samples=[
            SpeedSample(0.0, 0.20),
            SpeedSample(0.5, 0.21),
            SpeedSample(1.0, 0.20),
        ],
        response_samples=None,
        ticks_seen=100,
        left_ticks_seen=100,
        right_ticks_seen=100,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.5,
        rtk_distance_m=None,
        notes=(),
    )
    response_trial = compute_trial_metrics(
        name="resp",
        phase="response",
        target_speed=0.20,
        speed_samples=[
            SpeedSample(0.0, 0.10),
            SpeedSample(0.2, 0.28),
            SpeedSample(0.4, 0.12),
            SpeedSample(0.6, 0.30),
            SpeedSample(0.8, 0.11),
            SpeedSample(1.0, 0.31),
        ],
        response_samples=None,
        ticks_seen=130,
        left_ticks_seen=126,
        right_ticks_seen=134,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.7,
        rtk_distance_m=None,
        notes=(),
    )
    recommended, _ = recommend_drive_pid_params(base_params, [feedforward_trial], [response_trial])
    assert recommended.wheel_pid_kp < base_params.wheel_pid_kp
    assert recommended.wheel_pid_ki < base_params.wheel_pid_ki
    assert recommended.wheel_pid_integral_limit <= base_params.wheel_pid_integral_limit


def test_recommend_pid_only_params_keeps_early_pid_conservative_after_good_ff() -> None:
    base_params = _early_pid_params()
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.29),
            SpeedSample(0.2, 0.31),
            SpeedSample(0.4, 0.29),
            SpeedSample(0.6, 0.30),
            SpeedSample(0.8, 0.31),
            SpeedSample(1.0, 0.29),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        live_oscillation_detected=True,
        notes=(),
    )

    recommended, reasons = recommend_pid_only_params(base_params, [response_trial])

    assert 0.2 <= recommended.wheel_pid_kp <= 0.5
    assert recommended.wheel_pid_ki == 0.0
    assert recommended.wheel_pid_kd == 0.0
    assert recommended.wheel_pid_integral_limit == 0.0
    assert any("keeping KI at 0.0" in reason for reason in reasons)
    assert any("Kd remains 0.0" in reason for reason in reasons)


def test_recommend_pid_only_params_uses_small_kp_steps_from_low_gain_start() -> None:
    base_params = _early_pid_params(kp=0.2)
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.22),
            SpeedSample(0.4, 0.24),
            SpeedSample(0.8, 0.27),
            SpeedSample(1.2, 0.28),
            SpeedSample(1.8, 0.28),
            SpeedSample(2.4, 0.28),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=118,
        right_ticks_seen=122,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    recommended, _ = recommend_pid_only_params(base_params, [response_trial])

    assert 0.2 < recommended.wheel_pid_kp <= 0.5
    assert round(recommended.wheel_pid_kp - base_params.wheel_pid_kp, 3) in (0.1, 0.2)
    assert recommended.wheel_pid_ki == 0.0
    assert recommended.wheel_pid_kd == 0.0
    assert recommended.wheel_pid_integral_limit == 0.0


def test_recommend_pid_only_params_flags_imbalance_without_forcing_derivative() -> None:
    base_params = _params()
    response_trial = compute_trial_metrics(
        name="pid_step",
        phase="pid",
        target_speed=0.30,
        speed_samples=[
            SpeedSample(0.0, 0.18),
            SpeedSample(0.2, 0.36),
            SpeedSample(0.4, 0.17),
            SpeedSample(0.6, 0.35),
            SpeedSample(0.8, 0.18),
            SpeedSample(1.0, 0.34),
        ],
        response_samples=None,
        ticks_seen=120,
        left_ticks_seen=96,
        right_ticks_seen=144,
        params_used=base_params,
        ground_speed_mean=None,
        odom_distance_m=0.9,
        rtk_distance_m=None,
        notes=(),
    )

    recommended, reasons = recommend_pid_only_params(base_params, [response_trial])

    assert recommended.wheel_pid_kd == base_params.wheel_pid_kd
    assert any("Left/right wheel response diverged" in reason for reason in reasons)
