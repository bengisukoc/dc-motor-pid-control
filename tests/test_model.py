import numpy as np

from motor_control import MotorParameters, PIDGains, response_metrics, simulate


def test_zero_command_keeps_motor_stationary():
    data = simulate(PIDGains(0.2), duration=0.05, command_time=1.0, load_time=2.0)
    assert np.allclose(data["Speed (rad/s)"], 0.0)


def test_voltage_is_limited():
    p = MotorParameters(voltage_limit=12.0)
    data = simulate(PIDGains(10, 10, 1), parameters=p, duration=0.2)
    assert data["Voltage (V)"].abs().max() <= 12.0


def test_pid_tracks_positive_speed():
    data = simulate(PIDGains(0.28, 1.4, 0.0015, "PID"))
    assert data["Speed (rad/s)"].iloc[-1] > 140
    assert response_metrics(data)["Final absolute error (rad/s)"] < 10


def test_invalid_timestep_is_rejected():
    try:
        simulate(PIDGains(0.2), dt=0)
    except ValueError:
        return
    raise AssertionError("Expected ValueError")

