"""Physics model, PID controller and response metrics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class MotorParameters:
    resistance: float = 1.0       # ohm
    inductance: float = 0.05      # H
    torque_constant: float = 0.1  # N m / A
    back_emf_constant: float = 0.1  # V s / rad
    inertia: float = 0.01         # kg m^2
    viscous_friction: float = 0.001  # N m s / rad
    voltage_limit: float = 24.0   # V


@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    name: str = "Controller"


def _derivatives(state: np.ndarray, voltage: float, load: float, p: MotorParameters) -> np.ndarray:
    current, speed = state
    di = (voltage - p.resistance * current - p.back_emf_constant * speed) / p.inductance
    domega = (p.torque_constant * current - p.viscous_friction * speed - load) / p.inertia
    return np.array([di, domega])


def _rk4_step(state: np.ndarray, voltage: float, load: float, dt: float,
              p: MotorParameters) -> np.ndarray:
    k1 = _derivatives(state, voltage, load, p)
    k2 = _derivatives(state + 0.5 * dt * k1, voltage, load, p)
    k3 = _derivatives(state + 0.5 * dt * k2, voltage, load, p)
    k4 = _derivatives(state + dt * k3, voltage, load, p)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def simulate(
    gains: PIDGains,
    parameters: MotorParameters = MotorParameters(),
    duration: float = 4.0,
    dt: float = 0.001,
    target_speed: float = 150.0,
    command_time: float = 0.10,
    load_time: float = 2.50,
    load_torque: float = 0.20,
) -> pd.DataFrame:
    """Simulate motor response to a speed step and later load disturbance."""
    if duration <= 0 or dt <= 0:
        raise ValueError("duration and dt must be positive")
    time = np.arange(0.0, duration + dt, dt)
    state = np.zeros(2)
    integral = 0.0
    previous_error = 0.0
    rows = []

    for index, t in enumerate(time):
        target = target_speed if t >= command_time else 0.0
        load = load_torque if t >= load_time else 0.0
        error = target - state[1]
        derivative = 0.0 if index == 0 else (error - previous_error) / dt
        candidate_integral = integral + error * dt
        raw_voltage = gains.kp * error + gains.ki * candidate_integral + gains.kd * derivative
        voltage = float(np.clip(raw_voltage, -parameters.voltage_limit, parameters.voltage_limit))

        # Conditional integration prevents the integral term growing while the
        # actuator is saturated in the same direction as the error.
        saturated_high = raw_voltage > parameters.voltage_limit and error > 0
        saturated_low = raw_voltage < -parameters.voltage_limit and error < 0
        if not (saturated_high or saturated_low):
            integral = candidate_integral

        rows.append({
            "Time (s)": t,
            "Target speed (rad/s)": target,
            "Speed (rad/s)": state[1],
            "Current (A)": state[0],
            "Voltage (V)": voltage,
            "Load torque (N m)": load,
            "Error (rad/s)": error,
        })
        state = _rk4_step(state, voltage, load, dt, parameters)
        previous_error = error

    return pd.DataFrame(rows)


def response_metrics(data: pd.DataFrame, command_time: float = 0.10,
                     load_time: float = 2.50) -> dict[str, float]:
    """Calculate standard step-response metrics before the load disturbance."""
    after = data[(data["Time (s)"] >= command_time) & (data["Time (s)"] < load_time)]
    target = float(after["Target speed (rad/s)"].iloc[-1])
    time = after["Time (s)"].to_numpy()
    speed = after["Speed (rad/s)"].to_numpy()

    def first_crossing(level: float) -> float:
        indices = np.flatnonzero(speed >= level)
        return float(time[indices[0]]) if len(indices) else np.nan

    t10, t90 = first_crossing(0.1 * target), first_crossing(0.9 * target)
    rise = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan
    overshoot = max(0.0, 100 * (speed.max() - target) / target)
    tolerance = 0.02 * target
    outside = np.flatnonzero(np.abs(speed - target) > tolerance)
    settling = np.nan
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] < len(time) - 1:
        settling = float(time[outside[-1] + 1] - command_time)
    tail = data[data["Time (s)"] >= data["Time (s)"].max() - 0.1]
    steady_error = float(np.abs(tail["Error (rad/s)"].mean()))
    recovery = data[data["Time (s)"] >= load_time]
    outside_after_load = np.flatnonzero(np.abs(recovery["Error (rad/s)"].to_numpy()) > tolerance)
    if len(outside_after_load) == 0:
        recovery_time = 0.0
    else:
        later = recovery.iloc[outside_after_load[0]:]
        recovered = later[np.abs(later["Error (rad/s)"]) <= tolerance]
        recovery_time = float(recovered["Time (s)"].iloc[0] - load_time) if not recovered.empty else np.nan
    return {
        "Rise time (s)": rise,
        "Overshoot (%)": overshoot,
        "Settling time (s)": settling,
        "Final absolute error (rad/s)": steady_error,
        "Load recovery time (s)": recovery_time,
    }

