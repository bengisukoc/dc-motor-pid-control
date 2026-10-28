"""DC motor modelling and feedback-control tools."""

from .model import MotorParameters, PIDGains, response_metrics, simulate

__all__ = ["MotorParameters", "PIDGains", "response_metrics", "simulate"]

