import numpy as np
from simple_pid import PID


class PIDController:

    def __init__(self, ssim_range, size_range, ssim_setpoint, size_setpoint):
        self.ssim_range = ssim_range
        self.size_range = size_range
        self.ssim_pid =  PID(0.6, 0.5, 0.125, setpoint=ssim_setpoint * (self.ssim_range[1] - self.ssim_range[0]))
        self.ssim_pid.output_limits = ssim_range
        self.size_pid =  PID(0.6, 0.5, 0.125, setpoint=size_setpoint * (self.size_range[1] - self.size_range[0]))
        self.size_pid.output_limits = size_range

    def compute_u(self, qual_idx, size_idx):
        ssim_actuator_estim = self.ssim_pid(qual_idx * (self.ssim_range[1] - self.ssim_range[0]))
        size_actuator_estim = self.size_pid(size_idx * (self.size_range[1] - self.size_range[0]))

        return (size_actuator_estim, ssim_actuator_estim, ssim_actuator_estim)
