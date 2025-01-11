from . import BaseController
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, NamedTuple


# class Controller(BaseController):
#   """
#   A simple PID controller
#   """
#   def __init__(self,):
#     self.p = 0.3
#     self.i = 0.05
#     self.d = -0.1
#     self.error_integral = 0
#     self.prev_error = 0

#   def update(self, target_lataccel, current_lataccel, state, future_plan):
#       error = (target_lataccel - current_lataccel)
#       self.error_integral += error
#       error_diff = error - self.prev_error
#       self.prev_error = error
#       return self.p * error + self.i * self.error_integral + self.d * error_diff


# class ControlMetrics:
#     """Tracks controller performance metrics during execution"""
#     def __init__(self):
#         self.lataccel_errors = []
#         self.jerk_measures = []
#         self.steering_rates = []
#         self.saturation_events = 0
        
#     def update(self, error: float, jerk: float, steering_rate: float, is_saturated: bool):
#         self.lataccel_errors.append(error)
#         self.jerk_measures.append(jerk)
#         self.steering_rates.append(steering_rate)
#         if is_saturated:
#             self.saturation_events += 1
            
#     def get_summary(self) -> dict:
#         return {
#             'rms_error': np.sqrt(np.mean(np.array(self.lataccel_errors)**2)),
#             'peak_jerk': max(self.jerk_measures),
#             'steering_smoothness': np.std(self.steering_rates),
#             'saturation_count': self.saturation_events
#         }

# class Controller(BaseController):
#     """
#     Enhanced lateral controller combining feedforward prediction, adaptive feedback,
#     and performance monitoring.
#     """
#     def __init__(self):
#         # Core gains (will be tuned by evolution)
#         self.gains = {
#             'low_speed': {'p': 0.3, 'i': 0.05, 'd': -0.1},
#             'high_speed': {'p': 0.4, 'i': 0.02, 'd': -0.05}
#         }
        
#         # Feedforward and filtering parameters
#         self.ff_gain = 0.1
#         self.smoothing_alpha = 0.5  # Anti-jerk filter coefficient
        
#         # Controller state
#         self.error_integral = 0.0
#         self.prev_error = 0.0
#         self.prev_output = 0.0
#         self.prev_lataccel = 0.0
        
#         # Performance tracking
#         self.metrics = ControlMetrics()
        
#         # Control limits
#         self.steering_limits = (-0.5, 0.5)
#         self.max_steering_rate = 0.1  # rad/s
        
#         # Gain scheduling threshold
#         self.speed_threshold = 10.0  # m/s
        
#     def compute_feedforward(self, target_lataccel: float, road_lataccel: float) -> float:
#         """Predictive feedforward term based on target and road conditions"""
#         return self.ff_gain * (target_lataccel + road_lataccel)
#         """
#         Main control update implementing the hybrid feedforward-feedback architecture.
#         """
#         # Extract relevant state information
#         v_ego = getattr(state, 'v_ego', 0.0)
#         roll_lataccel = getattr(state, 'roll_lataccel', 0.0)
        
#         # 1. Compute adaptive gains based on velocity
#         gains = self.compute_gains(v_ego)
        
#         # 2. Feedforward term (predictive component)
#         ff_steer = self.compute_feedforward(target_lataccel, roll_lataccel)
        
#         # 3. Feedback computations
#         error = target_lataccel - current_lataccel
        
#         # Compute error derivative with filtering
#         deriv_error = (error - self.prev_error) / 0.1  # 10Hz control rate
        
#         # Update integral with anti-windup
#         if abs(self.prev_output) < max(abs(x) for x in self.steering_limits):
#             self.error_integral += error * 0.1  # dt = 0.1s
        
#         # Compute feedback term
#         fb_steer = (
#             gains['p'] * error +
#             gains['i'] * self.error_integral +
#             gains['d'] * deriv_error
#         )
        
#         # 4. Combine feedforward and feedback
#         raw_steer = ff_steer + fb_steer
        
#         # 5. Apply anti-jerk filtering
#         filtered_steer = (
#             self.prev_output + 
#             self.smoothing_alpha * (raw_steer - self.prev_output)
#         )
        
#         # 6. Apply steering limits
#         limited_steer = np.clip(
#             filtered_steer, 
#             self.steering_limits[0], 
#             self.steering_limits[1]
#         )
        
#         # 7. Compute metrics for performance tracking
#         jerk = (current_lataccel - self.prev_lataccel) / 0.1
#         steering_rate = (limited_steer - self.prev_output) / 0.1
#         is_saturated = abs(limited_steer) >= max(abs(x) for x in self.steering_limits)
        
#         self.metrics.update(
#             error=error,
#             jerk=jerk,
#             steering_rate=steering_rate,
#             is_saturated=is_saturated
#         )
        
#         # 8. Update controller state
#         self.prev_error = error
#         self.prev_output = limited_steer
#         self.prev_lataccel = current_lataccel
        
#         return limited_steer
    
#     def compute_gains(self, v_ego: float) -> dict[str, float]:
#       """
#       Adaptive gain computation based on vehicle velocity.
      
#       We use velocity-based interpolation rather than simple threshold switching
#       to ensure smooth transitions in control behavior.
#       """
#       # Smooth interpolation factor based on velocity
#       alpha = np.clip((v_ego - self.speed_threshold) / 5.0, 0.0, 1.0)
      
#       # Interpolate between low and high speed gains
#       return {
#           'p': (1 - alpha) * self.gains['low_speed']['p'] + alpha * self.gains['high_speed']['p'],
#           'i': (1 - alpha) * self.gains['low_speed']['i'] + alpha * self.gains['high_speed']['i'],
#           'd': (1 - alpha) * self.gains['low_speed']['d'] + alpha * self.gains['high_speed']['d']
#       }

#     def update(self, target_lataccel: float, current_lataccel: float, state: NamedTuple, future_plan=None):
        # """
        # Main control update implementing the hybrid feedforward-feedback architecture.
        # """
        # # Extract relevant state information
        # v_ego = getattr(state, 'v_ego', 0.0)
        # roll_lataccel = getattr(state, 'roll_lataccel', 0.0)
        
        # # 1. Compute adaptive gains based on velocity
        # gains = self.compute_gains(v_ego)
        
        # # 2. Feedforward term (predictive component)
        # ff_steer = self.compute_feedforward(target_lataccel, roll_lataccel)
        
        # # 3. Feedback computations
        # error = target_lataccel - current_lataccel
        
        # # Compute error derivative with filtering
        # deriv_error = (error - self.prev_error) / 0.1  # 10Hz control rate
        
        # # Update integral with anti-windup
        # if abs(self.prev_output) < max(abs(x) for x in self.steering_limits):
        #     self.error_integral += error * 0.1  # dt = 0.1s
        
        # # Compute feedback term
        # fb_steer = (
        #     gains['p'] * error +
        #     gains['i'] * self.error_integral +
        #     gains['d'] * deriv_error
        # )
        
        # # 4. Combine feedforward and feedback
        # raw_steer = ff_steer + fb_steer
        
        # # 5. Apply anti-jerk filtering
        # filtered_steer = (
        #     self.prev_output + 
        #     self.smoothing_alpha * (raw_steer - self.prev_output)
        # )
        
        # # 6. Apply steering limits
        # limited_steer = np.clip(
        #     filtered_steer, 
        #     self.steering_limits[0], 
        #     self.steering_limits[1]
        # )
        
        # # 7. Compute metrics for performance tracking
        # jerk = (current_lataccel - self.prev_lataccel) / 0.1
        # steering_rate = (limited_steer - self.prev_output) / 0.1
        # is_saturated = abs(limited_steer) >= max(abs(x) for x in self.steering_limits)
        
        # self.metrics.update(
        #     error=error,
        #     jerk=jerk,
        #     steering_rate=steering_rate,
        #     is_saturated=is_saturated
        # )
        
        # # 8. Update controller state
        # self.prev_error = error
        # self.prev_output = limited_steer
        # self.prev_lataccel = current_lataccel
        
        # return limited_steer



# @dataclass
# class ControlMetrics:
#     """Tracks controller performance metrics during execution"""
#     def __init__(self):
#         self.lataccel_errors = []
#         self.jerk_measures = []
#         self.steering_rates = []
#         self.saturation_events = 0
        
#     def update(self, error: float, jerk: float, steering_rate: float, is_saturated: bool):
#         self.lataccel_errors.append(error)
#         self.jerk_measures.append(jerk)
#         self.steering_rates.append(steering_rate)
#         if is_saturated:
#             self.saturation_events += 1
            
#     def get_summary(self) -> dict:
#         if not self.lataccel_errors:  # Handle empty case
#             return {
#                 'rms_error': 0.0,
#                 'peak_jerk': 0.0,
#                 'steering_smoothness': 0.0,
#                 'saturation_count': 0
#             }
#         return {
#             'rms_error': float(np.sqrt(np.mean(np.array(self.lataccel_errors)**2))),
#             'peak_jerk': float(max(abs(x) for x in self.jerk_measures)),
#             'steering_smoothness': float(np.std(self.steering_rates)),
#             'saturation_count': self.saturation_events
#         }

# class Controller:
#     """Enhanced lateral controller combining feedforward prediction and adaptive feedback"""
#     def __init__(self):
#         # Core gains (tuned for baseline performance)
#         self.gains = {
#             'low_speed': {'p': 0.3, 'i': 0.05, 'd': -0.1},
#             'high_speed': {'p': 0.4, 'i': 0.02, 'd': -0.05}
#         }
        
#         # Feedforward and filtering parameters
#         self.ff_gain = 0.1
#         self.smoothing_alpha = 0.5
        
#         # Controller state
#         self.error_integral = 0.0
#         self.prev_error = 0.0
#         self.prev_output = 0.0
#         self.prev_lataccel = 0.0
        
#         # Performance tracking
#         self.metrics = ControlMetrics()
        
#         # Control limits
#         self.steering_limits = (-2.0, 2.0)  # Matches environment limits
#         self.max_steering_rate = 0.1  # rad/s
        
#         # Gain scheduling threshold
#         self.speed_threshold = 10.0  # m/s
        
#     def compute_feedforward(self, target_lataccel: float, roll_lataccel: float) -> float:
#         """Predictive feedforward term based on target and road conditions"""
#         return self.ff_gain * (target_lataccel + roll_lataccel)
        
#     def compute_gains(self, v_ego: float) -> Dict[str, float]:
#         """Adaptive gain computation based on vehicle velocity"""
#         alpha = np.clip((v_ego - self.speed_threshold) / 5.0, 0.0, 1.0)
#         return {
#             'p': (1 - alpha) * self.gains['low_speed']['p'] + alpha * self.gains['high_speed']['p'],
#             'i': (1 - alpha) * self.gains['low_speed']['i'] + alpha * self.gains['high_speed']['i'],
#             'd': (1 - alpha) * self.gains['low_speed']['d'] + alpha * self.gains['high_speed']['d']
#         }

#     def update(self, target_lataccel: float, current_lataccel: float, 
#               state: NamedTuple, future_plan: Optional[NamedTuple] = None) -> float:
#         """Main control update implementing the hybrid feedforward-feedback architecture"""
#         # Handle first update when current_lataccel might be None
#         if current_lataccel is None:
#             current_lataccel = 0.0
            
#         # Extract state information safely
#         v_ego = getattr(state, 'v_ego', 0.0)
#         roll_lataccel = getattr(state, 'roll_lataccel', 0.0)
        
#         # 1. Compute adaptive gains based on velocity
#         gains = self.compute_gains(v_ego)
        
#         # 2. Feedforward term (predictive component)
#         ff_steer = self.compute_feedforward(target_lataccel, roll_lataccel)
        
#         # 3. Feedback computations
#         error = target_lataccel - current_lataccel
        
#         # Compute error derivative with filtering
#         deriv_error = (error - self.prev_error) / 0.1  # 10Hz control rate
        
#         # Update integral with anti-windup
#         if abs(self.prev_output) < max(abs(x) for x in self.steering_limits):
#             self.error_integral += error * 0.1
        
#         # Compute feedback term
#         fb_steer = (
#             gains['p'] * error +
#             gains['i'] * self.error_integral +
#             gains['d'] * deriv_error
#         )
        
#         # 4. Combine feedforward and feedback
#         raw_steer = ff_steer + fb_steer
        
#         # 5. Apply anti-jerk filtering
#         filtered_steer = (
#             self.smoothing_alpha * raw_steer +
#             (1 - self.smoothing_alpha) * self.prev_output
#         )
        
#         # 6. Apply steering limits
#         limited_steer = np.clip(
#             filtered_steer, 
#             self.steering_limits[0], 
#             self.steering_limits[1]
#         )
        
#         # 7. Compute metrics
#         jerk = (current_lataccel - self.prev_lataccel) / 0.1
#         steering_rate = (limited_steer - self.prev_output) / 0.1
#         is_saturated = abs(limited_steer) >= max(abs(x) for x in self.steering_limits)
        
#         self.metrics.update(
#             error=error,
#             jerk=jerk,
#             steering_rate=steering_rate,
#             is_saturated=is_saturated
#         )
        
#         # 8. Update controller state
#         self.prev_error = error
#         self.prev_output = limited_steer
#         self.prev_lataccel = current_lataccel
        
#         return limited_steer



# @dataclass
# class ControlState:
#     """Internal state management for controller continuity"""
#     error_integral: float = 0.0
#     prev_error: float = 0.0
#     prev_output: float = 0.0
#     prev_filtered: float = 0.0
    
#     def reset(self):
#         """Reset controller state"""
#         self.error_integral = 0.0
#         self.prev_error = 0.0
#         self.prev_output = 0.0
#         self.prev_filtered = 0.0

# class Controller:
#     """Refined PID controller with smooth response characteristics"""
    
#     def __init__(self):
#         # Core timing
#         self.dt = 0.1  # 10Hz control rate
        
#         # Control gains - tuned for balance of performance and smoothness
#         self.gains = {
#             'p': 0.25,  # Strong proportional for tracking
#             'i': 0.01,  # Light integral for steady-state
#             'd': -0.15  # Robust derivative damping
#         }
        
#         # Smoothing parameter - single alpha for consistency
#         self.smoothing_alpha = 0.4
        
#         # System constraints
#         self.steering_limits = (-2.0, 2.0)
#         self.max_steering_rate = 0.1
        
#         # Initialize state
#         self.state = ControlState()
    
#     def apply_smoothing(self, raw_command: float) -> float:
#         """Apply exponential smoothing to raw command"""
#         filtered = (self.smoothing_alpha * raw_command + 
#                    (1 - self.smoothing_alpha) * self.state.prev_filtered)
#         self.state.prev_filtered = filtered
#         return filtered
    
#     def enforce_limits(self, command: float) -> float:
#         """Apply rate and position limits to command"""
#         # Rate limiting
#         max_change = self.max_steering_rate * self.dt
#         rate_limited = np.clip(
#             command - self.state.prev_output,
#             -max_change,
#             max_change
#         )
        
#         # Position limiting
#         position_limited = np.clip(
#             self.state.prev_output + rate_limited,
#             self.steering_limits[0],
#             self.steering_limits[1]
#         )
        
#         return position_limited
    
#     def update(self, target_lataccel: float, current_lataccel: float, 
#               state: NamedTuple, future_plan: Optional[NamedTuple] = None) -> float:
#         """Execute one control cycle with refined PID logic"""
#         # Initialize if needed
#         if current_lataccel is None:
#             current_lataccel = 0.0
        
#         # Core PID computation
#         error = target_lataccel - current_lataccel
        
#         # Update integral with anti-windup
#         if abs(self.state.prev_output) < max(abs(x) for x in self.steering_limits):
#             self.state.error_integral += error * self.dt
            
#         # Compute filtered derivative
#         error_derivative = (error - self.state.prev_error) / self.dt
        
#         # Compute raw command
#         raw_command = (
#             self.gains['p'] * error +
#             self.gains['i'] * self.state.error_integral +
#             self.gains['d'] * error_derivative
#         )
        
#         # Apply smoothing filter
#         filtered_command = self.apply_smoothing(raw_command)
        
#         # Apply system limits
#         final_command = self.enforce_limits(filtered_command)
        
#         # Update controller state
#         self.state.prev_error = error
#         self.state.prev_output = final_command
        
#         return final_command




@dataclass
class AdaptiveState:
    """Enhanced state tracking for adaptive control"""
    error_integral: float = 0.0
    prev_error: float = 0.0
    prev_output: float = 0.0
    prev_filtered: float = 0.0
    error_window: List[float] = field(default_factory=list)
    curvature_estimate: float = 0.0
    
    def update_curvature(self, future_plan) -> None:
        """Estimate path curvature from future accelerations."""
        if not future_plan or not future_plan.lataccel:
            return
        
        accel_sequence = np.array(future_plan.lataccel[:10])
        if len(accel_sequence) < 2:
            return
            
        # Approximate curvature from acceleration changes (simple heuristic)
        self.curvature_estimate = np.abs(np.diff(accel_sequence)).mean()

class Controller:
    """Advanced adaptive controller with variable horizon and refined feedforward."""
    
    def __init__(self):
        self.dt = 0.1
        
        # Core gains (base PID terms)
        self.base_gains = {
            'p': 0.22,
            'i': 0.008,
            'd': -0.12
        }
        
        # Adaptive ranges for gains
        self.gain_ranges = {
            'p': (0.18, 0.28),
            'i': (0.005, 0.012),
            'd': (-0.15, -0.08)
        }
        
        # Feedforward curvature gain (for #3)
        # You can tune this factor in testing.
        self.curvature_gain = 0.3
        
        # Variable preview parameters
        self.base_preview_horizon = 8
        self.max_preview_horizon = 15
        self.min_preview_horizon = 5
        
        # Smoothing configuration
        self.base_smoothing = 0.35
        self.velocity_smooth_factor = 0.15
        
        # System constraints
        self.steering_limits = (-2.0, 2.0)
        self.max_steering_rate = 0.1
        
        # Initialize state
        self.state = AdaptiveState()
    
    def compute_variable_horizon(self, velocity: float, curvature: float) -> int:
        """
        Adaptive horizon logic:
          - Increase horizon with velocity (faster speed => further lookahead).
          - Decrease horizon in high-curvature segments (tighter turns => shorter horizon).
        """
        vel_factor = np.clip(velocity / 30.0, 0.0, 1.0)
        curve_factor = np.clip(1.0 - curvature / 0.5, 0.0, 1.0)
        
        horizon_factor = 0.7 * vel_factor + 0.3 * curve_factor
        horizon = self.min_preview_horizon + horizon_factor * (
            self.max_preview_horizon - self.min_preview_horizon
        )
        
        return int(round(horizon))
    
    def compute_adaptive_gains(self, error_stats: Dict[str, float], 
                               velocity: float, curvature: float) -> Dict[str, float]:
        """
        Dynamically interpolate PID gains based on:
          - velocity
          - curvature
          - recent error statistics
        """
        vel_factor = np.clip(velocity / 30.0, 0.0, 1.0)
        curve_factor = np.clip(1.0 - curvature / 0.5, 0.0, 1.0)
        error_factor = np.clip(1.0 - error_stats.get('std', 0.0) / 0.5, 0.0, 1.0)
        
        # Weighted blend of these factors
        adaptation = 0.5 * vel_factor + 0.3 * curve_factor + 0.2 * error_factor
        
        gains = {}
        for gain_type, base_gain in self.base_gains.items():
            low, high = self.gain_ranges[gain_type]
            gains[gain_type] = low + adaptation * (high - low)
            
        return gains
    
    def compute_feedforward(self, future_plan, velocity: float) -> float:
        """
        Refined feedforward (#3):
          - Use curvature directly as a polynomial term
          - Use a weighted sum of lataccel preview
        """
        if not future_plan or not future_plan.lataccel:
            return 0.0
            
        # 1) Compute adaptive horizon
        horizon = self.compute_variable_horizon(
            velocity, 
            self.state.curvature_estimate
        )
        
        preview = future_plan.lataccel[:horizon]
        if not preview:
            return 0.0
        
        # 2) Weighted sum of predicted accelerations
        decay_factor = 0.15 + 0.1 * (velocity / 30.0)  # velocity-based decay
        weights = np.exp(-np.arange(len(preview)) * decay_factor)
        weights /= np.sum(weights)  # normalize
        
        weighted_accel_sum = np.sum(weights * np.array(preview)) * 0.25
        
        # 3) Polynomial curvature term (e.g., linear for now)
        polynomial_term = self.curvature_gain * self.state.curvature_estimate
        
        return polynomial_term + weighted_accel_sum
    
    def update(self, target_lataccel: float, current_lataccel: float, 
               state: NamedTuple, future_plan: Optional[NamedTuple] = None) -> float:
        """
        Main control loop with:
          (1) anti-windup
          (2) jerk penalty
          (3) refined feedforward using curvature
        """
        if current_lataccel is None:
            current_lataccel = 0.0
        
        # 1. Compute error
        error = target_lataccel - current_lataccel
        self.state.error_window.append(error)
        if len(self.state.error_window) > 10:
            self.state.error_window.pop(0)
        
        # 2. Update curvature estimation
        self.state.update_curvature(future_plan)
        
        # 3. Error statistics
        error_stats = {
            'mean': np.mean(self.state.error_window),
            'std': np.std(self.state.error_window) if len(self.state.error_window) > 1 else 0.0
        }
        
        # 4. Adaptive PID gains
        gains = self.compute_adaptive_gains(
            error_stats, 
            state.v_ego,
            self.state.curvature_estimate
        )
        
        # 5. Integral update with dynamic anti-windup check (#1)
        # Only integrate if we’re not near the steering saturation limit
        if abs(self.state.prev_output) < 0.9 * max(abs(x) for x in self.steering_limits):
            self.state.error_integral += error * self.dt
        
        # 6. Derivative term
        error_derivative = (error - self.state.prev_error) / self.dt
        
        # 7. Feedforward (#3) (curvature + lataccel preview)
        feedforward = self.compute_feedforward(future_plan, state.v_ego)
        
        # 8. Raw PID + feedforward
        command = (
            gains['p'] * error
            + gains['i'] * self.state.error_integral
            + gains['d'] * error_derivative
            + feedforward
        )
        
        # 9. Jerk-limiting penalty (#2)
        steering_rate = (self.state.prev_output - self.state.prev_filtered) / self.dt
        jerk_penalty = 0.0
        if abs(steering_rate) > 0.005:  # threshold to penalize big output changes
            jerk_penalty = 0.05 * steering_rate  # tune weight
        command -= jerk_penalty
        
        # 10. Anti-windup clamp
        pre_smooth_command = np.clip(command, *self.steering_limits)
        if not np.isclose(pre_smooth_command, command):
            overshoot = command - pre_smooth_command
            if abs(gains['i']) > 1e-9:
                self.state.error_integral -= 0.5 * (overshoot / gains['i'])
        
        # 11. Dynamic smoothing
        smoothing = self.base_smoothing + self.velocity_smooth_factor * (state.v_ego / 30.0)
        filtered = smoothing * pre_smooth_command + (1.0 - smoothing) * self.state.prev_filtered
        
        # Update prev_filtered so we can compute next loop's steering_rate
        self.state.prev_filtered = filtered
        
        # 12. Rate limiting
        max_change = self.max_steering_rate * self.dt
        rate_limited = np.clip(filtered - self.state.prev_output, -max_change, max_change)
        
        # 13. Position limiting
        final_command = np.clip(self.state.prev_output + rate_limited, *self.steering_limits)
        
        # 14. Update state
        self.state.prev_error = error
        self.state.prev_output = final_command
        
        return final_command