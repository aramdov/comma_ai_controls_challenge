import numpy as np
from tinyphysics import TinyPhysicsModel, TinyPhysicsSimulator, CONTROL_START_IDX
import matplotlib.pyplot as plt
from typing import Dict, Optional, Tuple
import seaborn as sns

class ControllerAnalysis:
    """Advanced analysis framework for controller development"""
    
    def __init__(self, model_path: str, data_path: str):
        self.model = TinyPhysicsModel(model_path, debug=True)
        self.data_path = data_path
        
    def analyze_controller(self, controller, plot: bool = True) -> Dict:
        """Run comprehensive controller analysis"""
        # Initialize simulator with controller
        sim = TinyPhysicsSimulator(self.model, self.data_path, controller=controller, debug=False)
        
        # Run simulation
        costs = sim.rollout()
        
        if plot:
            self._plot_detailed_analysis(sim)
            
        # Compute advanced metrics
        advanced_metrics = self._compute_advanced_metrics(sim)
        
        return {
            'costs': costs,
            'metrics': advanced_metrics
        }
    
    def _compute_advanced_metrics(self, sim) -> Dict:
        """Compute detailed controller performance metrics"""
        eval_slice = slice(CONTROL_START_IDX, 500)
        target = np.array(sim.target_lataccel_history)[eval_slice]
        actual = np.array(sim.current_lataccel_history)[eval_slice]
        actions = np.array(sim.action_history)[eval_slice]
        
        # Tracking performance metrics
        tracking_error = target - actual
        rms_error = np.sqrt(np.mean(tracking_error**2))
        max_error = np.max(np.abs(tracking_error))
        
        # Control effort metrics
        action_diff = np.diff(actions)
        steering_rate = np.mean(np.abs(action_diff))
        max_steering_rate = np.max(np.abs(action_diff))
        
        # Spectral analysis of control signal
        freq_content = np.fft.fft(action_diff)
        power_spectrum = np.abs(freq_content)**2
        high_freq_energy = np.sum(power_spectrum[len(power_spectrum)//4:])
        
        return {
            'rms_tracking_error': float(rms_error),
            'max_tracking_error': float(max_error),
            'avg_steering_rate': float(steering_rate),
            'max_steering_rate': float(max_steering_rate),
            'high_frequency_content': float(high_freq_energy)
        }
    
    def _plot_detailed_analysis(self, sim):
        """Generate comprehensive analysis plots"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Lateral acceleration tracking
        ax1.plot(sim.target_lataccel_history, label='Target', alpha=0.7)
        ax1.plot(sim.current_lataccel_history, label='Actual', alpha=0.7)
        ax1.axvline(x=CONTROL_START_IDX, color='r', linestyle='--', alpha=0.5, label='Control Start')
        ax1.set_title('Lateral Acceleration Tracking')
        ax1.set_xlabel('Step')
        ax1.set_ylabel('Lateral Acceleration (m/s²)')
        ax1.legend()
        
        # Steering commands
        ax2.plot(sim.action_history, label='Steering', alpha=0.7)
        ax2.axvline(x=CONTROL_START_IDX, color='r', linestyle='--', alpha=0.5)
        ax2.set_title('Steering Commands')
        ax2.set_xlabel('Step')
        ax2.set_ylabel('Steering Angle')
        
        # Error analysis
        eval_slice = slice(CONTROL_START_IDX, 500)
        error = np.array(sim.target_lataccel_history)[eval_slice] - np.array(sim.current_lataccel_history)[eval_slice]
        ax3.plot(range(CONTROL_START_IDX, 500), error)
        ax3.set_title('Tracking Error')
        ax3.set_xlabel('Step')
        ax3.set_ylabel('Error (m/s²)')
        
        # Control effort analysis
        actions = np.array(sim.action_history)[eval_slice]
        ax4.plot(range(CONTROL_START_IDX, 499), np.diff(actions))
        ax4.set_title('Steering Rate')
        ax4.set_xlabel('Step')
        ax4.set_ylabel('Δ Steering/Step')
        
        plt.tight_layout()
        plt.show()

def analyze_controller(model_path: str, data_path: str, controller) -> Dict:
    """Convenience function for quick controller analysis"""
    analyzer = ControllerAnalysis(model_path, data_path)
    return analyzer.analyze_controller(controller)

if __name__ == "__main__":
    from controllers.pid import Controller
    results = analyze_controller(
        "./models/tinyphysics.onnx",
        "./data/00000.csv",
        Controller()
    )
    
    print("\nPerformance Summary:")
    print(f"Total Cost: {results['costs']['total_cost']:.3f}")
    print(f"Lateral Acceleration Cost: {results['costs']['lataccel_cost']:.3f}")
    print(f"Jerk Cost: {results['costs']['jerk_cost']:.3f}")
    
    print("\nAdvanced Metrics:")
    for metric, value in results['metrics'].items():
        print(f"{metric}: {value:.3f}")