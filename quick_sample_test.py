import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Dict, Optional
from tqdm import tqdm
import random
from tinyphysics import TinyPhysicsModel, TinyPhysicsSimulator

class QuickTester:
    """Efficient testing utility for rapid controller iteration"""
    
    def __init__(self, model_path: str, data_path: str, 
                 sample_size: int = 50, seed: Optional[int] = 42):
        """
        Initialize tester with configurable sample size
        
        Args:
            model_path: Path to the physics model
            data_path: Path to data directory
            sample_size: Number of segments to test (default 50)
            seed: Random seed for reproducibility
        """
        self.model = TinyPhysicsModel(model_path, debug=False)
        self.data_path = Path(data_path)
        self.sample_size = sample_size
        
        if seed is not None:
            random.seed(seed)
            
        # Get stratified sample of segments
        self.segments = self._get_stratified_sample()
        
    def _get_stratified_sample(self) -> List[Path]:
        """Get a stratified sample of segments based on complexity"""
        all_segments = sorted(self.data_path.glob("*.csv"))
        
        if len(all_segments) <= self.sample_size:
            return all_segments
            
        # Simple complexity scoring based on file size
        complexities = [(seg, seg.stat().st_size) for seg in all_segments]
        complexities.sort(key=lambda x: x[1])
        
        # Split into 3 complexity buckets
        n_per_bucket = self.sample_size // 3
        buckets = np.array_split(complexities, 3)
        
        # Sample from each bucket
        sampled_segments = []
        for bucket in buckets:
            sampled_segments.extend(
                random.sample([x[0] for x in bucket], 
                            min(n_per_bucket, len(bucket)))
            )
            
        return sampled_segments
        
    def test_controller(self, controller) -> Dict:
        """
        Run comprehensive test suite on controller
        
        Returns dict with aggregated metrics and detailed per-segment results
        """
        results = []
        
        for segment in tqdm(self.segments, desc="Testing segments"):
            sim = TinyPhysicsSimulator(
                self.model, 
                str(segment), 
                controller=controller,
                debug=False
            )
            
            costs = sim.rollout()
            
            # Compute additional metrics
            results.append({
                'segment': segment.stem,
                **costs
            })
            
        # Aggregate results
        df = pd.DataFrame(results)
        summary = {
            'lataccel_cost': {
                'mean': df.lataccel_cost.mean(),
                'std': df.lataccel_cost.std(),
                'min': df.lataccel_cost.min(),
                'max': df.lataccel_cost.max()
            },
            'jerk_cost': {
                'mean': df.jerk_cost.mean(),
                'std': df.jerk_cost.std(),
                'min': df.jerk_cost.min(),
                'max': df.jerk_cost.max()
            },
            'total_cost': {
                'mean': df.total_cost.mean(),
                'std': df.total_cost.std(),
                'min': df.total_cost.min(),
                'max': df.total_cost.max()
            },
            'detailed_results': df
        }
        
        return summary

def main():
    parser = argparse.ArgumentParser(description='Quick controller testing utility')
    parser.add_argument('--model_path', type=str, required=True)
    parser.add_argument('--data_path', type=str, required=True)
    parser.add_argument('--sample_size', type=int, default=50)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    
    # Import your controller
    from controllers.pid import Controller
    controller = Controller()
    
    # Run tests
    tester = QuickTester(
        args.model_path,
        args.data_path,
        sample_size=args.sample_size,
        seed=args.seed
    )
    
    results = tester.test_controller(controller)
    
    # Print summary
    print("\nTest Results Summary:")
    print("-" * 50)
    for metric in ['lataccel_cost', 'jerk_cost', 'total_cost']:
        print(f"\n{metric.upper()}:")
        for stat, value in results[metric].items():
            if stat != 'detailed_results':
                print(f"  {stat}: {value:.3f}")
    
if __name__ == "__main__":
    main()