The CSV structure reveals the core data we have to work with for our lateral control system. This helps us understand both the available inputs and the target behavior we need to achieve. Let's modify our test harness to align with this structure while maintaining our sophisticated control approach.

Available columns: ['t', 'vEgo', 'aEgo', 'roll', 'targetLateralAcceleration', 'steerCommand']

Core State Variables

- vEgo: Vehicle velocity in m/s
- aEgo: Forward acceleration in m/s²
- roll: Road roll angle in radians
- targetLateralAcceleration: Desired lateral acceleration
- steerCommand: Applied steering command

First row sample:
t                             0.000000
vEgo                         33.770260
aEgo                         -0.017300
roll                          0.037470
targetLateralAcceleration     1.003864
steerCommand                 -0.329734


# TEST 1 ROUTE
`python tinyphysics.py --model_path ./models/tinyphysics.onnx --data_path ./data/00000.csv --controller pid`

`python quick_sample_test.py --model_path ./models/tinyphysics.onnx --data_path ./data --sample_size 50`