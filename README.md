

**Problem:**
Grid-tied inverters (solar, battery storage, wind) must synchronize their output voltage phase with the utility grid before connecting. Discrete-time phase tracking introduces **angle wrapping issues** at ±π boundaries, causing:
- Sudden jumps in computed angular velocity (delta_w)
- False frequency estimates (50 Hz appearing as 500+ Hz)
- Controller instability and failed synchronization

**Solution:** Proper phase unwrapping using `atan2(sin(θ), cos(θ))` and discontinuity compensation.

**Test Scenarios:**
  1. Normal operation baseline (stable 50 Hz tracking)
  2. 30° phase jump disturbance rejection
  3. Frequency jump response (50 Hz → 51 Hz)
  4. Direct comparison: WITH vs WITHOUT unwrapping

**Phase Unwrapping Method:**
```matlab
theta = atan2(sin(theta), cos(theta));  % Keep in [-π, π]

% Handle delta_w discontinuities
if delta_w_raw > π:  delta_w = delta_w_raw - 2π
if delta_w_raw < -π: delta_w = delta_w_raw + 2π
```


***Some Graphs***

Normal Operation:
![graphs](phaseunwrapping/Screenshot%202025-10-25%20012909.png)

30 deg phase jump added:
![graphs](phaseunwrapping/Screenshot%202025-10-25%20013040.png)

comparison:
![graphs](phaseunwrapping/Screenshot%202025-10-25%20013107.png)


