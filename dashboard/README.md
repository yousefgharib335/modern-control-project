# Section 4.6 — Furuta dashboard

In MATLAB, open the project root folder and run `run_dashboard`.

The programmatic MATLAB GUI uses `uifigure` and requires MATLAB; it does not require a Simulink model to demonstrate the interface. It displays arm angle, pendulum angle, motor command, angular velocities, controller mode, three rolling time plots, and a synchronized 3D schematic. The window can be resized. Start, pause, reset, and CSV export are available.

The demo refresh target is 20 Hz using a MATLAB timer. These are scripted synthetic signals, not evidence of a working controller, plant model, hardware connection, or hard real-time performance. The mode is supplied with each sample; the dashboard does not implement the Stateflow supervisor. Pause controls the display only and is not a hardware emergency stop.

## Connect the team's signals

Stop the demo and select live input once:

```matlab
dashboard.connectLive();
```

Then call this from the team's MATLAB acquisition/display callback:

```matlab
sample = struct('t', 0.05, ...       % seconds, strictly increasing
    'theta0', 0.12, ...             % arm angle, radians
    'theta1', -0.03, ...            % pendulum angle, radians; upright = 0
    'u', 0.2, ...                   % team-defined actuator command
    'mode', 'Stabilization', ...    % actual supervisory state
    'omega0', 0.1, ...              % optional radians/second
    'omega1', -0.2);                % optional radians/second
dashboard.update(sample);
```

The GUI displays degrees and degrees/second; exported data retain radians and radians/second. Missing angular rates display as `--` and export as NaN. Agree on the hardware angle signs, upright offset, encoder scaling, actuator units, and acquisition transport before integration. Live mode is an input selection, not confirmation that a device is connected. The status shows the latest received timestamp and sample count.

Do not call graphics updates in generated embedded controller code. Feed the GUI from a MATLAB-side acquisition callback at a practical display rate (for example 10–20 Hz), separately from the controller sample rate. Simulink External Mode, serial, or another transport still needs to be implemented once the team selects it.

The schematic is driven by the same input angles. Its unit lengths are illustrative; it can be replaced by the Section 4.5 team's digital twin. It is not yet validated against physical hardware.

## Data logging

Export CSV saves all samples received since the last reset/source change, with columns for time, both angles, motor command, angular rates, mode, and source. Logging remains in memory until reset/close, so export before closing. Long experiments should use the team's acquisition logger for persistent recording.
