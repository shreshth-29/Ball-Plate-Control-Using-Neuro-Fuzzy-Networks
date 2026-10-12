# Ball and Plate Control Using Neuro-Fuzzy Networks

Repository created in September 2022. Reorganized and cleaned up in October 2026 (file structure and README only, the code is as it was written at the time, except where the README notes a fix).

Course project for EEN-351 (Artificial Neural Networks), IIT Roorkee, November 2022.

A ball rests on a flat plate whose tilt about two axes (angles alpha and beta) is set by two servo motors. The goal is to tilt the plate so that the ball tracks a desired trajectory on the plate surface, in this case a Lissajous curve. This repository contains MATLAB simulations of four controllers for that task: a hand-designed Mamdani fuzzy controller, a neuro-fuzzy controller whose consequent parameters are learned by backpropagation, a classical PID controller, and a combined neuro-fuzzy plus PID controller. The full write-up is in [reports/Ball_n_Plate_NeuroFuzzy.pdf](reports/Ball_n_Plate_NeuroFuzzy.pdf).

## Approach

### Plant model

The equations of motion are derived with the Euler-Lagrange method using the kinetic energy of the ball and plate and the gravitational potential of the ball. The state is the ball position and velocity along x and y. The model is linearised by assuming the plate angular rates are approximately zero, which gives a double integrator in each axis driven by the corresponding plate angle, with gain 7 (the value of g divided by the rolling factor 1 plus I over m r squared, which is 7/5 for a solid ball). In the scripts this appears as `A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0]`, `B = [0 0; 7 0; 0 0; 0 7]`, integrated with forward Euler at a 0.01 s step. The x and y axes are controlled independently, with alpha driving x and beta driving y.

### Fuzzy controller

A Mamdani fuzzy inference system (`PlateAngles.fis`) takes four inputs, the position error and velocity error in each axis, and outputs the two plate angles. Each input has five Gaussian membership functions (Large Negative, Negative, Zero, Positive, Large Positive) and each output has seven trapezoidal membership functions (Major Negative through Major Positive). The rule base has 50 rules written by hand, 25 per axis, covering the 5 by 5 combinations of position error and velocity error. Inference uses min composition and centroid defuzzification.

### Neuro-fuzzy controller

The neuro-fuzzy controller is a Takagi-Sugeno style network built on the same five Gaussian membership functions per input. For each axis, the 25 products of the position-error and velocity-error membership values are normalised to form a firing-strength vector W (`NeuroFuzzy.m`). Each rule has a linear consequent `f_i = p_i1 + p_i2 * e + p_i3 * e_dot`, so the consequent parameters form a 25 by 3 matrix P per axis. The control angle is the weighted sum of consequents, passed through a tansig nonlinearity in the standalone neuro-fuzzy version. P is initialised randomly (seeded) and trained online by gradient descent with momentum on a squared-error loss over position and velocity error, with the plant simulated in the loop (`train_fuzzy.m`). Training runs on a figure-eight Lissajous trajectory from a non-zero initial state; testing runs from the origin.

### PID baseline and the combined controller

The PID baseline (`just_pid.m`) uses proportional action on position error and velocity error (Kp = 10 on both) with the integral gain set to zero in the committed script. The neuro-fuzzy controller on its own leaves a steady-state error, so the final controller (`test_fuzzy_pid.m`) runs the neuro-fuzzy output and a PID output in parallel and blends them as `alpha = k * alpha_nf + (2 - k) * alpha_pid` with k = 0.7. In this variant the neuro-fuzzy output is linear (no tansig) and training uses `train_fuzzy_pid.m`.

## Results

All numbers are from the report (Fig. 19, RMS error over the simulated run). The first four rows are on the figure-eight trajectory; the last two rows are on a second, more complex Lissajous trajectory used to check robustness.

| Controller | RMS error (position) | RMS error (velocity) |
| --- | --- | --- |
| Fuzzy | 0.0012 | 0.0054 |
| Neuro-fuzzy | 0.0050 | 0.0042 |
| PID | 0.0020 | 0.0021 |
| Neuro-fuzzy + PID | 0.0020 | 0.0020 |
| PID (complex trajectory) | 0.0022 | 0.0027 |
| Neuro-fuzzy + PID (complex trajectory) | 0.0021 | 0.0024 |

Qualitative observations from the report:

- The fuzzy controller settles quickly with little steady-state error. Control inputs are of the order 0.1 rad with a peak of 0.32 rad at the first instant.
- The neuro-fuzzy controller on its own has a visible steady-state error but needs much less control effort, of the order 0.005 rad with a peak of 0.03 rad.
- The PID controller reduces both settling time and steady-state error compared with neuro-fuzzy alone, at the cost of larger control inputs.
- The combined neuro-fuzzy plus PID controller drives the steady-state error to zero and gives the lowest RMS errors on the complex trajectory, which is the basis for the report's conclusion that it is the best of the four.

The report does not give settling time or overshoot as numbers, only the plots.

## Script guide

Run everything from the repository root after adding the source folders to the path:

```matlab
addpath(genpath('matlab'))
```

Each test script trains its own parameter matrices by calling a training function, simulates the plant on the test trajectory, prints `rmse_p` and `rmse_v`, and plots the actual versus desired trajectory and the control angles over time. The desired trajectory is defined in a local function `desiredtrajpoint` at the bottom of each script; the committed scripts do not all use the same one, so edit that function if you want a like-for-like comparison.

| File | What it does |
| --- | --- |
| `matlab/controllers/PlateAngles.fis` | The hand-designed Mamdani fuzzy inference system: 4 inputs, 2 outputs, 50 rules. Open with `fuzzyLogicDesigner` or `readfis`. |
| `matlab/controllers/NeuroFuzzy.m` | Function. Given a position error and velocity error, returns the 25 normalised rule firing strengths from the Gaussian membership functions. Used by every neuro-fuzzy script. |
| `matlab/train/train_fuzzy.m` | Function. Trains the two 25 by 3 consequent matrices (one per axis) by online gradient descent with momentum, tansig output. Called by `test_fuzzy.m`. |
| `matlab/train/train_fuzzy_pid.m` | Function. Same training loop with a linear (no tansig) output, intended for the combined controller. See the note below. |
| `matlab/test/justFuzzy.m` | Script. Simulates the plant with the fuzzy controller from `PlateAngles.fis` via `evalfis`. No training needed. |
| `matlab/test/just_pid.m` | Script. Simulates the plant with the PID baseline. |
| `matlab/test/test_fuzzy.m` | Script. Trains the neuro-fuzzy controller (calls `train_fuzzy`) and then simulates it on the test trajectory. |
| `matlab/test/test_fuzzy_pid.m` | Script. Trains the neuro-fuzzy controller and simulates the combined neuro-fuzzy plus PID controller with blend weight k = 0.7. |

Suggested order: `justFuzzy.m`, then `just_pid.m`, then `test_fuzzy.m`, then `test_fuzzy_pid.m`. This follows the order of the report.

Note: `just_pid.m` also calls the training function even though the PID controller does not use its output; that call can be removed to save time.

## Repository layout

```
.
|-- matlab/
|   |-- controllers/
|   |   |-- NeuroFuzzy.m          rule firing strengths for the neuro-fuzzy network
|   |   `-- PlateAngles.fis       Mamdani fuzzy inference system
|   |-- train/
|   |   |-- train_fuzzy.m         online training, tansig output
|   |   `-- train_fuzzy_pid.m     online training, linear output
|   `-- test/
|       |-- justFuzzy.m           fuzzy controller simulation
|       |-- just_pid.m            PID baseline simulation
|       |-- test_fuzzy.m          neuro-fuzzy controller simulation
|       `-- test_fuzzy_pid.m      neuro-fuzzy + PID simulation
|-- reports/
|   `-- Ball_n_Plate_NeuroFuzzy.pdf   project report
|-- .gitignore
`-- README.md
```

## Requirements

- MATLAB. The report does not state a version. `justFuzzy.m` uses the `evalfis(fis, input)` argument order introduced in R2018b, so R2018b or later is needed for that script.
- Fuzzy Logic Toolbox, for `readfis` and `evalfis`.
- Deep Learning Toolbox, for `tansig` (used in `train_fuzzy.m` and `test_fuzzy.m`).
- No Simulink models are included. Everything is plain MATLAB scripts with forward Euler integration.

## Report

[reports/Ball_n_Plate_NeuroFuzzy.pdf](reports/Ball_n_Plate_NeuroFuzzy.pdf): "Designing a Neuro-Fuzzy Controller for a Ball and Plate System". Covers the Lagrangian derivation of the plant, the fuzzy and neuro-fuzzy controller design, the combined controller, the result plots, and the RMS error table.

## Group members

Sameer Talwar, Shreshth Mehrotra, Shubham Jha, Vivek Sharma
