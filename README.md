# Identification of kinematic and dynamic parameters of a manipulator using a data-driven approach


This repository contains the MATLAB/Simulink files needed to reproduce the results of the paper *"Identification of kinematic and dynamic parameters of a manipulator using a data-driven approach"*.

A 3-DoF serial manipulator (built from Dynamixel AX-12A servos) is simulated in Simscape Multibody. The simulated joint positions, velocities, accelerations and torques are then fed to the **SINDy** (Sparse Identification of Nonlinear Dynamics) algorithm, which identifies a sparse model of the robot's kinematics and dynamics and compares it against the simulation.

---

## Contents

- [Requirements](#requirements)
- [Repository layout](#repository-layout)
- [Setup](#setup)
- [Running the identification](#running-the-identification)
- [Tuning the identification](#tuning-the-identification)
- [Robot model and CAD files](#robot-model-and-cad-files)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Requirements

- **MATLAB and Simulink R2023a** or later
- [Simscape Multibody](https://la.mathworks.com/products/simscape-multibody.html)
- [Robotics System Toolbox](https://la.mathworks.com/products/robotics.html)
- The **SINDy MATLAB code base** (not included here, see [Setup](#setup))

Optional, only if you want to re-import the robot from CAD:

- [Simscape Multibody Link plugin](https://la.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html) (for SolidWorks/STEP), or
- [Onshape Import](https://la.mathworks.com/help/sm/ug/onshape-import.html) (for the Onshape model)

---

## Repository layout

| File | Description |
|------|-------------|
| `serial3_dataSim.m` | **Main entry point.** Loads the robot model and parameters, runs the Simulink simulation, then calls `serial3_sindy.m`. |
| `serial3_sindy.m` | Builds the SINDy function libraries from the simulation data, runs sparse regression, and plots identified vs. simulated joint positions and torques. |
| `serial3_simulink.slx` | Simulink / Simscape Multibody model of the manipulator that generates the training data. |
| `serial3_v3.mat` | `rigidBodyTree` model of the manipulator (variable `serial3_v3`). |
| `Ensamble_SWML_DataFile.m` | Parameter file (`smiData`) exported by `smimport` from the CAD assembly: rigid transforms, masses, inertias, etc. |
| `*.STEP` | CAD geometry of each robot part (AX-12A servos, brackets, base, tube). Used by the Simscape model for visualization. |
| `serial3_simulink.slxc`, `slprj/` | Simulink cache / build artifacts. Safe to delete; MATLAB regenerates them. |

---

## Setup

1. **Clone this repository**

   ```bash
   git clone <this-repo-url>
   cd dataidrobot
   ```

2. **Add the SINDy code base.** Download the MATLAB SINDy code from
   [faculty.washington.edu/kutz/page26](https://faculty.washington.edu/kutz/page26/)
   and copy its files (at least `sparsifyDynamics.m`) into this folder, or add its folder to the MATLAB path:

   ```matlab
   addpath(genpath('path/to/sindy'))
   ```

3. **Open MATLAB in the repository folder** so that the model, the `.mat` file and the STEP files are on the path.

---

## Running the identification

From the MATLAB command window, in the repository folder:

```matlab
serial3_dataSim
```

The script will:

1. Load the `rigidBodyTree` model (`serial3_v3.mat`) and the Simscape parameters (`Ensamble_SWML_DataFile.m`).
2. Set gravity to `[0 0 -9.81]` and define the test configurations (including `config1 = [1.322; 1.4169; 1.2113]`, the configuration used in the paper).
3. Ask you to pick a Simulink model. **Select `serial3_simulink.slx`.**
4. Update and run the simulation.
5. Run `serial3_sindy.m`, which:
   - builds the function libraries `Theta_qs` (for joint positions) and `Theta_qs2` (for joint torques),
   - computes the sparse coefficient matrices `Xis` and `Xis2` with `sparsifyDynamics`,
   - opens **6 figures** comparing SINDy predictions (dashed) with the simulation:
     - Figures 1–3: joint positions θ₁, θ₂, θ₃
     - Figures 4–6: joint torques τ₁, τ₂, τ₃

The identified coefficients (`Xis`, `Xis2`) are printed in the command window and stay in the workspace for further analysis.

You can also run the steps separately: run the Simulink model from its own window (this creates the `out` variable in the workspace), then run `serial3_sindy`.

---

## Tuning the identification

The main parameters are at the top of the SINDy section in `serial3_sindy.m`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `lambda` | `1e-3` | Sparsification threshold. Higher values give sparser (simpler) models; lower values keep more terms. Joint 1 torque is of order `1e-3`, so its coefficients are easily thresholded to zero. |
| `useRichTorqueLibrary` | `false` | Set in `serial3_dataSim.m`. `false` uses the linear torque library `[1 q̈ q̇ q]` (10 terms). `true` adds sin/cos(q), sin/cos(q)·q̈ and sin/cos(q₂+q₃) (24 terms) and lowers the torque threshold to `lambdaTau = 1e-5`. |
| `n` | `3` | Number of target columns (joints) that `sparsifyDynamics` refits after thresholding. |
| `polyorder`, `usesine` | `4`, `0` | Polynomial/trigonometric library options. The libraries are currently built by hand in `Theta_qs` / `Theta_qs2`, so edit those to try other candidate functions. |

The time vector `tt` is taken from the simulation output, so it always matches the number of samples.

### Trajectory

The excitation trajectory is defined in `serial3_dataSim.m` and used by the *Polynomial Trajectory* block of the Simulink model (variables `trajWaypoints`, `trajTimePoints`, `trajVelBC`, `trajAccBC`):

| `useViaPoints` | Trajectory | Notes |
|---|---|---|
| `false` | One quintic segment from the home configuration to `config1` (0–10 s). | Trajectory used in the paper. All joints share the same time profile, so the torque library `Theta_qs2` is rank deficient (MATLAB warns "Rank deficient"). |
| `true` (default) | Home → 3 intermediate configurations → `config1`, every 2.5 s. | Joints move independently and `Theta_qs2` has full rank. Edit the intermediate configurations in `serial3_dataSim.m`; keep them within the ±π/2 joint limits. |

---

## Robot model and CAD files

- **URDF, meshes and config:** [serial3_robot_v3 repository](https://github.com/johncgh22/serial3_robot_v3.git)
- **Onshape CAD assembly:** [Serial Manipulator CAD files](https://cad.onshape.com/documents/24177e1622b7fec708ecbcbd/w/95ff83bf90adcb17a6e9096b/e/46fdcad5c9582ce624ec01ee?renderMode=0&uiState=69efbc494fe06c90d68c7818)

### Re-importing the model into Simscape Multibody

- **SolidWorks / STEP:** install the [Simscape Multibody Link plugin](https://la.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html), export the assembly to XML and import it with `smimport`.
- **Onshape:** use [Onshape Import](https://la.mathworks.com/help/sm/ug/onshape-import.html). See [this example](https://la.mathworks.com/help/sm/ug/export-a-model-from-onshape-software.html) for a step-by-step guide.

If you generate your own import, replace `Ensamble_SWML_DataFile.m` with the data file produced by your import (it is called from `serial3_dataSim.m`).

---

## Troubleshooting

- **`Undefined function 'sparsifyDynamics'`**: the SINDy code base is not on the MATLAB path. See [Setup](#setup), step 2.
- **`Unrecognized function or variable 'out'`**: `serial3_sindy.m` reads the simulation results from the `out` variable. `serial3_dataSim.m` creates it automatically. If you run `serial3_sindy.m` on its own, run the Simulink model first (from its window, or with `out = sim('serial3_simulink');`).
- **`Warning: Rank deficient`** from `sparsifyDynamics`: the joint signals are linearly dependent. This is expected with `useViaPoints = false` (see [Trajectory](#trajectory)).
- **Missing geometry in the Mechanics Explorer**: make sure MATLAB's current folder is the repository root so the `.STEP` files can be found.
- **Stale build or cache errors**: delete `slprj/` and `serial3_simulink.slxc` and run again.

---

## License

This project is released under the [MIT License](LICENSE).

**Author:** Juan C. García-Hernández
