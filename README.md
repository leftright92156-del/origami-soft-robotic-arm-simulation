# Origami Soft Robotic Arm: Kinematics & Workspace

A MATLAB GUI for configuring origami-inspired soft robotic modules,
assembling a multi-module arm, commanding pressure-dependent states,
visualizing kinematics, and analyzing end-effector trajectories and
workspace.

## Features

-   **Interactive module definition:** set module ID, skeleton count,
    initial/final gauge pressure, and configuration yaw/pitch.
-   **Assembly and configuration:** combine modules with different
    skeleton counts and interface configurations.
-   **Theoretical pressure mapping:** compute fold angle, configured
    bending angle, and axial length/stroke from the built-in nonlinear
    pressure model.
-   **Pressure-based motion programming:** command intermediate states
    using pressure targets such as `P:0`, `P:-30`, and `P:-70`.
-   **Real-time visualization:** display the assembled robot,
    end-effector trajectory, and top-view projection.
-   **Workspace analysis:** sample configurations using Monte Carlo
    exploration and visualize the resulting workspace.
-   **Plot and data export:** export figures and numerical results using
    the application's built-in controls.
-   **Publication-oriented plotting:** includes figure styling and
    export options for paper-ready graphics.

## Repository contents

``` text
OrigamiSoftRoboticArm_GitHub/
├── OrigamiRobotProMLPAt.m       # Main MATLAB GUI class
├── run_origami_robot.m          # Convenience launcher
├── README.md                    # Project overview and quick start
├── CHANGELOG.md                 # Version notes
├── .gitignore                   # MATLAB and generated-output exclusions
└── docs/
    └── THEORETICAL_MODEL.md     # Pressure mapping model and conventions
```

## Requirements

-   MATLAB Desktop with support for `uifigure`, `uiaxes`, UI controls,
    and modern graphics export.
-   **Recommended:** MATLAB R2022a or newer.
-   No separate model data files are required by the current single-file
    implementation.

The code has been checked as source text, but this package has **not**
been validated by running it in every MATLAB release or operating
system. If an older MATLAB release reports an unsupported UI or graphics
property, use a newer release or adapt the relevant property call.

## Quick start

1.  Download and extract this repository, or clone it:

    ``` bash
    git clone <YOUR_GITHUB_REPOSITORY_URL>
    ```

2.  Open MATLAB and set the Current Folder to the extracted repository
    directory.

3.  Run either command:

    ``` matlab
    app = OrigamiRobotProMLPAt();
    ```

    or

    ``` matlab
    app = run_origami_robot();
    ```

4.  The application window should open. Keep the returned `app` variable
    in the workspace while using the GUI.

## Basic workflow

1.  **Define a module.** Set its ID, skeleton count, initial/final
    pressure, and configuration yaw/pitch.

2.  **Add the module.** The application uses the built-in theoretical
    pressure mapping to determine the angle or axial-length response.

3.  **Build the assembly.** Add modules and configure their relative
    orientations to create the desired robot shape.

4.  **Enter a pressure sequence.** Use pressure targets in the
    motion-sequence editor. For example:

    ``` text
    k+C, P:-70 ; k+A, P:-70;k+B, P:-70;
    k+C, P:-70 ; A, P:0,0.5;k+B, P:-70;
    k+C, P:-70 ; A, P:-30,0.5;k+B, P:-70;
    ```

    The exact module IDs must match the modules in the assembly. See the
    in-app command hints for hold syntax and concurrent commands. A
    pressure command must remain within that module's configured
    pressure range.

5.  **Run the sequence.** The application animates the configuration and
    generates the end-effector trajectory.

6.  **Analyze and export.** Use the workspace, plotting, and export
    controls as needed.

## Built-in theoretical mapping

The nonlinear pressure--folding-angle relationship is based on the
calibrated equivalent-energy model documented in
[`docs/THEORETICAL_MODEL.md`](docs/THEORETICAL_MODEL.md). The same
single-skeleton model is used as the basis for the module-level
response; the configured stack and skeleton count determine the
assembled kinematics and output quantity.

The model is theoretical. Real hardware limits, leakage, hysteresis,
manufacturing variation, and pressure-controller dynamics may cause
deviations from the predicted response.

## Units and conventions

-   Pressure is **gauge pressure**, in kPa. Negative values denote
    vacuum relative to ambient pressure.
-   Displayed angles are in degrees; trigonometric calculations use
    radians.
-   Length, stroke, and workspace coordinates are in millimeters.
-   The built-in inversion enforces a positive fold-angle branch and
    clamps pressure to its implemented numerical range. Do not interpret
    extrapolated theoretical values as verified hardware capability.

## Troubleshooting

-   **The class cannot be found:** make sure `OrigamiRobotProMLPAt.m` is
    in the MATLAB Current Folder or on the MATLAB path.
-   **The GUI does not open:** run the code from MATLAB Desktop, not
    from a non-graphical MATLAB session.
-   **A pressure command is rejected:** confirm that the target pressure
    lies between the module's configured initial and final pressure.
-   **A plot appears clipped on a different display size:** maximize the
    application window and check the display scaling. The GUI uses fixed
    pixel-based panel positions, so unusual display scaling may require
    layout adjustments.

## License

No license is included in this package because the project owner has not
specified one. Before enabling public reuse, choose and add an
appropriate license (for example, MIT or BSD-3-Clause). Without a
license, public visibility on GitHub does not by itself grant permission
to reuse, modify, or redistribute the code.

## Citation

If you use this software in academic work, cite the associated paper or
repository once the final paper and repository URL are available. Add a
`CITATION.cff` file with the final bibliographic details before release
if you want GitHub to display a citation prompt.
