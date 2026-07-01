# Game-Piece-Detection

Greenfield WPILib + PhotonVision simulation stack for RGB-D game-piece detection.

## Current Status

- Fresh WPILib Java project scaffolded for 2026.
- PhotonLib vendordep added.
- Simulated drivetrain pose is published to `Field2d`.
- PhotonVision simulation is initialized with a generic front RGB-D camera.
- Day 2 field model added with fuel truth objects, pickup and scoring zones, and a simple waypoint path follower.
- Day 3 vision simulation injects field fuel truth as PhotonVision simulated targets with latency/noise telemetry.
- Day 4 RGB-D simulation produces color/depth-relative fuel observations from the same truth model.
- Day 5 RGB-D viability sweeps publish range, angle, occlusion, latency, noise, FOV, and mounting pass/fail metrics.
- MapleSim is installed through `vendordeps/maple-sim.json` and now owns field physics for fuel, walls, bump obstacles, and robot collision response.
- Fuel can be picked up on robot contact using the simulated intake capture radius.
- Automated tests cover drivetrain motion, fuel pickup/scoring state, path follower output, and a full simple pickup-to-score sim path.

MapleSim physics telemetry keys:
- `SimField/FuelPickedUpThisLoop`
- `SimField/TotalFuelPickups`
- `MapleSim/Enabled`
- `MapleSim/RobotHitObstacle`
- `MapleSim/AvailableFuel`
- `MapleSim/MaxFuelSpeed`

Day 5 viability telemetry keys:
- `Day5/ViabilitySweepReady`
- `Day5/TotalCases`
- `Day5/VisiblePercent`
- `Day5/UsablePercent`
- `Day5/CloseRangeUsablePercent`
- `Day5/BestUsableRangeMeters`
- `Day5/WorstUsableRangeMeters`
- `Day5/AvgUsableDepthErrorMeters`
- `Day5/FirstLimitingCase`
- `Day5/MeetsCloseRangeTarget`
- `Day5/MeetsOverallTarget`

## Commands

This repo is pinned to the WPILib 2026 JDK in `gradle.properties`. If WPILib is installed somewhere else, update `org.gradle.java.home` before building.

```sh
GRADLE_USER_HOME=.gradle/user-home ./gradlew build
GRADLE_USER_HOME=.gradle/user-home ./gradlew simulateJava
```

In the simulator Driver Station, enable Autonomous to run the Day 2 pickup-to-score path. Teleop currently parks the robot until the planned teleop-assist work.

Existing legacy Python files are intentionally not part of the new stack.
