# Game-Piece-Detection

Greenfield WPILib + PhotonVision simulation stack for RGB-D game-piece detection.

## Current Status

- Fresh WPILib Java project scaffolded for 2026.
- PhotonLib vendordep added.
- Simulated drivetrain pose is published to `Field2d`.
- PhotonVision simulation is initialized with a generic front RGB-D camera.
- Day 2 field model added with fuel truth objects, pickup and scoring zones, and a simple waypoint path follower.
- Day 3 vision simulation now injects field fuel truth as PhotonVision simulated targets with camera latency/noise properties and publishes target telemetry to SmartDashboard.
- Automated tests cover drivetrain motion, fuel pickup/scoring state, path follower output, and a full simple pickup-to-score sim path.

Day 3 telemetry keys now exposed:
- `Vision/VisibleFuelTargets`
- `Vision/HasTargets`
- `Vision/BestFuelYawDeg`
- `Vision/BestFuelPitchDeg`
- `Vision/BestFuelClass`
- `Vision/BestFuelConfidence`
- `Vision/FuelLatencyMs`
- `Vision/BestFuelRangeMeters`
- `Vision/BestFuelCameraToTargetX`
- `Vision/BestFuelCameraToTargetY`
- `Vision/BestFuelCameraToTargetZ`
- `Vision/ConfiguredLatencyStdDevMs`
- `Vision/ConfiguredCalibNoiseXPixels`
- `Vision/ConfiguredCalibNoiseYPixels`
- `Vision/MinFuelTargetAreaPixels`
- `Vision/MaxFuelSightRangeMeters`
- `Vision/UpdateSkippedForThrottle`
- `Vision/UpdateRanThisLoop`
- `Vision/UpdateFault`
- `Vision/UpdateHealthy`
- `Vision/Status`

## Commands

This repo is pinned to the WPILib 2026 JDK in `gradle.properties`. If WPILib is installed somewhere else, update `org.gradle.java.home` before building.

```sh
GRADLE_USER_HOME=.gradle/user-home ./gradlew build
GRADLE_USER_HOME=.gradle/user-home ./gradlew simulateJava
```

In the simulator Driver Station, enable Autonomous to run the Day 2 pickup-to-score path. Teleop currently parks the robot until the planned teleop-assist work.

Day 3 should add PhotonVision/PhotonLib object simulation for fuel detections with latency/noise. Existing legacy Python files are intentionally not part of the new stack.
