# Game-Piece-Detection

Greenfield WPILib + PhotonVision simulation stack for RGB-D game-piece detection.

## Current Status

- Fresh WPILib Java project scaffolded for 2026.
- PhotonLib vendordep added.
- Simulated drivetrain pose is published to `Field2d`.
- PhotonVision simulation is initialized with a generic front RGB-D camera.
- Day 2 field model added with fuel truth objects, pickup and scoring zones, and a simple waypoint path follower.
- Day 3 vision simulation now injects field fuel truth as PhotonVision simulated targets with camera latency/noise properties and publishes target telemetry to SmartDashboard.
- Day 4 adds simulated RGB-D sync layer producing relative fuel 3D positions from the same truth model using intrinsics/extrinsics, min/max range, dropout, and depth noise.
- Day 4 depth telemetry now keeps last-seen target values and publishes the current rejection reason when no fuel is visible.
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
Day 4 telemetry keys now exposed:
- `Vision/DepthConfiguredRangeMeters`
- `Vision/DepthAvailableFuelTargets`
- `Vision/DepthVisibleFuelTargets`
- `Vision/DepthVisiblePercent`
- `Vision/DepthHasTargets`
- `Vision/DepthAllTargets`
- `Vision/DepthConfigFov`
- `Vision/DepthBestDepthRange`
- `Vision/DepthBestColorYawDeg`
- `Vision/DepthBestColorPitchDeg`
- `Vision/DepthBestCameraToTargetX`
- `Vision/DepthBestCameraToTargetY`
- `Vision/DepthBestCameraToTargetZ`
- `Vision/DepthObservationSummary`
- `Vision/DepthVisibleFuelSummary`
- `Vision/DepthAllFuelSummary`
- `Vision/DepthTargetCount`
- `Vision/DepthUpdateSkippedForThrottle`
- `Vision/DepthUpdateRanThisLoop`
- `Vision/DepthCameraForwardAxis`
- `Vision/DepthCurrentRejectReason`
- `Vision/DepthRejectReason`
- `Vision/DepthRejectClosestId`
- `Vision/DepthRejectClosestRange`
- `Vision/DepthRejectClosestYawDeg`
- `Vision/DepthRejectClosestPitchDeg`
- `Vision/DepthHasLastSeenTarget`
- `Vision/DepthLastSeenDepthRange`
- `Vision/DepthLastSeenColorYawDeg`
- `Vision/DepthLastSeenCameraToTargetX`
- `Vision/DepthLastSeenCameraToTargetY`
- `Vision/DepthLastSeenCameraToTargetZ`
- `Vision/DepthLastSeenSummary`

## Commands

This repo is pinned to the WPILib 2026 JDK in `gradle.properties`. If WPILib is installed somewhere else, update `org.gradle.java.home` before building.

```sh
GRADLE_USER_HOME=.gradle/user-home ./gradlew build
GRADLE_USER_HOME=.gradle/user-home ./gradlew simulateJava
```

In the simulator Driver Station, enable Autonomous to run the Day 2 pickup-to-score path. Teleop currently parks the robot until the planned teleop-assist work.

Day 4 adds synchronized simulated depth outputs (with depth noise, dropout, and range/FOV filtering) from the same truth fuel targets used by PhotonVision simulation.
