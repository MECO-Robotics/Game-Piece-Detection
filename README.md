# Game-Piece-Detection

Greenfield WPILib + PhotonVision simulation stack for RGB-D game-piece detection.

## Day 1 Status

- Fresh WPILib Java project scaffolded for 2026.
- PhotonLib vendordep added.
- Simulated drivetrain pose is published to `Field2d`.
- PhotonVision simulation is initialized with a generic front RGB-D camera.

## Commands

This repo is pinned to the WPILib 2026 JDK in `gradle.properties`. If WPILib is installed somewhere else, update `org.gradle.java.home` before building.

```sh
GRADLE_USER_HOME=.gradle/user-home ./gradlew build
GRADLE_USER_HOME=.gradle/user-home ./gradlew simulateJava
```

Day 2 should add the field model, game-piece truth objects, pickup/scoring zones, and simple pathing. Existing legacy Python files are intentionally not part of the new stack.
