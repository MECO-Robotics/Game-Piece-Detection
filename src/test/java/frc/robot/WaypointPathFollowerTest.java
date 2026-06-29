package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sim.WaypointPathFollower;
import org.junit.jupiter.api.Test;

class WaypointPathFollowerTest {
  @Test
  void calculatesForwardCommandTowardWaypoint() {
    WaypointPathFollower follower = new WaypointPathFollower();
    ChassisSpeeds speeds =
        follower.calculate(
            new Pose2d(1.0, 1.0, Rotation2d.kZero),
            new Pose2d(3.0, 1.0, Rotation2d.kZero));

    assertTrue(speeds.vxMetersPerSecond > 0.0);
    assertTrue(Math.abs(speeds.vyMetersPerSecond) < 1e-9);
  }

  @Test
  void neverCommandsRotationNearFuelOrAcrossField() {
    WaypointPathFollower follower = new WaypointPathFollower();
    ChassisSpeeds closeToFuel =
        follower.calculate(
            new Pose2d(2.10, 1.45, Rotation2d.fromDegrees(20.0)),
            Constants.Field.kStartingFuelPoses.get(0));
    ChassisSpeeds acrossField =
        follower.calculate(
            new Pose2d(2.35, 1.55, Rotation2d.fromDegrees(20.0)),
            Constants.Field.kScoringZoneCenter);

    assertEquals(0.0, closeToFuel.omegaRadiansPerSecond, 1e-9);
    assertEquals(0.0, acrossField.omegaRadiansPerSecond, 1e-9);
  }
}
