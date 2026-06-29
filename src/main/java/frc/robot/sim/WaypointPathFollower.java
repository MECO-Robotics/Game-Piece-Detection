package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class WaypointPathFollower {
  private static final double kTranslationKp = 1.4;

  public ChassisSpeeds calculate(Pose2d robotPose, Pose2d targetPose) {
    double fieldErrorX = targetPose.getX() - robotPose.getX();
    double fieldErrorY = targetPose.getY() - robotPose.getY();
    double distanceMeters = Math.hypot(fieldErrorX, fieldErrorY);

    double fieldVx = 0.0;
    double fieldVy = 0.0;
    if (distanceMeters > 1e-6) {
      double speed =
          Math.min(
              distanceMeters * kTranslationKp,
              Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond);
      fieldVx = speed * fieldErrorX / distanceMeters;
      fieldVy = speed * fieldErrorY / distanceMeters;
    }

    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();
    double robotVx = fieldVx * cos + fieldVy * sin;
    double robotVy = -fieldVx * sin + fieldVy * cos;
    return new ChassisSpeeds(robotVx, robotVy, 0.0);
  }

  public boolean isAtTarget(Pose2d robotPose, Pose2d targetPose) {
    return robotPose.getTranslation().getDistance(targetPose.getTranslation())
        <= Constants.Field.kWaypointToleranceMeters;
  }
}
