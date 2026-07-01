package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class WaypointPathFollower {
  private static final double kCruiseSpeedMetersPerSecond =
      Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond;
  private static final double kWaypointArrivalEpsilonMeters = 0.000001;

  public ChassisSpeeds calculate(Pose2d robotPose, Pose2d targetPose) {
    double fieldErrorX = targetPose.getX() - robotPose.getX();
    double fieldErrorY = targetPose.getY() - robotPose.getY();
    double distanceMeters = Math.hypot(fieldErrorX, fieldErrorY);

    double fieldVx = 0.0;
    double fieldVy = 0.0;
    if (distanceMeters > kWaypointArrivalEpsilonMeters) {
      fieldVx = kCruiseSpeedMetersPerSecond * fieldErrorX / distanceMeters;
      fieldVy = kCruiseSpeedMetersPerSecond * fieldErrorY / distanceMeters;
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
