package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

public class SimDrivetrain {
  private final Field2d field = new Field2d();
  private final Timer timer = new Timer();

  private Pose2d pose =
      new Pose2d(
          Constants.Drivetrain.kStartingX,
          Constants.Drivetrain.kStartingY,
          Constants.Drivetrain.kStartingHeading);
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private double lastTimestampSeconds;

  public SimDrivetrain() {
    timer.start();
    lastTimestampSeconds = timer.get();
    field.setRobotPose(pose);
  }

  public void drive(
      double xMetersPerSecond, double yMetersPerSecond, double omegaRadiansPerSecond) {
    commandedSpeeds =
        new ChassisSpeeds(
            clamp(
                xMetersPerSecond,
                -Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond,
                Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond),
            clamp(
                yMetersPerSecond,
                -Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond,
                Constants.Drivetrain.kMaxLinearSpeedMetersPerSecond),
            clamp(
                omegaRadiansPerSecond,
                -Constants.Drivetrain.kMaxAngularSpeedRadiansPerSecond,
                Constants.Drivetrain.kMaxAngularSpeedRadiansPerSecond));
  }

  public void stop() {
    commandedSpeeds = new ChassisSpeeds();
  }

  public void resetToStart() {
    pose =
        new Pose2d(
            Constants.Drivetrain.kStartingX,
            Constants.Drivetrain.kStartingY,
            Constants.Drivetrain.kStartingHeading);
    field.setRobotPose(pose);
    lastTimestampSeconds = timer.get();
  }

  public void periodic() {
    double now = timer.get();
    double dtSeconds = Math.max(0.0, Math.min(0.05, now - lastTimestampSeconds));
    lastTimestampSeconds = now;

    Rotation2d heading = pose.getRotation();
    double cos = heading.getCos();
    double sin = heading.getSin();
    double fieldVx = commandedSpeeds.vxMetersPerSecond * cos - commandedSpeeds.vyMetersPerSecond * sin;
    double fieldVy = commandedSpeeds.vxMetersPerSecond * sin + commandedSpeeds.vyMetersPerSecond * cos;

    pose =
        new Pose2d(
            pose.getX() + fieldVx * dtSeconds,
            pose.getY() + fieldVy * dtSeconds,
            heading.plus(Rotation2d.fromRadians(commandedSpeeds.omegaRadiansPerSecond * dtSeconds)));
    field.setRobotPose(pose);
  }

  public Pose2d getPose() {
    return pose;
  }

  public Field2d getField() {
    return field;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
