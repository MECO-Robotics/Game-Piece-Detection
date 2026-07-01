package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;

public record SimGamePiece(
    int id, String type, Pose2d pose, double vxMetersPerSecond, double vyMetersPerSecond) {
  public SimGamePiece(int id, String type, Pose2d pose) {
    this(id, type, pose, 0.0, 0.0);
  }

  public SimGamePiece withPose(Pose2d newPose) {
    return new SimGamePiece(id, type, newPose, vxMetersPerSecond, vyMetersPerSecond);
  }

  public SimGamePiece withVelocity(double vx, double vy) {
    return new SimGamePiece(id, type, pose, vx, vy);
  }

  public SimGamePiece withPoseAndVelocity(Pose2d newPose, double vx, double vy) {
    return new SimGamePiece(id, type, newPose, vx, vy);
  }
}
