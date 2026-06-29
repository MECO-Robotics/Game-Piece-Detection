package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;

public record SimGamePiece(int id, String type, Pose2d pose) {
  public SimGamePiece withPose(Pose2d newPose) {
    return new SimGamePiece(id, type, newPose);
  }
}
