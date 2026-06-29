package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public record FieldZone(String name, Pose2d center, double lengthMeters, double widthMeters) {
  public boolean contains(Pose2d pose) {
    double dx = Math.abs(pose.getX() - center.getX());
    double dy = Math.abs(pose.getY() - center.getY());
    return dx <= lengthMeters / 2.0 && dy <= widthMeters / 2.0;
  }

  public List<Pose2d> markerPoses() {
    double halfLength = lengthMeters / 2.0;
    double halfWidth = widthMeters / 2.0;
    return List.of(
        new Pose2d(center.getX() - halfLength, center.getY() - halfWidth, Rotation2d.kZero),
        new Pose2d(center.getX() + halfLength, center.getY() - halfWidth, Rotation2d.kZero),
        new Pose2d(center.getX() + halfLength, center.getY() + halfWidth, Rotation2d.kZero),
        new Pose2d(center.getX() - halfLength, center.getY() + halfWidth, Rotation2d.kZero),
        center);
  }
}
