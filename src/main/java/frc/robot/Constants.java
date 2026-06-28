package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  private Constants() {}

  public static final class Drivetrain {
    public static final double kMaxLinearSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kStartingX = 1.0;
    public static final double kStartingY = 1.0;
    public static final Rotation2d kStartingHeading = Rotation2d.fromDegrees(0.0);

    private Drivetrain() {}
  }

  public static final class Vision {
    public static final String kPhotonCameraName = "front-rgbd-sim";
    public static final int kCameraWidth = 640;
    public static final int kCameraHeight = 480;
    public static final double kHorizontalFovDegrees = 70.0;
    public static final double kNominalFps = 30.0;
    public static final double kAverageLatencyMs = 30.0;
    public static final double kLatencyStdDevMs = 5.0;
    public static final Transform3d kRobotToCamera =
        new Transform3d(
            new Translation3d(0.28, 0.0, 0.42),
            new Rotation3d(0.0, Math.toRadians(-12.0), 0.0));

    private Vision() {}
  }
}
