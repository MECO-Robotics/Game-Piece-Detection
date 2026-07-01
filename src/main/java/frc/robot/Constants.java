package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

public final class Constants {
  private Constants() {}

  public static final class Drivetrain {
    public static final double kMaxLinearSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kStartingX = 1.0;
    public static final double kStartingY = 1.0;
    public static final Rotation2d kStartingHeading = Rotation2d.fromDegrees(0.0);

    public static Pose2d kStartingPose() {
      return new Pose2d(kStartingX, kStartingY, kStartingHeading);
    }

    private Drivetrain() {}
  }

  public static final class Field {
    public static final double kLengthMeters = 16.54;
    public static final double kWidthMeters = 8.21;
    public static final double kPickupRadiusMeters = 0.45;
    public static final double kWaypointToleranceMeters = 0.18;
    public static final double kScoringZoneToleranceMeters = 0.35;

    public static final Pose2d kPickupZoneCenter =
        new Pose2d(2.25, 1.65, Rotation2d.fromDegrees(0.0));
    public static final double kPickupZoneLengthMeters = 2.5;
    public static final double kPickupZoneWidthMeters = 2.0;

    public static final Pose2d kScoringZoneCenter =
        new Pose2d(7.35, 3.9, Rotation2d.fromDegrees(180.0));
    public static final double kScoringZoneLengthMeters = 1.4;
    public static final double kScoringZoneWidthMeters = 1.6;

    public static final Pose2d kBumpCenter =
        new Pose2d(kLengthMeters / 2.0, kWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kBumpLengthMeters = kLengthMeters - 1.0;
    public static final double kBumpWidthMeters = 0.18;

    public static final List<Pose2d> kStartingFuelPoses =
        List.of(
            new Pose2d(2.35, 1.55, Rotation2d.fromDegrees(0.0)),
            new Pose2d(2.75, 2.15, Rotation2d.fromDegrees(0.0)),
            new Pose2d(3.2, 1.2, Rotation2d.fromDegrees(0.0)),
            new Pose2d(4.15, 3.6, Rotation2d.fromDegrees(0.0)));

    private Field() {}
  }

  public static final class Vision {
    public static final String kPhotonCameraName = "front-rgbd-sim";
    public static final int kCameraWidth = 640;
    public static final int kCameraHeight = 480;
    public static final double kHorizontalFovDegrees = 70.0;
    public static final double kNominalFps = 30.0;
    public static final double kSimUpdatePeriodSeconds = 1.0 / kNominalFps;
    public static final double kAverageLatencyMs = 30.0;
    public static final double kLatencyStdDevMs = 5.0;
    public static final double kCalibXErrorPixels = 0.3;
    public static final double kCalibYErrorPixels = 0.3;
    public static final long kSimNoiseSeed = 3026L;
    public static final double kMinFuelTargetAreaPixels = 4.0;
    public static final double kMaxFuelSightRangeMeters = 8.0;
    public static final float kFuelTargetConfidence = 0.98f;
    public static final double kFuelTargetRadiusMeters = 0.076;
    public static final double kFuelTargetDiameterMeters = 2.0 * kFuelTargetRadiusMeters;
    public static final int kFuelDetectionClass = 1;
    public static final double kFuelTargetHeightMeters = 0.08;
    public static final double kDepthHorizontalFovDegrees = 95.0;
    public static final double kDepthMinRangeMeters = 0.05;
    public static final double kDepthMaxRangeMeters = 8.0;
    public static final double kDepthNoiseStdDevMeters = 0.025;
    public static final double kDepthDropoutRate = 0.0;
    public static final long kDepthNoiseSeed = 8127L;
    public static final Transform3d kRobotToCamera =
        new Transform3d(
            new Translation3d(0.28, 0.0, 0.42),
            new Rotation3d(0.0, Math.toRadians(-12.0), 0.0));

    private Vision() {}
  }

  public static final class Physics {
    public static final double kRobotCollisionRadiusMeters = 0.42;
    public static final double kRobotBumperLengthMeters = 0.84;
    public static final double kRobotBumperWidthMeters = 0.84;
    public static final double kFuelCollisionRadiusMeters = Vision.kFuelTargetRadiusMeters;
    public static final double kCollisionSlopMeters = 0.015;
    public static final double kFuelVelocityTransfer = 0.85;
    public static final double kFuelWallRestitution = 0.35;
    public static final double kFuelFrictionPerSecond = 2.2;
    public static final double kMaxFuelSpeedMetersPerSecond = 4.0;
    public static final double kIntakeCaptureRadiusMeters =
        kRobotCollisionRadiusMeters + kFuelCollisionRadiusMeters + kCollisionSlopMeters;

    private Physics() {}
  }

  public static final class Viability {
    public static final double[] kRangeMeters = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.5, 7.0, 8.0};
    public static final double[] kYawDegrees = {-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0};
    public static final double[] kOcclusionFractions = {0.0, 0.25, 0.5, 0.75};
    public static final double[] kLatencyMs = {0.0, 30.0, 80.0, 120.0};
    public static final double[] kDepthNoiseMeters = {0.0, 0.025, 0.075, 0.15};
    public static final double[] kHorizontalFovDegrees = {60.0, 70.0, 95.0};
    public static final double[] kCameraPitchDegrees = {-20.0, -12.0, 0.0};
    public static final double kMaxUsableDepthErrorMeters = 0.2;
    public static final double kMinUsableConfidence = 0.5;
    public static final double kAssumedApproachSpeedMetersPerSecond = 1.0;

    private Viability() {}
  }
}
