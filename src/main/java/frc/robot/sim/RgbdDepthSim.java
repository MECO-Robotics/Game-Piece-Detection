package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

public class RgbdDepthSim {
  private static final String kFuelFormat =
      "id=%d,visible=%b,truthRange=%.3f,depthRange=%.3f,colorYaw=%.2f,colorPitch=%.2f,"
          + "cameraX=%.3f,cameraY=%.3f,cameraZ=%.3f,depthConf=%.2f,colorConf=%.2f,aligned=%b";

  private final Random random = new Random(Constants.Vision.kDepthNoiseSeed);
  private double lastDepthUpdateSeconds = -Constants.Vision.kSimUpdatePeriodSeconds;
  private FuelDepthObservation lastVisibleFuel;

  public void update(Pose2d robotPose, List<Pose2d> fuelPoses) {
    double nowSeconds = RobotController.getFPGATime() / 1_000_000.0;
    SmartDashboard.putNumber("Vision/DepthAvailableFuelTargets", fuelPoses.size());

    if (nowSeconds - lastDepthUpdateSeconds < Constants.Vision.kSimUpdatePeriodSeconds) {
      SmartDashboard.putBoolean("Vision/DepthUpdateSkippedForThrottle", true);
      SmartDashboard.putBoolean("Vision/DepthUpdateRanThisLoop", false);
      return;
    }

    lastDepthUpdateSeconds = nowSeconds;
    SmartDashboard.putBoolean("Vision/DepthUpdateSkippedForThrottle", false);
    SmartDashboard.putBoolean("Vision/DepthUpdateRanThisLoop", true);

    Pose3d cameraPose = robotPoseToPose3d(robotPose).transformBy(Constants.Vision.kRobotToCamera);
    List<FuelDepthObservation> observations = new ArrayList<>();

    for (int i = 0; i < fuelPoses.size(); i++) {
      observations.add(createObservation(i + 1, cameraPose, fuelPoses.get(i)));
    }

    publishDepthTelemetry(observations);
  }

  private FuelDepthObservation createObservation(
      int fuelId, Pose3d cameraPose, Pose2d fuelPose) {
    Pose3d fuelPose3d =
        new Pose3d(
            fuelPose.getX(),
            fuelPose.getY(),
            Constants.Vision.kFuelTargetHeightMeters,
            new Rotation3d(0.0, 0.0, fuelPose.getRotation().getRadians()));
    Pose3d targetInCameraFrame = fuelPose3d.relativeTo(cameraPose);
    Translation3d delta = targetInCameraFrame.getTranslation();

    double forward = -delta.getX();
    double left = -delta.getY();
    double up = delta.getZ();
    double range = delta.getNorm();
    double yawDeg = Math.toDegrees(Math.atan2(left, forward));
    double pitchDeg = Math.toDegrees(Math.atan2(up, Math.hypot(left, forward)));

    double horizontalFovHalf = Math.toRadians(Constants.Vision.kDepthHorizontalFovDegrees) / 2.0;
    double verticalFovHalf =
        Math.atan(
            Math.tan(horizontalFovHalf)
                * ((double) Constants.Vision.kCameraHeight / Constants.Vision.kCameraWidth));
    boolean inDepthRange =
        range >= Constants.Vision.kDepthMinRangeMeters
            && range <= Constants.Vision.kDepthMaxRangeMeters;
    boolean inFov =
        forward > 0.0
            && Math.abs(Math.toRadians(yawDeg)) <= horizontalFovHalf
            && Math.abs(Math.toRadians(pitchDeg)) <= verticalFovHalf;

    boolean dropout = random.nextDouble() < Constants.Vision.kDepthDropoutRate;
    boolean visible = inDepthRange && inFov && !dropout && range > 1e-6;
    String rejectionReason = rejectionReason(forward, inDepthRange, inFov, dropout, range);
    if (!visible) {
      return new FuelDepthObservation(
          fuelId,
          false,
          yawDeg,
          pitchDeg,
          range,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          false,
          rejectionReason);
    }

    double noisedRange = range + random.nextGaussian() * Constants.Vision.kDepthNoiseStdDevMeters;
    noisedRange =
        Math.max(
            Constants.Vision.kDepthMinRangeMeters,
            Math.min(noisedRange, Constants.Vision.kDepthMaxRangeMeters));
    double pitchRad = Math.toRadians(pitchDeg);
    double yawRad = Math.toRadians(yawDeg);
    double depthX = noisedRange * Math.cos(pitchRad) * Math.cos(yawRad);
    double depthY = noisedRange * Math.cos(pitchRad) * Math.sin(yawRad);
    double depthZ = noisedRange * Math.sin(pitchRad);

    return new FuelDepthObservation(
        fuelId,
        true,
        yawDeg,
        pitchDeg,
        range,
        noisedRange,
        depthX,
        depthY,
        depthZ,
        Constants.Vision.kFuelTargetConfidence,
        1.0,
        true,
        "visible");
  }

  private void publishDepthTelemetry(List<FuelDepthObservation> observations) {
    SmartDashboard.putBoolean("Vision/DepthHasTargets", hasVisibleTargets(observations));
    SmartDashboard.putBoolean("Vision/DepthAllTargets", !observations.isEmpty());
    SmartDashboard.putBoolean("Vision/DepthConfigFov", true);
    SmartDashboard.putString("Vision/DepthCameraForwardAxis", "-X");
    SmartDashboard.putString(
        "Vision/DepthConfiguredRangeMeters",
        String.format(
            Locale.US,
            "%.2f-%.2f",
            Constants.Vision.kDepthMinRangeMeters,
            Constants.Vision.kDepthMaxRangeMeters));

    List<FuelDepthObservation> visibleFuel =
        observations.stream().filter(obs -> obs.visible).toList();
    SmartDashboard.putNumber("Vision/DepthVisibleFuelTargets", visibleFuel.size());

    if (visibleFuel.isEmpty()) {
      SmartDashboard.putNumber("Vision/DepthVisiblePercent", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestDepthRange", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestColorYawDeg", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestColorPitchDeg", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestCameraToTargetX", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestCameraToTargetY", 0.0);
      SmartDashboard.putNumber("Vision/DepthBestCameraToTargetZ", 0.0);
      SmartDashboard.putString("Vision/DepthObservationSummary", "none");
      publishRejectTelemetry(observations);
      SmartDashboard.putString(
          "Vision/DepthAllFuelSummary",
          String.join("|", observations.stream().map(this::formatSummaryOrMissing).toList()));
      publishLastSeenTelemetry();
      return;
    }

    FuelDepthObservation bestFuel = visibleFuel.get(0);
    for (int i = 1; i < visibleFuel.size(); i++) {
      if (visibleFuel.get(i).noisedRangeMeters < bestFuel.noisedRangeMeters) {
        bestFuel = visibleFuel.get(i);
      }
    }

    SmartDashboard.putNumber(
        "Vision/DepthVisiblePercent",
        ((double) visibleFuel.size()) / Math.max(1.0, observations.size()));
    SmartDashboard.putNumber("Vision/DepthBestDepthRange", bestFuel.noisedRangeMeters);
    SmartDashboard.putNumber("Vision/DepthBestColorYawDeg", bestFuel.colorYawDegrees);
    SmartDashboard.putNumber("Vision/DepthBestColorPitchDeg", bestFuel.colorPitchDegrees);
    SmartDashboard.putNumber("Vision/DepthBestCameraToTargetX", bestFuel.cameraToTargetX);
    SmartDashboard.putNumber("Vision/DepthBestCameraToTargetY", bestFuel.cameraToTargetY);
    SmartDashboard.putNumber("Vision/DepthBestCameraToTargetZ", bestFuel.cameraToTargetZ);
    SmartDashboard.putString("Vision/DepthCurrentRejectReason", "visible");
    SmartDashboard.putString("Vision/DepthRejectReason", "visible");
    SmartDashboard.putNumber("Vision/DepthRejectClosestId", bestFuel.fuelId);
    SmartDashboard.putNumber("Vision/DepthRejectClosestRange", bestFuel.rangeMeters);
    SmartDashboard.putNumber("Vision/DepthRejectClosestYawDeg", bestFuel.colorYawDegrees);
    SmartDashboard.putNumber("Vision/DepthRejectClosestPitchDeg", bestFuel.colorPitchDegrees);
    lastVisibleFuel = bestFuel;
    publishLastSeenTelemetry();

    List<String> visibleSummaries = new ArrayList<>();
    for (FuelDepthObservation observation : visibleFuel) {
      visibleSummaries.add(formatSummary(observation));
    }
    SmartDashboard.putString("Vision/DepthVisibleFuelSummary", String.join(";", visibleSummaries));
    SmartDashboard.putNumber("Vision/DepthTargetCount", observations.size());
    SmartDashboard.putString("Vision/DepthObservationSummary", formatSummary(bestFuel));
    SmartDashboard.putString(
        "Vision/DepthAllFuelSummary",
        String.join("|", observations.stream().map(this::formatSummaryOrMissing).toList()));
  }

  private boolean hasVisibleTargets(List<FuelDepthObservation> observations) {
    return observations.stream().anyMatch(obs -> obs.visible);
  }

  private String rejectionReason(
      double forward, boolean inDepthRange, boolean inFov, boolean dropout, double range) {
    if (range <= 1e-6) {
      return "same-position";
    }
    if (forward <= 0.0) {
      return "behind-camera";
    }
    if (!inDepthRange) {
      return "outside-depth-range";
    }
    if (!inFov) {
      return "outside-fov";
    }
    if (dropout) {
      return "depth-dropout";
    }
    return "visible";
  }

  private void publishRejectTelemetry(List<FuelDepthObservation> observations) {
    if (observations.isEmpty()) {
      SmartDashboard.putString("Vision/DepthCurrentRejectReason", "no-available-fuel");
      SmartDashboard.putString("Vision/DepthRejectReason", "no-available-fuel");
      SmartDashboard.putNumber("Vision/DepthRejectClosestId", -1.0);
      SmartDashboard.putNumber("Vision/DepthRejectClosestRange", 0.0);
      SmartDashboard.putNumber("Vision/DepthRejectClosestYawDeg", 0.0);
      SmartDashboard.putNumber("Vision/DepthRejectClosestPitchDeg", 0.0);
      return;
    }

    FuelDepthObservation closest = observations.get(0);
    for (int i = 1; i < observations.size(); i++) {
      if (observations.get(i).rangeMeters < closest.rangeMeters) {
        closest = observations.get(i);
      }
    }

    SmartDashboard.putString("Vision/DepthCurrentRejectReason", closest.rejectionReason);
    SmartDashboard.putString("Vision/DepthRejectReason", closest.rejectionReason);
    SmartDashboard.putNumber("Vision/DepthRejectClosestId", closest.fuelId);
    SmartDashboard.putNumber("Vision/DepthRejectClosestRange", closest.rangeMeters);
    SmartDashboard.putNumber("Vision/DepthRejectClosestYawDeg", closest.colorYawDegrees);
    SmartDashboard.putNumber("Vision/DepthRejectClosestPitchDeg", closest.colorPitchDegrees);
  }

  private void publishLastSeenTelemetry() {
    SmartDashboard.putBoolean("Vision/DepthHasLastSeenTarget", lastVisibleFuel != null);
    if (lastVisibleFuel == null) {
      SmartDashboard.putNumber("Vision/DepthLastSeenDepthRange", 0.0);
      SmartDashboard.putNumber("Vision/DepthLastSeenColorYawDeg", 0.0);
      SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetX", 0.0);
      SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetY", 0.0);
      SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetZ", 0.0);
      SmartDashboard.putString("Vision/DepthLastSeenSummary", "none");
      return;
    }

    SmartDashboard.putNumber("Vision/DepthLastSeenDepthRange", lastVisibleFuel.noisedRangeMeters);
    SmartDashboard.putNumber("Vision/DepthLastSeenColorYawDeg", lastVisibleFuel.colorYawDegrees);
    SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetX", lastVisibleFuel.cameraToTargetX);
    SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetY", lastVisibleFuel.cameraToTargetY);
    SmartDashboard.putNumber("Vision/DepthLastSeenCameraToTargetZ", lastVisibleFuel.cameraToTargetZ);
    SmartDashboard.putString("Vision/DepthLastSeenSummary", formatSummary(lastVisibleFuel));
  }

  private String formatSummary(FuelDepthObservation obs) {
    return String.format(
        Locale.US,
        kFuelFormat,
        obs.fuelId,
        obs.visible,
        obs.rangeMeters,
        obs.noisedRangeMeters,
        obs.colorYawDegrees,
        obs.colorPitchDegrees,
        obs.cameraToTargetX,
        obs.cameraToTargetY,
        obs.cameraToTargetZ,
        obs.depthConfidence,
        obs.colorConfidence,
        obs.synced);
  }

  private String formatSummaryOrMissing(FuelDepthObservation obs) {
    return obs.visible
        ? formatSummary(obs)
        : String.format(
            Locale.US,
            "id=%d,visible=false,reason=%s,truthRange=%.3f,yaw=%.2f,pitch=%.2f",
            obs.fuelId,
            obs.rejectionReason,
            obs.rangeMeters,
            obs.colorYawDegrees,
            obs.colorPitchDegrees);
  }

  private Pose3d robotPoseToPose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        0.0,
        new Rotation3d(
            0.0,
            0.0,
            pose.getRotation().getRadians()));
  }

  private static final class FuelDepthObservation {
    private final int fuelId;
    private final boolean visible;
    private final double colorYawDegrees;
    private final double colorPitchDegrees;
    private final double rangeMeters;
    private final double noisedRangeMeters;
    private final double cameraToTargetX;
    private final double cameraToTargetY;
    private final double cameraToTargetZ;
    private final double colorConfidence;
    private final double depthConfidence;
    private final boolean synced;
    private final String rejectionReason;

    private FuelDepthObservation(
        int fuelId,
        boolean visible,
        double colorYawDegrees,
        double colorPitchDegrees,
        double rangeMeters,
        double noisedRangeMeters,
        double cameraToTargetX,
        double cameraToTargetY,
        double cameraToTargetZ,
        double colorConfidence,
        double depthConfidence,
        boolean synced,
        String rejectionReason) {
      this.fuelId = fuelId;
      this.visible = visible;
      this.colorYawDegrees = colorYawDegrees;
      this.colorPitchDegrees = colorPitchDegrees;
      this.rangeMeters = rangeMeters;
      this.noisedRangeMeters = noisedRangeMeters;
      this.cameraToTargetX = cameraToTargetX;
      this.cameraToTargetY = cameraToTargetY;
      this.cameraToTargetZ = cameraToTargetZ;
      this.colorConfidence = colorConfidence;
      this.depthConfidence = depthConfidence;
      this.synced = synced;
      this.rejectionReason = rejectionReason;
    }
  }
}
