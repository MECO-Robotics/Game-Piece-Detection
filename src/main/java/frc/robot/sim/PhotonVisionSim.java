package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSim {
  private static final String kFuelTargetFamily = "fuel";

  private final PhotonCamera camera = new PhotonCamera(Constants.Vision.kPhotonCameraName);
  private final VisionSystemSim visionSystem = new VisionSystemSim("rgbd-photonvision-sim");
  private final PhotonCameraSim cameraSim;
  private final TargetModel fuelTargetModel =
      new TargetModel(Constants.Vision.kFuelTargetDiameterMeters);
  private PhotonPipelineResult latestPhotonResult = new PhotonPipelineResult();
  private double lastVisionUpdateSeconds = -Constants.Vision.kSimUpdatePeriodSeconds;

  public PhotonVisionSim() {
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(
        Constants.Vision.kCameraWidth,
        Constants.Vision.kCameraHeight,
        Rotation2d.fromDegrees(Constants.Vision.kHorizontalFovDegrees));
    cameraProperties.setCalibError(
        Constants.Vision.kCalibXErrorPixels, Constants.Vision.kCalibYErrorPixels);
    cameraProperties.setRandomSeed(Constants.Vision.kSimNoiseSeed);
    cameraProperties.setFPS(Constants.Vision.kNominalFps);
    cameraProperties.setAvgLatencyMs(Constants.Vision.kAverageLatencyMs);
    cameraProperties.setLatencyStdDevMs(Constants.Vision.kLatencyStdDevMs);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    cameraSim.setMinTargetAreaPixels(Constants.Vision.kMinFuelTargetAreaPixels);
    cameraSim.setMaxSightRange(Constants.Vision.kMaxFuelSightRangeMeters);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

    visionSystem.addCamera(cameraSim, Constants.Vision.kRobotToCamera);
    SmartDashboard.putBoolean("Vision/PhotonSimReady", true);
    SmartDashboard.putString("Vision/PhotonCamera", Constants.Vision.kPhotonCameraName);
    SmartDashboard.putString("Vision/TargetModel", "spherical-fuel");
    SmartDashboard.putNumber("Vision/CameraWidthPixels", Constants.Vision.kCameraWidth);
    SmartDashboard.putNumber("Vision/CameraHeightPixels", Constants.Vision.kCameraHeight);
    SmartDashboard.putNumber("Vision/HorizontalFovDeg", Constants.Vision.kHorizontalFovDegrees);
    SmartDashboard.putNumber("Vision/ConfiguredAvgLatencyMs", Constants.Vision.kAverageLatencyMs);
    SmartDashboard.putNumber("Vision/ConfiguredLatencyStdDevMs", Constants.Vision.kLatencyStdDevMs);
    SmartDashboard.putNumber("Vision/ConfiguredCalibNoiseXPixels", Constants.Vision.kCalibXErrorPixels);
    SmartDashboard.putNumber("Vision/ConfiguredCalibNoiseYPixels", Constants.Vision.kCalibYErrorPixels);
    SmartDashboard.putNumber("Vision/MinFuelTargetAreaPixels", Constants.Vision.kMinFuelTargetAreaPixels);
    SmartDashboard.putNumber("Vision/MaxFuelSightRangeMeters", Constants.Vision.kMaxFuelSightRangeMeters);
  }

  public void update(Pose2d robotPose, List<Pose2d> fuelPoses) {
    double nowSeconds = RobotController.getFPGATime() / 1_000_000.0;
    SmartDashboard.putNumber("Vision/AvailableFuelTargets", fuelPoses.size());

    if (nowSeconds - lastVisionUpdateSeconds < Constants.Vision.kSimUpdatePeriodSeconds) {
      SmartDashboard.putBoolean("Vision/UpdateSkippedForThrottle", true);
      SmartDashboard.putBoolean("Vision/UpdateRanThisLoop", false);
      return;
    }

    lastVisionUpdateSeconds = nowSeconds;
    SmartDashboard.putBoolean("Vision/UpdateSkippedForThrottle", false);
    SmartDashboard.putBoolean("Vision/UpdateRanThisLoop", true);
    visionSystem.removeVisionTargets(kFuelTargetFamily);
    visionSystem.addVisionTargets(kFuelTargetFamily, buildFuelTargets(fuelPoses));

    visionSystem.update(robotPose);
    publishLatestVisionResults();
    SmartDashboard.putNumber("Vision/RobotToCameraX", Constants.Vision.kRobotToCamera.getX());
    SmartDashboard.putNumber("Vision/RobotToCameraZ", Constants.Vision.kRobotToCamera.getZ());
  }

  private VisionTargetSim[] buildFuelTargets(List<Pose2d> fuelPoses) {
    List<VisionTargetSim> targets = new ArrayList<>();
    for (int i = 0; i < fuelPoses.size(); i++) {
      Pose2d fuelPose = fuelPoses.get(i);
      VisionTargetSim target =
          new VisionTargetSim(
              toPose3d(fuelPose),
              fuelTargetModel,
              fuelPoseId(i),
              Constants.Vision.kFuelTargetConfidence);
      targets.add(target);
    }
    return targets.toArray(new VisionTargetSim[0]);
  }

  private int fuelPoseId(int index) {
    return index + 1;
  }

  private Pose3d toPose3d(Pose2d pose2d) {
    return new Pose3d(
        pose2d.getX(),
        pose2d.getY(),
        Constants.Vision.kFuelTargetHeightMeters,
        new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));
  }

  private void publishLatestVisionResults() {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
    if (!unreadResults.isEmpty()) {
      latestPhotonResult = unreadResults.get(unreadResults.size() - 1);
    }

    PhotonPipelineResult latestResult = latestPhotonResult;
    SmartDashboard.putNumber("Vision/LatestResultAgeMs", cameraResultAgeMs(latestResult));
    SmartDashboard.putNumber("Vision/VisibleFuelTargets", latestResult.getTargets().size());
    SmartDashboard.putBoolean("Vision/HasTargets", latestResult.hasTargets());
    SmartDashboard.putBoolean("Vision/HasFuelTarget", latestResult.hasTargets());
    SmartDashboard.putNumber("Vision/FuelLatencyMs", latestResult.metadata.getLatencyMillis());

    if (!latestResult.hasTargets()) {
      SmartDashboard.putNumber("Vision/BestFuelYawDeg", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelPitchDeg", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelClass", -1.0);
      SmartDashboard.putNumber("Vision/BestFuelConfidence", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelRangeMeters", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelCameraToTargetX", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelCameraToTargetY", 0.0);
      SmartDashboard.putNumber("Vision/BestFuelCameraToTargetZ", 0.0);
      return;
    }

    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
    Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
    SmartDashboard.putNumber("Vision/BestFuelYawDeg", bestTarget.getYaw());
    SmartDashboard.putNumber("Vision/BestFuelPitchDeg", bestTarget.getPitch());
    SmartDashboard.putNumber("Vision/BestFuelClass", bestTarget.getDetectedObjectClassID());
    SmartDashboard.putNumber("Vision/BestFuelConfidence", bestTarget.getDetectedObjectConfidence());
    SmartDashboard.putNumber(
        "Vision/BestFuelRangeMeters", cameraToTarget.getTranslation().getNorm());
    SmartDashboard.putNumber("Vision/BestFuelCameraToTargetX", cameraToTarget.getX());
    SmartDashboard.putNumber("Vision/BestFuelCameraToTargetY", cameraToTarget.getY());
    SmartDashboard.putNumber("Vision/BestFuelCameraToTargetZ", cameraToTarget.getZ());
  }

  private double cameraResultAgeMs(PhotonPipelineResult result) {
    if (result.metadata.getPublishTimestampMicros() <= 0) {
      return 0.0;
    }

    return Math.max(
        0.0, (RobotController.getFPGATime() - result.metadata.getPublishTimestampMicros()) / 1000.0);
  }
}
