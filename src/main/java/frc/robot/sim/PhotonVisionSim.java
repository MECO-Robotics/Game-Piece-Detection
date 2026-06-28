package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonVisionSim {
  private final PhotonCamera camera = new PhotonCamera(Constants.Vision.kPhotonCameraName);
  private final VisionSystemSim visionSystem = new VisionSystemSim("rgbd-photonvision-sim");
  private final PhotonCameraSim cameraSim;

  public PhotonVisionSim() {
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(
        Constants.Vision.kCameraWidth,
        Constants.Vision.kCameraHeight,
        Rotation2d.fromDegrees(Constants.Vision.kHorizontalFovDegrees));
    cameraProperties.setFPS(Constants.Vision.kNominalFps);
    cameraProperties.setAvgLatencyMs(Constants.Vision.kAverageLatencyMs);
    cameraProperties.setLatencyStdDevMs(Constants.Vision.kLatencyStdDevMs);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

    visionSystem.addCamera(cameraSim, Constants.Vision.kRobotToCamera);
    SmartDashboard.putBoolean("Vision/PhotonSimReady", true);
    SmartDashboard.putString("Vision/PhotonCamera", Constants.Vision.kPhotonCameraName);
  }

  public void update(Pose2d robotPose) {
    visionSystem.update(robotPose);
    SmartDashboard.putNumber("Vision/RobotToCameraX", Constants.Vision.kRobotToCamera.getX());
    SmartDashboard.putNumber("Vision/RobotToCameraZ", Constants.Vision.kRobotToCamera.getZ());
  }
}
