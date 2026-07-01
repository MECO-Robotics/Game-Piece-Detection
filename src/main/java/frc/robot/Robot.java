package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhotonVisionSim;
import frc.robot.sim.SimField;
import frc.robot.sim.RgbdDepthSim;
import frc.robot.sim.WaypointPathFollower;
import frc.robot.subsystems.SimDrivetrain;
import java.util.Optional;

public class Robot extends TimedRobot {
  private enum Day2AutoStep {
    DRIVE_TO_FUEL,
    DRIVE_TO_SCORE,
    DONE
  }

  private final SimDrivetrain drivetrain = new SimDrivetrain();
  private final SimField simField = new SimField(drivetrain.getField());
  private final WaypointPathFollower pathFollower = new WaypointPathFollower();
  private PhotonVisionSim photonVisionSim;
  private RgbdDepthSim rgbdDepthSim;
  private Day2AutoStep day2AutoStep = Day2AutoStep.DONE;
  private Optional<Pose2d> activeFuelTarget = Optional.empty();

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", drivetrain.getField());

    if (isSimulation()) {
      photonVisionSim = new PhotonVisionSim();
      rgbdDepthSim = new RgbdDepthSim();
    }
  }

  @Override
  public void robotPeriodic() {
    drivetrain.periodic();
    simField.periodic(drivetrain.getPose());

    SmartDashboard.putNumber("Robot/PoseX", drivetrain.getPose().getX());
    SmartDashboard.putNumber("Robot/PoseY", drivetrain.getPose().getY());
    SmartDashboard.putNumber("Robot/HeadingDeg", drivetrain.getPose().getRotation().getDegrees());
    SmartDashboard.putString("Day2Auto/Step", day2AutoStep.name());

    if (photonVisionSim != null) {
      try {
        photonVisionSim.update(drivetrain.getPose(), simField.getAvailableFuelPoses());
        rgbdDepthSim.update(drivetrain.getPose(), simField.getAvailableFuelPoses());
        SmartDashboard.putBoolean("Vision/UpdateFault", false);
        SmartDashboard.putBoolean("Vision/UpdateHealthy", true);
        SmartDashboard.putString("Vision/Status", "OK");
      } catch (RuntimeException ex) {
        SmartDashboard.putBoolean("Vision/UpdateFault", true);
        SmartDashboard.putBoolean("Vision/UpdateHealthy", false);
        SmartDashboard.putString("Vision/Status", "FAULT");
        SmartDashboard.putString("Vision/UpdateFaultMessage", ex.getMessage());
      }
    }
  }

  @Override
  public void autonomousInit() {
    drivetrain.resetToStart();
    simField.reset();
    activeFuelTarget = simField.getFirstAvailableFuelPose();
    day2AutoStep = activeFuelTarget.isPresent() ? Day2AutoStep.DRIVE_TO_FUEL : Day2AutoStep.DONE;
  }

  @Override
  public void autonomousPeriodic() {
    switch (day2AutoStep) {
      case DRIVE_TO_FUEL -> driveToFuel();
      case DRIVE_TO_SCORE -> driveToScore();
      case DONE -> drivetrain.stop();
    }
  }

  @Override
  public void autonomousExit() {
    drivetrain.stop();
  }

  @Override
  public void teleopInit() {
    drivetrain.stop();
  }

  @Override
  public void disabledInit() {
    drivetrain.stop();
  }

  private void driveToFuel() {
    if (activeFuelTarget.isEmpty()) {
      day2AutoStep = Day2AutoStep.DONE;
      return;
    }

    drivetrain.drive(pathFollower.calculate(drivetrain.getPose(), activeFuelTarget.get()));
    if (pathFollower.isAtTarget(drivetrain.getPose(), activeFuelTarget.get())
        && simField.tryPickupClosestFuel(drivetrain.getPose())) {
      day2AutoStep = Day2AutoStep.DRIVE_TO_SCORE;
    }
  }

  private void driveToScore() {
    drivetrain.drive(pathFollower.calculate(drivetrain.getPose(), Constants.Field.kScoringZoneCenter));
    if (pathFollower.isAtTarget(drivetrain.getPose(), Constants.Field.kScoringZoneCenter)
        && simField.tryScoreCarriedFuel(drivetrain.getPose())) {
      day2AutoStep = Day2AutoStep.DONE;
    }
  }
}
