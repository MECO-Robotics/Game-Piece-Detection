package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhotonVisionSim;
import frc.robot.subsystems.SimDrivetrain;

public class Robot extends TimedRobot {
  private final SimDrivetrain drivetrain = new SimDrivetrain();
  private PhotonVisionSim photonVisionSim;

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", drivetrain.getField());

    if (isSimulation()) {
      photonVisionSim = new PhotonVisionSim();
    }
  }

  @Override
  public void robotPeriodic() {
    drivetrain.periodic();

    SmartDashboard.putNumber("Robot/PoseX", drivetrain.getPose().getX());
    SmartDashboard.putNumber("Robot/PoseY", drivetrain.getPose().getY());
    SmartDashboard.putNumber("Robot/HeadingDeg", drivetrain.getPose().getRotation().getDegrees());

    if (photonVisionSim != null) {
      photonVisionSim.update(drivetrain.getPose());
    }
  }

  @Override
  public void autonomousInit() {
    drivetrain.resetToStart();
    drivetrain.drive(1.0, 0.0, 0.35);
  }

  @Override
  public void autonomousExit() {
    drivetrain.stop();
  }

  @Override
  public void teleopInit() {
    drivetrain.drive(0.6, 0.0, 0.2);
  }

  @Override
  public void disabledInit() {
    drivetrain.stop();
  }
}
