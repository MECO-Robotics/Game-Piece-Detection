package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.SimDrivetrain;
import org.junit.jupiter.api.Test;

class SimDrivetrainTest {
  @Test
  void driveCommandMovesRobotForwardInSim() throws InterruptedException {
    SimDrivetrain drivetrain = new SimDrivetrain();
    double startX = drivetrain.getPose().getX();

    drivetrain.drive(1.0, 0.0, 0.0);
    Thread.sleep(25);
    drivetrain.periodic();

    assertTrue(drivetrain.getPose().getX() > startX);
  }
}
