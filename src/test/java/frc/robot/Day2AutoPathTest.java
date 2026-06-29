package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sim.SimField;
import frc.robot.sim.WaypointPathFollower;
import frc.robot.subsystems.SimDrivetrain;
import org.junit.jupiter.api.Test;

class Day2AutoPathTest {
  @Test
  void simplePathCanPickupAndScoreFuel() {
    SimDrivetrain drivetrain = new SimDrivetrain();
    SimField simField = new SimField(drivetrain.getField());
    WaypointPathFollower follower = new WaypointPathFollower();
    Pose2d fuelTarget = simField.getFirstAvailableFuelPose().orElseThrow();

    for (int i = 0; i < 250 && !simField.isCarryingFuel(); i++) {
      drivetrain.drive(follower.calculate(drivetrain.getPose(), fuelTarget));
      drivetrain.simulationPeriodic(0.02);
      simField.periodic(drivetrain.getPose());
      if (follower.isAtTarget(drivetrain.getPose(), fuelTarget)) {
        simField.tryPickupClosestFuel(drivetrain.getPose());
      }
    }

    for (int i = 0; i < 350 && simField.getScoredFuelCount() == 0; i++) {
      drivetrain.drive(follower.calculate(drivetrain.getPose(), Constants.Field.kScoringZoneCenter));
      drivetrain.simulationPeriodic(0.02);
      simField.periodic(drivetrain.getPose());
      if (follower.isAtTarget(drivetrain.getPose(), Constants.Field.kScoringZoneCenter)) {
        simField.tryScoreCarriedFuel(drivetrain.getPose());
      }
    }

    drivetrain.stop();
    assertFalse(simField.isCarryingFuel());
    assertEquals(1, simField.getScoredFuelCount());
  }
}
