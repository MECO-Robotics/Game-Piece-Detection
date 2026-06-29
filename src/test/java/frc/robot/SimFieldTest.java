package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.sim.SimField;
import org.junit.jupiter.api.Test;

class SimFieldTest {
  @Test
  void pickupAndScoreMoveFuelThroughTruthStates() {
    SimField simField = new SimField(new Field2d());
    Pose2d firstFuelPose = Constants.Field.kStartingFuelPoses.get(0);

    assertEquals(Constants.Field.kStartingFuelPoses.size(), simField.getAvailableFuelCount());
    assertTrue(simField.tryPickupClosestFuel(firstFuelPose));
    assertTrue(simField.isCarryingFuel());
    assertEquals(Constants.Field.kStartingFuelPoses.size() - 1, simField.getAvailableFuelCount());

    assertFalse(simField.tryScoreCarriedFuel(firstFuelPose));
    assertTrue(simField.tryScoreCarriedFuel(Constants.Field.kScoringZoneCenter));
    assertFalse(simField.isCarryingFuel());
    assertEquals(1, simField.getScoredFuelCount());
  }
}
