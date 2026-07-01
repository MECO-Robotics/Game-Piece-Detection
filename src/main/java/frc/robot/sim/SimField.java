package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class SimField {
  private final Field2d field;
  private final FieldZone pickupZone =
      new FieldZone(
          "PickupZone",
          Constants.Field.kPickupZoneCenter,
          Constants.Field.kPickupZoneLengthMeters,
          Constants.Field.kPickupZoneWidthMeters);
  private final FieldZone scoringZone =
      new FieldZone(
          "ScoringZone",
          Constants.Field.kScoringZoneCenter,
          Constants.Field.kScoringZoneLengthMeters,
          Constants.Field.kScoringZoneWidthMeters);

  private final List<SimGamePiece> availableFuel = new ArrayList<>();
  private final List<SimGamePiece> scoredFuel = new ArrayList<>();
  private Optional<SimGamePiece> carriedFuel = Optional.empty();

  public SimField(Field2d field) {
    this.field = field;
    reset();
  }

  public void reset() {
    availableFuel.clear();
    scoredFuel.clear();
    carriedFuel = Optional.empty();

    for (int i = 0; i < Constants.Field.kStartingFuelPoses.size(); i++) {
      availableFuel.add(new SimGamePiece(i + 1, "fuel", Constants.Field.kStartingFuelPoses.get(i)));
    }

    publishFieldObjects(Constants.Drivetrain.kStartingPose());
  }

  public void periodic(Pose2d robotPose) {
    carriedFuel = carriedFuel.map(piece -> piece.withPose(robotPose));
    publishFieldObjects(robotPose);
    SmartDashboard.putNumber("SimField/AvailableFuel", availableFuel.size());
    SmartDashboard.putNumber("SimField/ScoredFuel", scoredFuel.size());
    SmartDashboard.putBoolean("SimField/CarryingFuel", carriedFuel.isPresent());
    SmartDashboard.putBoolean("SimField/RobotInPickupZone", pickupZone.contains(robotPose));
    SmartDashboard.putBoolean("SimField/RobotInScoringZone", scoringZone.contains(robotPose));
  }

  public Optional<Pose2d> getFirstAvailableFuelPose() {
    return availableFuel.stream().findFirst().map(SimGamePiece::pose);
  }

  public List<Pose2d> getAvailableFuelPoses() {
    return availableFuel.stream().map(SimGamePiece::pose).toList();
  }

  public boolean tryPickupClosestFuel(Pose2d robotPose) {
    if (carriedFuel.isPresent()) {
      return false;
    }

    Optional<SimGamePiece> closest =
        availableFuel.stream()
            .filter(piece -> distance(robotPose, piece.pose()) <= Constants.Field.kPickupRadiusMeters)
            .min(Comparator.comparingDouble(piece -> distance(robotPose, piece.pose())));

    closest.ifPresent(
        piece -> {
          availableFuel.remove(piece);
          carriedFuel = Optional.of(piece.withPose(robotPose));
        });
    return closest.isPresent();
  }

  public boolean tryScoreCarriedFuel(Pose2d robotPose) {
    if (carriedFuel.isEmpty() || !scoringZone.contains(robotPose)) {
      return false;
    }

    int scoredIndex = scoredFuel.size();
    Pose2d scoredPose =
        new Pose2d(
            Constants.Field.kScoringZoneCenter.getX(),
            Constants.Field.kScoringZoneCenter.getY() + scoredIndex * 0.18,
            Constants.Field.kScoringZoneCenter.getRotation());
    scoredFuel.add(carriedFuel.get().withPose(scoredPose));
    carriedFuel = Optional.empty();
    return true;
  }

  public int getAvailableFuelCount() {
    return availableFuel.size();
  }

  public int getScoredFuelCount() {
    return scoredFuel.size();
  }

  public boolean isCarryingFuel() {
    return carriedFuel.isPresent();
  }

  public boolean isInPickupZone(Pose2d robotPose) {
    return pickupZone.contains(robotPose);
  }

  public boolean isInScoringZone(Pose2d robotPose) {
    return scoringZone.contains(robotPose);
  }

  private void publishFieldObjects(Pose2d robotPose) {
    field.getObject("AvailableFuel").setPoses(availableFuel.stream().map(SimGamePiece::pose).toList());
    field.getObject("CarriedFuel")
        .setPoses(carriedFuel.map(piece -> List.of(piece.pose())).orElseGet(List::of));
    field.getObject("ScoredFuel").setPoses(scoredFuel.stream().map(SimGamePiece::pose).toList());
    field.getObject("PickupZone").setPoses(pickupZone.markerPoses());
    field.getObject("ScoringZone").setPoses(scoringZone.markerPoses());
    field.getObject("RobotTruth").setPose(robotPose);
  }

  private static double distance(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }
}
