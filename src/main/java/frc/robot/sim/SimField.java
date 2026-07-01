package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;

public class SimField {
  private final Field2d field;
  private final MaplePhysicsSim maplePhysics = new MaplePhysicsSim();
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
  private final FieldZone bump =
      new FieldZone(
          "Bump",
          Constants.Field.kBumpCenter,
          Constants.Field.kBumpLengthMeters,
          Constants.Field.kBumpWidthMeters);

  private Optional<SimGamePiece> carriedFuel = Optional.empty();
  private int scoredFuelCount;
  private boolean fuelPickedUpThisLoop;
  private int totalFuelPickups;

  public SimField(Field2d field) {
    this.field = field;
    reset();
  }

  public void reset() {
    maplePhysics.reset();
    carriedFuel = Optional.empty();
    scoredFuelCount = 0;
    fuelPickedUpThisLoop = false;
    totalFuelPickups = 0;
    publishFieldObjects(Constants.Drivetrain.kStartingPose());
  }

  public Pose2d applyPhysics(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, double dtSeconds) {
    fuelPickedUpThisLoop = false;
    Pose2d mapleRobotPose = maplePhysics.update(robotPose, robotRelativeSpeeds);
    if (tryPickupClosestFuel(mapleRobotPose)) {
      fuelPickedUpThisLoop = true;
      totalFuelPickups++;
    }
    return mapleRobotPose;
  }

  public void periodic(Pose2d robotPose) {
    carriedFuel = carriedFuel.map(piece -> piece.withPose(robotPose));
    publishFieldObjects(robotPose);
    SmartDashboard.putNumber("SimField/AvailableFuel", maplePhysics.getAvailableFuelCount());
    SmartDashboard.putNumber("SimField/ScoredFuel", scoredFuelCount);
    SmartDashboard.putBoolean("SimField/CarryingFuel", carriedFuel.isPresent());
    SmartDashboard.putBoolean("SimField/RobotInPickupZone", pickupZone.contains(robotPose));
    SmartDashboard.putBoolean("SimField/RobotInScoringZone", scoringZone.contains(robotPose));
    SmartDashboard.putBoolean("SimField/FuelPickedUpThisLoop", fuelPickedUpThisLoop);
    SmartDashboard.putNumber("SimField/TotalFuelPickups", totalFuelPickups);
    SmartDashboard.putBoolean("MapleSim/Enabled", true);
    SmartDashboard.putBoolean("MapleSim/RobotHitObstacle", maplePhysics.didRobotHitMapleObstacle());
    SmartDashboard.putNumber("MapleSim/AvailableFuel", maplePhysics.getAvailableFuelCount());
    SmartDashboard.putNumber("MapleSim/MaxFuelSpeed", maplePhysics.getMaxFuelSpeed());
  }

  public Optional<Pose2d> getFirstAvailableFuelPose() {
    return maplePhysics.getFirstAvailableFuelPose();
  }

  public List<Pose2d> getAvailableFuelPoses() {
    return maplePhysics.getAvailableFuelPoses();
  }

  public List<Pose2d> getAvailableFuelPoses() {
    return availableFuel.stream().map(SimGamePiece::pose).toList();
  }

  public boolean tryPickupClosestFuel(Pose2d robotPose) {
    if (carriedFuel.isPresent()) {
      return false;
    }

    boolean pickedUp =
        maplePhysics.tryPickupClosestFuel(
            robotPose, Constants.Physics.kIntakeCaptureRadiusMeters);
    if (pickedUp) {
      carriedFuel = Optional.of(new SimGamePiece(-1, "fuel", robotPose));
    }
    return pickedUp;
  }

  public boolean tryScoreCarriedFuel(Pose2d robotPose) {
    if (carriedFuel.isEmpty() || !scoringZone.contains(robotPose)) {
      return false;
    }

    scoredFuelCount++;
    carriedFuel = Optional.empty();
    return true;
  }

  public int getAvailableFuelCount() {
    return maplePhysics.getAvailableFuelCount();
  }

  public int getScoredFuelCount() {
    return scoredFuelCount;
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
    field.getObject("AvailableFuel").setPoses(maplePhysics.getAvailableFuelPoses());
    field.getObject("CarriedFuel")
        .setPoses(carriedFuel.map(piece -> List.of(piece.pose())).orElseGet(List::of));
    field.getObject("ScoredFuel").setPoses(scoredFuelMarkerPoses());
    field.getObject("PickupZone").setPoses(pickupZone.markerPoses());
    field.getObject("ScoringZone").setPoses(scoringZone.markerPoses());
    field.getObject("Bump").setPoses(bump.markerPoses());
    field.getObject("FieldBounds").setPoses(fieldBoundaryMarkerPoses());
    field.getObject("RobotTruth").setPose(robotPose);
  }

  private List<Pose2d> scoredFuelMarkerPoses() {
    return java.util.stream.IntStream.range(0, scoredFuelCount)
        .mapToObj(
            index ->
                new Pose2d(
                    Constants.Field.kScoringZoneCenter.getX(),
                    Constants.Field.kScoringZoneCenter.getY() + index * 0.18,
                    Constants.Field.kScoringZoneCenter.getRotation()))
        .toList();
  }

  private List<Pose2d> fieldBoundaryMarkerPoses() {
    return List.of(
        new Pose2d(0.0, 0.0, Rotation2d.kZero),
        new Pose2d(Constants.Field.kLengthMeters, 0.0, Rotation2d.kZero),
        new Pose2d(Constants.Field.kLengthMeters, Constants.Field.kWidthMeters, Rotation2d.kZero),
        new Pose2d(0.0, Constants.Field.kWidthMeters, Rotation2d.kZero));
  }
}
