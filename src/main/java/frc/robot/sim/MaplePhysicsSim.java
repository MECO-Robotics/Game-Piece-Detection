package frc.robot.sim;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class MaplePhysicsSim {
  private static final String kFuelType = "fuel";
  private static final GamePieceOnFieldSimulation.GamePieceInfo kFuelInfo =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          kFuelType,
          Geometry.createCircle(Constants.Physics.kFuelCollisionRadiusMeters),
          Meters.of(Constants.Vision.kFuelTargetHeightMeters),
          Kilograms.of(0.2),
          Constants.Physics.kFuelFrictionPerSecond,
          5.0,
          Constants.Physics.kFuelWallRestitution);

  private final FieldPhysicsArena arena = new FieldPhysicsArena();
  private final SimpleMapleDrivetrain drivetrain =
      new SimpleMapleDrivetrain(Constants.Drivetrain.kStartingPose());
  private final List<GamePieceOnFieldSimulation> availableFuel = new ArrayList<>();
  private boolean robotConstrainedThisLoop;

  public MaplePhysicsSim() {
    SimulatedArena.overrideInstance(arena);
    arena.addDriveTrainSimulation(drivetrain);
    reset();
  }

  public void reset() {
    arena.clearGamePieces();
    availableFuel.clear();
    for (Pose2d fuelPose : Constants.Field.kStartingFuelPoses) {
      GamePieceOnFieldSimulation fuel = new GamePieceOnFieldSimulation(kFuelInfo, fuelPose);
      availableFuel.add(fuel);
      arena.addGamePiece(fuel);
    }
    drivetrain.setSimulationWorldPose(Constants.Drivetrain.kStartingPose());
  }

  public Pose2d update(Pose2d proposedRobotPose, ChassisSpeeds robotRelativeSpeeds) {
    drivetrain.setSimulationWorldPose(proposedRobotPose);
    drivetrain.setRobotSpeeds(toFieldRelative(robotRelativeSpeeds, proposedRobotPose));
    arena.simulationPeriodic();

    Pose2d maplePose = drivetrain.getSimulatedDriveTrainPose();
    robotConstrainedThisLoop =
        maplePose.getTranslation().getDistance(proposedRobotPose.getTranslation()) > 0.005;
    return maplePose;
  }

  public Optional<Pose2d> getFirstAvailableFuelPose() {
    return availableFuel.stream().findFirst().map(GamePieceOnFieldSimulation::getPoseOnField);
  }

  public List<Pose2d> getAvailableFuelPoses() {
    return availableFuel.stream().map(GamePieceOnFieldSimulation::getPoseOnField).toList();
  }

  public boolean tryPickupClosestFuel(Pose2d robotPose, double pickupRadiusMeters) {
    Optional<GamePieceOnFieldSimulation> closest =
        availableFuel.stream()
            .filter(fuel -> fuel.getPoseOnField().getTranslation().getDistance(robotPose.getTranslation())
                <= pickupRadiusMeters)
            .min(
                Comparator.comparingDouble(
                    fuel ->
                        fuel.getPoseOnField()
                            .getTranslation()
                            .getDistance(robotPose.getTranslation())));

    closest.ifPresent(
        fuel -> {
          availableFuel.remove(fuel);
          arena.removeGamePiece(fuel);
        });
    return closest.isPresent();
  }

  public int getAvailableFuelCount() {
    return availableFuel.size();
  }

  public double getMaxFuelSpeed() {
    return availableFuel.stream()
        .mapToDouble(fuel -> fuel.getVelocity3dMPS().toTranslation2d().getNorm())
        .max()
        .orElse(0.0);
  }

  public boolean didRobotHitMapleObstacle() {
    return robotConstrainedThisLoop;
  }

  private static ChassisSpeeds toFieldRelative(ChassisSpeeds robotRelativeSpeeds, Pose2d robotPose) {
    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();
    return new ChassisSpeeds(
        robotRelativeSpeeds.vxMetersPerSecond * cos - robotRelativeSpeeds.vyMetersPerSecond * sin,
        robotRelativeSpeeds.vxMetersPerSecond * sin + robotRelativeSpeeds.vyMetersPerSecond * cos,
        robotRelativeSpeeds.omegaRadiansPerSecond);
  }

  private static class FieldPhysicsArena extends SimulatedArena {
    FieldPhysicsArena() {
      super(new FieldPhysicsMap());
      disableBreakdownPublishing();
    }

    @Override
    public void placeGamePiecesOnField() {}
  }

  private static class FieldPhysicsMap extends SimulatedArena.FieldMap {
    FieldPhysicsMap() {
      addBorderLine(new Translation2d(0.0, 0.0), new Translation2d(Constants.Field.kLengthMeters, 0.0));
      addBorderLine(
          new Translation2d(Constants.Field.kLengthMeters, 0.0),
          new Translation2d(Constants.Field.kLengthMeters, Constants.Field.kWidthMeters));
      addBorderLine(
          new Translation2d(Constants.Field.kLengthMeters, Constants.Field.kWidthMeters),
          new Translation2d(0.0, Constants.Field.kWidthMeters));
      addBorderLine(new Translation2d(0.0, Constants.Field.kWidthMeters), new Translation2d(0.0, 0.0));
      addRectangularObstacle(
          Constants.Field.kBumpLengthMeters,
          Constants.Field.kBumpWidthMeters,
          Constants.Field.kBumpCenter);
    }
  }

  private static class SimpleMapleDrivetrain extends AbstractDriveTrainSimulation {
    SimpleMapleDrivetrain(Pose2d initialPose) {
      super(
          DriveTrainSimulationConfig.Default()
              .withBumperSize(
                  Meters.of(Constants.Physics.kRobotBumperLengthMeters),
                  Meters.of(Constants.Physics.kRobotBumperWidthMeters)),
          initialPose);
    }

    @Override
    public void simulationSubTick() {}
  }
}
