package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RgbdViabilitySweep {
  private SweepResult latestResult = SweepResult.empty();

  public void runAndPublish() {
    latestResult = runSweep();
    publish(latestResult);
  }

  public SweepResult latestResult() {
    return latestResult;
  }

  private SweepResult runSweep() {
    int totalCases = 0;
    int visibleCases = 0;
    int usableCases = 0;
    int usableCloseCases = 0;
    int closeCases = 0;
    double worstUsableRange = 0.0;
    double bestUsableRange = 0.0;
    double totalUsableDepthError = 0.0;
    String limitingCase = "none";

    for (double fovDeg : Constants.Viability.kHorizontalFovDegrees) {
      for (double pitchDeg : Constants.Viability.kCameraPitchDegrees) {
        for (double rangeMeters : Constants.Viability.kRangeMeters) {
          for (double yawDeg : Constants.Viability.kYawDegrees) {
            for (double occlusion : Constants.Viability.kOcclusionFractions) {
              for (double latencyMs : Constants.Viability.kLatencyMs) {
                for (double noiseMeters : Constants.Viability.kDepthNoiseMeters) {
                  totalCases++;
                  boolean closeRange = rangeMeters <= 4.0;
                  closeCases += closeRange ? 1 : 0;

                  CaseResult result =
                      evaluateCase(fovDeg, pitchDeg, rangeMeters, yawDeg, occlusion, latencyMs, noiseMeters);
                  if (result.visible) {
                    visibleCases++;
                  }
                  if (result.usable) {
                    usableCases++;
                    usableCloseCases += closeRange ? 1 : 0;
                    totalUsableDepthError += result.depthErrorMeters;
                    worstUsableRange = Math.max(worstUsableRange, rangeMeters);
                    bestUsableRange = bestUsableRange == 0.0 ? rangeMeters : Math.min(bestUsableRange, rangeMeters);
                  } else if (limitingCase.equals("none")) {
                    limitingCase = result.reason;
                  }
                }
              }
            }
          }
        }
      }
    }

    return new SweepResult(
        totalCases,
        visibleCases,
        usableCases,
        closeCases,
        usableCloseCases,
        bestUsableRange,
        worstUsableRange,
        usableCases > 0 ? totalUsableDepthError / usableCases : 0.0,
        limitingCase);
  }

  private CaseResult evaluateCase(
      double fovDeg,
      double pitchDeg,
      double rangeMeters,
      double yawDeg,
      double occlusion,
      double latencyMs,
      double noiseMeters) {
    boolean inRange =
        rangeMeters >= Constants.Vision.kDepthMinRangeMeters
            && rangeMeters <= Constants.Vision.kDepthMaxRangeMeters;
    boolean inFov = Math.abs(yawDeg) <= fovDeg / 2.0;
    boolean pitchUsable = pitchDeg >= -25.0 && pitchDeg <= 5.0;
    double targetAreaPixels = targetAreaPixels(fovDeg, rangeMeters, occlusion);
    boolean enoughPixels = targetAreaPixels >= Constants.Vision.kMinFuelTargetAreaPixels;
    boolean visible = inRange && inFov && pitchUsable && enoughPixels && occlusion < 0.8;

    double latencyErrorMeters =
        Constants.Viability.kAssumedApproachSpeedMetersPerSecond * (latencyMs / 1000.0);
    double depthErrorMeters = Math.hypot(noiseMeters, latencyErrorMeters);
    double confidence =
        Constants.Vision.kFuelTargetConfidence
            * (1.0 - occlusion)
            * Math.max(0.0, 1.0 - depthErrorMeters / 0.5);
    boolean usable =
        visible
            && depthErrorMeters <= Constants.Viability.kMaxUsableDepthErrorMeters
            && confidence >= Constants.Viability.kMinUsableConfidence;

    String reason = "usable";
    if (!inRange) {
      reason = "range";
    } else if (!inFov) {
      reason = "fov";
    } else if (!pitchUsable) {
      reason = "mount";
    } else if (!enoughPixels) {
      reason = "pixels";
    } else if (occlusion >= 0.8) {
      reason = "occlusion";
    } else if (depthErrorMeters > Constants.Viability.kMaxUsableDepthErrorMeters) {
      reason = "depth-error";
    } else if (confidence < Constants.Viability.kMinUsableConfidence) {
      reason = "confidence";
    }

    return new CaseResult(visible, usable, depthErrorMeters, confidence, reason);
  }

  private double targetAreaPixels(double fovDeg, double rangeMeters, double occlusion) {
    double focalPixels =
        Constants.Vision.kCameraWidth / (2.0 * Math.tan(Math.toRadians(fovDeg) / 2.0));
    double diameterPixels =
        focalPixels * Constants.Vision.kFuelTargetDiameterMeters / Math.max(0.001, rangeMeters);
    double radiusPixels = diameterPixels / 2.0;
    return Math.PI * radiusPixels * radiusPixels * (1.0 - occlusion);
  }

  private void publish(SweepResult result) {
    SmartDashboard.putBoolean("Day5/ViabilitySweepReady", true);
    SmartDashboard.putNumber("Day5/TotalCases", result.totalCases);
    SmartDashboard.putNumber("Day5/VisibleCases", result.visibleCases);
    SmartDashboard.putNumber("Day5/UsableCases", result.usableCases);
    SmartDashboard.putNumber("Day5/VisiblePercent", result.visiblePercent());
    SmartDashboard.putNumber("Day5/UsablePercent", result.usablePercent());
    SmartDashboard.putNumber("Day5/CloseRangeUsablePercent", result.closeRangeUsablePercent());
    SmartDashboard.putNumber("Day5/BestUsableRangeMeters", result.bestUsableRangeMeters);
    SmartDashboard.putNumber("Day5/WorstUsableRangeMeters", result.worstUsableRangeMeters);
    SmartDashboard.putNumber("Day5/AvgUsableDepthErrorMeters", result.averageUsableDepthErrorMeters);
    SmartDashboard.putString("Day5/FirstLimitingCase", result.firstLimitingCase);
    SmartDashboard.putBoolean("Day5/MeetsCloseRangeTarget", result.closeRangeUsablePercent() >= 0.75);
    SmartDashboard.putBoolean("Day5/MeetsOverallTarget", result.usablePercent() >= 0.35);
  }

  private record CaseResult(
      boolean visible,
      boolean usable,
      double depthErrorMeters,
      double confidence,
      String reason) {}

  public record SweepResult(
      int totalCases,
      int visibleCases,
      int usableCases,
      int closeRangeCases,
      int closeRangeUsableCases,
      double bestUsableRangeMeters,
      double worstUsableRangeMeters,
      double averageUsableDepthErrorMeters,
      String firstLimitingCase) {
    static SweepResult empty() {
      return new SweepResult(0, 0, 0, 0, 0, 0.0, 0.0, 0.0, "none");
    }

    double visiblePercent() {
      return totalCases == 0 ? 0.0 : ((double) visibleCases) / totalCases;
    }

    double usablePercent() {
      return totalCases == 0 ? 0.0 : ((double) usableCases) / totalCases;
    }

    double closeRangeUsablePercent() {
      return closeRangeCases == 0 ? 0.0 : ((double) closeRangeUsableCases) / closeRangeCases;
    }
  }
}
