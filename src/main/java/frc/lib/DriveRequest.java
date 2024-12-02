package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public record DriveRequest(
  DriveRequest.TranslationMode translationMode,
  DriveRequest.RotationMode rotationMode,
  Translation2d translationAxis,
  Translation2d headingAxis,
  double rotationVelocityAxis) {

  /** Translation modes */
  private enum TranslationMode {
    /** Field centric driving */
    FIELD_CENTRIC,
    /** Robot centric driving */
    ROBOT_CENTRIC
  }

  /** Rotation mode */
  private enum RotationMode {
    /** Drifting (no rotation requested) */
    DRIFTING,
    /** Spinning (rotation velocity requested) */
    SPINNING,
    /** Alligning (rotation heading requested) */
    ALLIGNING,
    /** Targetting (rotation heading to target requested) */
    TARGETTING
  }

  /**
   * Returns true if no rotation requested
   * 
   * @param headingAxis heading axis
   * @param aligning true if aligning is requested
   * @return true if no reotation is requested
   */
  private static boolean isDrifting(Translation2d headingAxis, boolean aligning) {
    if (aligning) {
      return headingAxis.getNorm() < 0.7;
    }

    return Math.abs(headingAxis.getY()) < 0.1;
  }

  /**
   * Create a new driver request from controller inputs
   * 
   * @param controller the input controller
   * @return a new driver request from controller inputs
   */
  public static DriveRequest fromController(CommandXboxController controller) {
    boolean snipingRequested = Math.abs(controller.getLeftTriggerAxis()) > 0.5;
    boolean aligningRequested = Math.abs(controller.getRightTriggerAxis()) > 0.5;
    boolean targetRequested = controller.leftBumper().getAsBoolean();

    double translationX = -controller.getLeftY();
    double translationY = -controller.getLeftX();

    double translationMagnitude = Math.hypot(translationX, translationY);
    Rotation2d translationDirection = new Rotation2d(translationX, translationY);

    translationMagnitude = MathUtil.applyDeadband(translationMagnitude, 0.1);
    translationMagnitude =
      Math.copySign(translationMagnitude * translationMagnitude, translationMagnitude);

    if (snipingRequested) {
      translationMagnitude *= 0.25;
    }

    Translation2d translationAxis = new Translation2d(translationMagnitude, translationDirection);

    TranslationMode translationMode = TranslationMode.FIELD_CENTRIC;

    Translation2d headingAxis = new Translation2d(-controller.getRightY(), -controller.getRightX());

    RotationMode rotationMode;

    if (isDrifting(headingAxis, aligningRequested)) {
      rotationMode = RotationMode.DRIFTING;
    } else if (aligningRequested) {
      rotationMode = RotationMode.ALLIGNING;
    } else if (targetRequested) {
      rotationMode = RotationMode.TARGETTING;
    } else {
      rotationMode = RotationMode.SPINNING;
    }

    double rotationVelocityAxis = 0.0;

    if (rotationMode == RotationMode.SPINNING) {
      rotationVelocityAxis = headingAxis.getY();
    }

    return new DriveRequest(
      translationMode, rotationMode, translationAxis, headingAxis, rotationVelocityAxis);
  }
}
