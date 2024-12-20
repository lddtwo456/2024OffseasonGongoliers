package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Drive request */
public record DriveRequest(
    DriveRequest.TranslationMode translationMode,
    DriveRequest.RotationMode rotationMode,
    Translation2d translationAxis,
    Translation2d headingAxis,
    double rotationVelocityAxis) {
  
  /** Translation mode enum */
  private enum TranslationMode {
    /** Field-centric driving */
    FIELD_CENTRIC,
    /** Robot-centric driving */
    ROBOT_CENTRIC
  }

  /** Rotation mode enum */
  private enum RotationMode {
    /** Drifting (no requested rotation) */
    DRIFTING,
    /** Spinning (velocity requested) */
    SPINNING,
    /** Aligning (heading requested) */
    ALIGNING,
    /** Targetting (targetting heading requested) */
    TARGETTING
  }

  /**
   * Returns true if no rotation is requested
   * 
   * @param headingAxis axis of right stick (headnig axis for aligning)
   * @param aligning true if aligning is requested
   * @return true if no rotation is requested
   */
  private static boolean isDrifting(Translation2d headingAxis, boolean aligning) {
    if (aligning) { // if you're aligning (aiming the robot chassis wherever the right stick is pointing), have a very high stick deadzone to prevent huge rotation chaanges
      return headingAxis.getNorm() < 0.7;
    }
    
    return Math.abs(headingAxis.getY()) < 0.1; // if you aren't alligning, use a lower right stick deadzone of 0.1
  }

  /**
   * Creates a new driver request from controller inputs
   * 
   * @param controller input controller
   * @return a new driver request from controller inputs
   */
  public static DriveRequest fromController(CommandXboxController controller) {
    boolean snipingRequested = Math.abs(controller.getLeftTriggerAxis()) > 0.5;
    boolean aligningRequested = Math.abs(controller.getRightTriggerAxis()) > 0.5;

    double translationX = -controller.getLeftY();
    double translationY = -controller.getLeftX();
    double translationMagnitude = Math.hypot(translationX, translationY);
    
    // apply deadzone and exponential smoothing to left stick
    translationMagnitude = MathUtil.applyDeadband(translationMagnitude, 0.1);
    translationMagnitude = Math.copySign(translationMagnitude * translationMagnitude, translationMagnitude);

    if (snipingRequested) {
      translationMagnitude *= 0.25;
    }

    Translation2d translationAxis = new Translation2d(translationMagnitude, new Rotation2d(translationX, translationY));
    
    TranslationMode translationMode = TranslationMode.FIELD_CENTRIC;

    // direction of right stick for aligning mode
    Translation2d headingAxis = new Translation2d(-controller.getRightY(), -controller.getRightX());

    // determine rotation mode (if right stick in deadzone default to drifting, if aligning and not in deadzone and aligning requested you're aligning, and if not in deadzone and not aligning you're spinning)
    RotationMode rotationMode;
    if (isDrifting(headingAxis, aligningRequested)) {
      rotationMode = RotationMode.DRIFTING;
    } else if (aligningRequested) {
      rotationMode = RotationMode.ALIGNING;
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
