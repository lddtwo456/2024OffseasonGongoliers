package frc.lib.controllers;

/** Interface for motor controller used as position control */
public interface PositionController {

  /** Position controller values */
  public static class PositionControllerValues {

    /** Current position in rotations */
    public double posRotations = 0.0;

    /** Current velocity in rotations per second */
    public double velRotationsPerSec = 0.0;

    /** Current acceleration in rotations per second per second */
    public double accRotationsPerSecPerSec = 0.0;

    /** Current voltage */
    public double motorVolts = 0.0;

    /** Current current */
    public double motorAmps = 0.0;
  }

  /**
   * Configures the position controller
   */
  public void configure();

  /**
   * Updates the position controller's values
   * 
   * @param values new values
   */
  public void update(PositionControllerValues values);

  /**
   * Sets the position of the controller
   * 
   * @param posRotations position in rotations
   */
  public void setPos(double posRotations);

  /**
   * Sets the setpoint (target position and velocity) of the controller
   * 
   * @param posRotations position in rotations
   * @param velRotationsPerSec velocity in rotations per second
   */
  public void setSetpoint(double posRotations, double velRotationsPerSec);
}
