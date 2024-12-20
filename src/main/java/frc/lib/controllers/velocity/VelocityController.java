package frc.lib.controllers.velocity;

/** Interface for motor controller used as velocity control */
public interface VelocityController {
  
  /** Velocity controller values */
  public static class VelocityControllerValues {

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
   * Configures the velocity controller
   */
  public void configure();

  /**
   * Get the velocity controller's updated values
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(VelocityControllerValues values);

  /**
   * Sets the position of the controller
   * 
   * @param posRotations position in rotations
   */
  public void setPos(double posRotations);

  /**
   * Sets the target velocity of the controller
   *  
   * @param velRotationsPerSec
   */
  public void setSetpoint(double velRotationsPerSec);
}
