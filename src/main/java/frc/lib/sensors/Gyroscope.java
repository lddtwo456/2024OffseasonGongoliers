package frc.lib.sensors;

/** Gyroscope interface */
public interface Gyroscope {
  
  /** Gyroscope values */
  public static class GyroscopeValues {

    /** Roll angle in rotations */
    public double rollRotations = 0.0;

    /** Pitch angle in rotations */
    public double pitchRotations = 0.0;

    /** Yaw angle in rotations */
    public double yawRotations = 0.0;

    /** Roll velocity in rotations per second */
    public double rollVelocityRotations = 0.0;

    /** Pitch velocity in rotations per second */
    public double pitchVelocityRotations = 0.0;

    /** Yaw velocity in rotations per second */
    public double yawVelocityRotations = 0.0;
  }

  /** Configures the gyroscope */
  public void configure();

  /**
   * Updates the gyroscope's values
   * 
   * @param values values to be updated
   */
  public void getUpdatedVals(GyroscopeValues values);
  
  /**
   * Sets the gyroscope's yaw in rotations
   * 
   * @param yawRotations yaw in rotations
   */
  public void setYaw(double yawRotations);
}
