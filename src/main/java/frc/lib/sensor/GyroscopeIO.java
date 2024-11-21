package frc.lib.sensor;

// plagiarism is bad
public interface GyroscopeIO {
  public static class GyroscopeIOValues {
    /** Roll angle in rotations */
    public double rollRotations = 0.0;

    /** Pitch angle in rotations */
    public double pitchRotations = 0.0;

    /** Yaw angle in rotations */
    public double yawRotatoins = 0.0;

    /** Roll velocity in rotations */
    public double rollVelocityRotations = 0.0;

    /** Pitch velocity in rotations */
    public double pitchVelocityRotations = 0.0;

    /** Yaw velocity in rotations */
    public double yawVelocityRotations = 0.0;
  }

  /** Configures the gyroscope */
  public void configure();

  /**
   * Updates gyroscope's values
   * 
   * @param values values
   */
  public void update(GyroscopeIOValues values);

  /**
   * Set gyroscope's yaw angle
   * 
   * @param yawRotations yaw angle in rotations
   */
  public void setYaw(double yawRotations);
}