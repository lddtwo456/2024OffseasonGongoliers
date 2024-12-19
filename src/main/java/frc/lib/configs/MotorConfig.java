package frc.lib.configs;

/** 
 * Motor config
 * 
 * @param neutralBrake true means the motor will brake when given no voltage
 * @param ccwPositive true means positive voltage rotates the motor ccw
 * @param motorToMechRatio ratio of motor rotations to mechanism rotations
 * @param statorCurrentLimit current limit in the stator
 * @param supplyCurrentLimit supply current limit
 */
public record MotorConfig(
    boolean neutralBrake,
    boolean ccwPositive,
    double motorToMechRatio,
    double statorCurrentLimit,
    double supplyCurrentLimit) {
  
  /** Construct default config */
  public MotorConfig() {
    this(false, true, 1.0, 80.0, 40.0);
  }
}
