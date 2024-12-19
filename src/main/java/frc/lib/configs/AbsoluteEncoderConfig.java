package frc.lib.configs;

import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * Absolute encoder config 
 * 
 * @param ccwPositive true means positive voltages turn the motor ccw
 * @param sensorToMechRatio ratio of sensor rotations to mechanism rotations
 * @param offset starting offset for encoder
 */
public record AbsoluteEncoderConfig(
    boolean ccwPositive, 
    double sensorToMechRatio, 
    Rotation2d offset) {
  
  /** Construct default config */
  public AbsoluteEncoderConfig() {
    this(true, 1.0, new Rotation2d());
  }
}
