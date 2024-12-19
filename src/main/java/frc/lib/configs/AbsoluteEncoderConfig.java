package frc.lib.configs;

import edu.wpi.first.math.geometry.Rotation2d;

/** Absolute encoder config */
public record AbsoluteEncoderConfig(
    boolean ccwPositive, 
    double sensorToMechRatio, 
    Rotation2d offset) {
  
  /** Construct default config */
  public AbsoluteEncoderConfig() {
    this(true, 1.0, new Rotation2d());
  }
}
