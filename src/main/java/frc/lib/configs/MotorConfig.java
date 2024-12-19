package frc.lib.configs;

/** Motor config */
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
