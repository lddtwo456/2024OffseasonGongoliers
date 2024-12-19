package frc.lib.configs;

/** Motion profile config */
public record MotionProfileConfig(
    double maxVelocity,
    double maxAcceleration) {
  
  /** Construct default config */
  public MotionProfileConfig() {
    this(0.0, 0.0);
  }
}
