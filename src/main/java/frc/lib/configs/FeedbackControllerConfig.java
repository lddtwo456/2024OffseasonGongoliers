package frc.lib.configs;

/** Feedback controller config */
public record FeedbackControllerConfig(
    double kP,
    double kI,
    double kD,
    boolean continuous,
    double tolerance,
    double rateTolerance) {
  
  /** Construct default config */
  public FeedbackControllerConfig() {
    this(0.0, 0.0, 0.0, false, 0.0, 0.0);
  }
}
