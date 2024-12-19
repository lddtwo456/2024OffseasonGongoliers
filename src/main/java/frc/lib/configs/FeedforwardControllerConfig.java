package frc.lib.configs;

/** Feedforward config */
public record FeedforwardControllerConfig(
    double kS,
    double kG,
    double kV,
    double kA) {
  
  /** Construct default config */
  public FeedforwardControllerConfig() {
    this(0.0, 0.0, 0.0, 0.0);
  }
}
