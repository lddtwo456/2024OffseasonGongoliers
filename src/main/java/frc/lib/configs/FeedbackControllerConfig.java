package frc.lib.configs;

import edu.wpi.first.math.controller.PIDController;

/** 
 * Feedback controller config
 * 
 * @param kP proportional voltage
 * @param kI integral voltage
 * @param kD derivative voltage
 * @param continuous true enables continuous input
 * @param tolerance position tolerance
 * @param rateTolerance velocity tolerance
 */
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

  /**
   * Creates a new PID controller using this config
   * 
   * @return a new PID controller using this config
   */
  public PIDController createPIDController() {
    final PIDController pidController = new PIDController(kP, kI, kD);

    pidController.setTolerance(tolerance, rateTolerance);

    if (continuous) {
      pidController.enableContinuousInput(-0.5, 0.5);
    }

    return pidController;
  }
}
