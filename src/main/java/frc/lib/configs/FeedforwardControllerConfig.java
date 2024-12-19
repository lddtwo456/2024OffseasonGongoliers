package frc.lib.configs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** 
 * Feedforward config
 * 
 * @param kS voltage to overcome static friction
 * @param kG voltage to overcome gravity
 * @param kV voltage to overcome friction or drag that reduces velocity
 * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
 */
public record FeedforwardControllerConfig(
    double kS,
    double kG,
    double kV,
    double kA) {
  
  /** Construct default config */
  public FeedforwardControllerConfig() {
    this(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Construct simple feedforward without kG
   * 
   * @param kS voltage to overcome static friction
   * @param kV voltage to overcome friction or drag that reduces velocity
   * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
   */
  public FeedforwardControllerConfig(double kS, double kV, double kA) {
    this(kS, 0.0, kV, kA);
  }

  /**
   * Creates a simple motor feedforward using this config
   * 
   * @return a simple motor feedforward using this config
   */
  public SimpleMotorFeedforward createSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Creates an arm feedforward using this config
   * 
   * @return an arm feedforward using this config
   */
  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }
}
