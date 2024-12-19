package frc.lib.configs;

/** 
 * Mechanism config
 * 
 * @param absoluteEncoderConfig absolute encoder config
 * @param feedbackControllerConfig feedback controller config
 * @param feedforwardControllerConfig feedforward controller config
 * @param motionProfileConfig motion profile config
 * @param motorConfig motor config
 */
public record MechanismConfig(
    AbsoluteEncoderConfig absoluteEncoderConfig,
    FeedbackControllerConfig feedbackControllerConfig,
    FeedforwardControllerConfig feedforwardControllerConfig,
    MotionProfileConfig motionProfileConfig,
    MotorConfig motorConfig) {
  
  /** Construct default config */
  public MechanismConfig() {
    this(
      new AbsoluteEncoderConfig(),
      new FeedbackControllerConfig(),
      new FeedforwardControllerConfig(),
      new MotionProfileConfig(),
      new MotorConfig()
    );
  }
}
