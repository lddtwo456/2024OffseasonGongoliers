package frc.lib.configs;

/** Easier way to construct mechanism configs */
public class SimpleMechanismConfigBuilder {
  
  private AbsoluteEncoderConfig absoluteEncoderConfig;

  private FeedbackControllerConfig feedbackControllerConfig;
  
  private FeedforwardControllerConfig feedforwardControllerConfig;

  private MotionProfileConfig motionProfileConfig;

  private MotorConfig motorConfig;

  private SimpleMechanismConfigBuilder(
      AbsoluteEncoderConfig absoluteEncoderConfig,
      FeedbackControllerConfig feedbackControllerConfig,
      FeedforwardControllerConfig feedforwardControllerConfig,
      MotionProfileConfig motionProfileConfig,
      MotorConfig motorConfig) {
    this.absoluteEncoderConfig = absoluteEncoderConfig;
    this.feedbackControllerConfig = feedbackControllerConfig;
    this.feedforwardControllerConfig = feedforwardControllerConfig;
    this.motionProfileConfig = motionProfileConfig;
    this.motorConfig = motorConfig;
  }

  public static SimpleMechanismConfigBuilder defaults() {
    return new SimpleMechanismConfigBuilder(
      new AbsoluteEncoderConfig(), 
      new FeedbackControllerConfig(), 
      new FeedforwardControllerConfig(), 
      new MotionProfileConfig(), 
      new MotorConfig());
  }

  public void withAbsoluteEncoderConfig(AbsoluteEncoderConfig config) {
    this.absoluteEncoderConfig = config;
  }

  public void withFeedbackControllerConfig(FeedbackControllerConfig config) {
    this.feedbackControllerConfig = config;
  }

  public void withFeedforwardControllerConfig(FeedforwardControllerConfig config) {
    this.feedforwardControllerConfig = config;
  }

  public void withMotionProfileConfig(MotionProfileConfig config) {
    this.motionProfileConfig = config;
  }

  public void withMotorConfig(MotorConfig config) {
    this.motorConfig = config;
  }

  public MechanismConfig build() {
    return new MechanismConfig(
      this.absoluteEncoderConfig,
      this.feedbackControllerConfig,
      this.feedforwardControllerConfig,
      this.motionProfileConfig,
      this.motorConfig
    );
  }
}
