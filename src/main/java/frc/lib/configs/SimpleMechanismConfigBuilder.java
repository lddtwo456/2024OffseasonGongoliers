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

  public SimpleMechanismConfigBuilder withAbsoluteEncoderConfig(AbsoluteEncoderConfig config) {
    this.absoluteEncoderConfig = config;
    return this;
  }

  public SimpleMechanismConfigBuilder withFeedbackControllerConfig(FeedbackControllerConfig config) {
    this.feedbackControllerConfig = config;
    return this;
  }

  public SimpleMechanismConfigBuilder withFeedforwardControllerConfig(FeedforwardControllerConfig config) {
    this.feedforwardControllerConfig = config;
    return this;
  }

  public SimpleMechanismConfigBuilder withMotionProfileConfig(MotionProfileConfig config) {
    this.motionProfileConfig = config;
    return this;
  }

  public SimpleMechanismConfigBuilder withMotorConfig(MotorConfig config) {
    this.motorConfig = config;
    return this;
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
