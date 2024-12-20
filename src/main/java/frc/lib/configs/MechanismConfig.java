package frc.lib.configs;

import frc.lib.configs.AbsoluteEncoderConfig.AbsoluteEncoderBuilder;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;

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

  /** Easier more modular way to construct mechanism configs */
  public static class MechanismBuilder {
    private AbsoluteEncoderConfig absoluteEncoderConfig;
    private FeedbackControllerConfig feedbackControllerConfig;
    private FeedforwardControllerConfig feedforwardControllerConfig;
    private MotionProfileConfig motionProfileConfig;
    private MotorConfig motorConfig;

    private MechanismBuilder(
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

    public static MechanismBuilder defaults() {
      return new MechanismBuilder(
        AbsoluteEncoderBuilder.defaults().build(),
        FeedbackControllerBuilder.defaults().build(),
        FeedforwardControllerBuilder.defaults().build(),
        MotionProfileBuilder.defaults().build(),
        MotorBuilder.defaults().build());
    }

    public static MechanismBuilder edit(MechanismConfig config) {
      return new MechanismBuilder(
        config.absoluteEncoderConfig(), 
        config.feedbackControllerConfig(), 
        config.feedforwardControllerConfig(), 
        config.motionProfileConfig(), 
        config.motorConfig());
    }

    public MechanismBuilder absoluteEncoderConfig(AbsoluteEncoderConfig absoluteEncoderConfig) {
      this.absoluteEncoderConfig = absoluteEncoderConfig;
      return this;
    }

    public MechanismBuilder feedbackControllerConfig(FeedbackControllerConfig feedbackControllerConfig) {
      this.feedbackControllerConfig = feedbackControllerConfig;
      return this;
    }

    public MechanismBuilder feedforwardControllerConfig(FeedforwardControllerConfig feedforwardControllerConfig) {
      this.feedforwardControllerConfig = feedforwardControllerConfig;
      return this;
    }

    public MechanismBuilder motionProfileConfig(MotionProfileConfig motionProfileConfig) {
      this.motionProfileConfig = motionProfileConfig;
      return this;
    }

    public MechanismBuilder motorConfig(MotorConfig motorConfig) {
      this.motorConfig = motorConfig;
      return this;
    }

    public MechanismConfig build() {
      return new MechanismConfig(
        this.absoluteEncoderConfig, 
        this.feedbackControllerConfig, 
        this.feedforwardControllerConfig, 
        this.motionProfileConfig, 
        this.motorConfig);
    }
  }
}
