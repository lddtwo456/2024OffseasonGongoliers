package frc.lib.config;

import java.util.Objects;

import edu.wpi.first.math.geometry.Rotation2d;

public record AbsoluteEncoderConfig(
    boolean ccwPositive, double sensorToMechanismRatio, Rotation2d offset) {
  
  public AbsoluteEncoderConfig {
    Objects.requireNonNull(ccwPositive);
    Objects.requireNonNull(sensorToMechanismRatio);
    Objects.requireNonNull(offset);
  }

  public static final class AbsoluteEncoderConfigBuilder {
    /** Interpret ccw rotation on the encoder as positive velocity */
    private boolean ccwPositive;

    /** Ratio between absolute encoder motion and actual mechanism motion */
    private double sensorToMechanismRatio;

    /** Offset between absolute encoder reading and actual mechanism position */
    private Rotation2d offset;

    public static AbsoluteEncoderConfigBuilder defaults() {
      return new AbsoluteEncoderConfigBuilder(true, 1.0, new Rotation2d());
    }

    public static AbsoluteEncoderConfigBuilder from(AbsoluteEncoderConfig absoluteEncoderConfig) {
      return new AbsoluteEncoderConfigBuilder(
        absoluteEncoderConfig.ccwPositive,
        absoluteEncoderConfig.sensorToMechanismRatio,
        absoluteEncoderConfig.offset);
    }

    private AbsoluteEncoderConfigBuilder(
        boolean ccwPositive, double sensorToMechanismRatio, Rotation2d offset) {

      this.ccwPositive = ccwPositive;
      this.sensorToMechanismRatio = sensorToMechanismRatio;
      this.offset = offset;
    }

    public AbsoluteEncoderConfigBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public AbsoluteEncoderConfigBuilder sensorToMechanismRatio(double sensorToMechanismRatio) {
      this.sensorToMechanismRatio = sensorToMechanismRatio;
      return this;
    }

    public AbsoluteEncoderConfigBuilder offset(Rotation2d offset) {
      this.offset = offset;
      return this;
    }

    public AbsoluteEncoderConfig build() {
      return new AbsoluteEncoderConfig(this.ccwPositive, this.sensorToMechanismRatio, this.offset);
    }
  }
}
