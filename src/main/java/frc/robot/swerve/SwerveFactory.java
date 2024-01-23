package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.MotorCurrentLimits;
import frc.robot.Robot;
import frc.robot.swerve.SwerveConstants.DriveMotorConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** Helper class for creating hardware for the swerve subsystem. */
public class SwerveFactory {

  /**
   * Creates a swerve module.
   *
   * @return a swerve module.
   */
  public static SwerveModuleIO createModule(SwerveModuleConfig config) {
    if (Robot.isReal()) return new SwerveModuleIOCustom(config);

    return new SwerveModuleIOCustom(config);
  }

  /**
   * Creates an azimuth encoder.
   *
   * @return an azimuth encoder.
   */
  public static AzimuthEncoderIO createAzimuthEncoder(SwerveModuleConfig config) {
    if (Robot.isReal())
      return new AzimuthEncoderIOCANcoder(config.moduleCAN().azimuth(), config.offset());

    return new AzimuthEncoderIOSim();
  }

  /**
   * Creates an azimuth encoder configuration.
   *
   * @return an azimuth encoder configuration.
   */
  public static CANcoderConfiguration createAzimuthEncoderConfig() {
    CANcoderConfiguration azimuthEncoderConfig = new CANcoderConfiguration();

    return azimuthEncoderConfig;
  }

  /**
   * Creates a steer motor.
   *
   * @return a steer motor.
   */
  public static SteerMotorIO createSteerMotor(SwerveModuleConfig config) {
    if (Robot.isReal())
      return new SteerMotorIOTalonFXPIDF(config.moduleCAN().steer(), config.moduleCAN().azimuth());

    return new SteerMotorIOSim();
  }

  /**
   * Creates a steer motor configuration.
   *
   * @return a steer motor configuration.
   */
  public static TalonFXConfiguration createSteerMotorConfig() {
    TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();

    steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // TODO Copied from Nutrons 2023 release
    steerMotorConfig.CurrentLimits =
        new MotorCurrentLimits(0.0, 40.0, 3.0, 1.0).asCurrentLimitsConfigs();

    return steerMotorConfig;
  }

  /**
   * Creates a drive motor.
   *
   * @return a drive motor.
   */
  public static DriveMotorIO createDriveMotor(SwerveModuleConfig config) {
    if (Robot.isReal()) return new DriveMotorIOTalonFXPID(config.moduleCAN().drive());

    return new DriveMotorIOSim();
  }

  /**
   * Creates a drive motor configuration.
   *
   * @return a drive motor configuration.
   */
  public static TalonFXConfiguration createDriveMotorConfig() {
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    driveMotorConfig.Feedback.SensorToMechanismRatio = MK4iConstants.DRIVE_GEARING;

    driveMotorConfig.CurrentLimits = DriveMotorConstants.CURRENT_LIMITS.asCurrentLimitsConfigs();

    return driveMotorConfig;
  }
}
