package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.FeedbackControllerConfig;
import frc.lib.configs.FeedforwardControllerConfig;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MotionProfileConfig;
import frc.lib.configs.MotorConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;

/** Swerve subsystem */
public class Swerve extends SubsystemBase {
  
  /** Swerve subsystem singleton */
  private static Swerve instance = null;

  /** Swerve modules */
  private final SwerveModule[] swerves = new SwerveModule[4];

  /** Swerve kinematics */
  private final SwerveDriveKinematics swerveKinematics;

  /** Steer motor config */
  private final MechanismConfig steerConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(150.0 / 7.0)
          .statorCurrentLimit(20.0)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kS(0.205)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .continuous(true)
          .kP(54.0)
          .kD(0.15)
          .tolerance(Units.degreesToRotations(1.0))
          .build())
    .build();

  /** Drive motor config */
  private final MechanismConfig driveConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(6.75)
          .statorCurrentLimit(50.0)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kS(0.34)
          .kV(0.725)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.75)
          .build())
      .build();

  /** Wheel circumference */
  private final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

  /** Translation motion profile config */
  private final MotionProfileConfig translationMotionProfileConfig =
    new MotionProfileConfig(4.5, 18);

  /** Rotation motion profile config */
  private final MotionProfileConfig rotationMotionProfileConfig =
    new MotionProfileConfig(1.0, 0.0);
  
  /** Initializes the swerve subsystem and configures swerve hardware */
  private Swerve() {
    swerves[0] =
      SwerveFactory.createNorthWestModule(steerConfig, driveConfig, wheelCircumference);
    swerves[1] =
      SwerveFactory.createNorthEastModule(steerConfig, driveConfig, wheelCircumference);
    swerves[2] =
      SwerveFactory.createSouthEastModule(steerConfig, driveConfig, wheelCircumference);
    swerves[3] =
      SwerveFactory.createSouthWestModule(steerConfig, driveConfig, wheelCircumference);

    swerveKinematics = 
      new SwerveDriveKinematics(
        SwerveFactory.getNorthWestModuleTranslation(),
        SwerveFactory.getNorthEastModuleTranslation(),
        SwerveFactory.getSouthEastModuleTranslation(),
        SwerveFactory.getSouthWestModuleTranslation());
  }

  /** 
   * Returns the swerve subsystem instance, creates a new instance if instance is null (singleton)
   * 
   * @return the swerve subsystem instance
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  /**
   * Returns swerve kinematics
   * 
   * @return swerve kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * Returns the module states
   * 
   * @return the module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleStates[i] = swerves[i].getState();
    }

    return moduleStates;
  }

  /**
   * Returns the module setpoints
   * 
   * @return the module setpoints
   */
  public SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] moduleSetpoints = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleSetpoints[i] = swerves[i].getSetpoint();
    }

    return moduleSetpoints;
  }

  /**
   * Returns the module positions
   * 
   * @return the module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = swerves[i].getPosition();
    }

    return modulePositions;
  }
}
