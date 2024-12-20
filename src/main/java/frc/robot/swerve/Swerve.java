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
import frc.lib.configs.SimpleMechanismConfigBuilder;
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
    
  }
}
