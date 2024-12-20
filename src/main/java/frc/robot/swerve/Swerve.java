package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.FeedbackControllerConfig;
import frc.lib.configs.FeedforwardControllerConfig;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MotorConfig;
import frc.lib.configs.SimpleMechanismConfigBuilder;

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
    SimpleMechanismConfigBuilder.defaults()
      .withMotorConfig(
        new MotorConfig(false, false, 150.0/7.0, 20.0, 40.0))
      .withFeedforwardControllerConfig(
        new FeedforwardControllerConfig(1.0, 0.0, 0.0))
      .withFeedbackControllerConfig(
        new FeedbackControllerConfig(54.0, 0.0, 0.16, false, Units.degreesToRotations(1.0), 0.0))
      .build();

  /** Drive motor config */
  private final MechanismConfig driveConfig =
    SimpleMechanismConfigBuilder.defaults()
      .withMotorConfig(
        new MotorConfig(false, false, 6.75, 50.0, 40.0))
      .withFeedforwardControllerConfig(
        new FeedforwardControllerConfig(0.14, 0.725, 0.0))
      .withFeedbackControllerConfig(
        new FeedbackControllerConfig(0.75, 0.0, 0.0, false, 0.0, 0.0))
      .build();
}
