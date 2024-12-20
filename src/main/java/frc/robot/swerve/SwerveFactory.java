package frc.robot.swerve;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerTalonFXSteer;
import frc.lib.controllers.swerve.SwerveModule;
import frc.lib.controllers.swerve.SwerveModuleTalonFXCANcoder;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityControllerTalonFXPIDF;
import frc.robot.Robot;

/** Creates swerve hardware */
public class SwerveFactory {
  
  private static SwerveModule createModuleO(
      CAN steer,
      CAN azimuth,
      CAN drive,
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return new SwerveModuleTalonFXCANcoder(
      createSteerMotor(steer, azimuth, steerConfig),
      createDriveMotor(drive, driveConfig),
      wheelCircumference);
  }

  /**
   * Creates a steer motor
   * 
   * @param steer steer motor CAN
   * @param azimuth azimuth encoder CAN
   * @param config mechanism conrig
   * @return a steer motor
   */
  private static PositionController createSteerMotor(
      CAN steer, CAN azimuth, MechanismConfig config) {
    
    if (Robot.isReal()) {
      return new PositionControllerTalonFXSteer(steer, azimuth, config, false);
    }

    //TODO sim TalonFX steer motor
    return new PositionControllerTalonFXSteer(steer, azimuth, config, false);
  }

  /**
   * Creates a drive motor
   * 
   * @param drive drive motor CAN
   * @param config mechanism config
   * @return a drive motor
   */
  private static VelocityController createDriveMotor(CAN drive, MechanismConfig config) {
    if (Robot.isReal()) {
      return new VelocityControllerTalonFXPIDF(drive, config, false);
    }

    //TODO sim TalonFX drive motor
    return new VelocityControllerTalonFXPIDF(drive, config, false);
  }
}
