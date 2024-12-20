package frc.lib.controllers.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Swerve module interface */
public interface SwerveModule {
  
  /** 
   * Returns the swerve module's state
   * 
   * @return the swerve module's state
   */
  public SwerveModuleState getState();

  /**
   * Returns the swerve module's target state
   * 
   * @return the swerve module's target state
   */
  public SwerveModuleState getSetpoint();

  /**
   * Sets the swerve module's target state
   * 
   * @param setpoint the swerve module's target state
   * @param lazy if true, optimize the swerve setpoint
   */
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy);

  /**
   * Returns the swerve module's current position
   * 
   * @return the swerve module's current position
   */
  public SwerveModulePosition getPosition();
}
