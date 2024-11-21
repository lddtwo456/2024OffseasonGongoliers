package frc.robot;

import java.util.EnumSet;
import java.util.Set;

/** Constants of the entire robot */
public class RobotConstants {

  /** Periodic calls per second */
  public static final double PERIODIC_RATE = 50;

  /** Length of each robot periodic call in seconds */
  public static final double PERIODIC_DURATION = 1 / PERIODIC_RATE;

  /** Subsystems */
  public enum Subsystem {
    SWERVE,
    ODOMETRY,
  }

  /** Real subsystems */
  public static final Set<Subsystem> REAL_SUBSYSTEMS = 
    EnumSet.of(
      Subsystem.SWERVE, Subsystem.ODOMETRY
    );
}
