package frc.robot.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.sensor.GyroscopeIO;
import frc.lib.sensor.GyroscopeIO.GyroscopeIOValues;
import frc.robot.odometry.limeilghts.LimelightsIO;
import frc.robot.odometry.targetting.FieldTargetsSupplier;
import frc.robot.odometry.targetting.FieldTargetsSupplierBlue;
import frc.robot.odometry.targetting.FieldTargetsSupplierRed;
import frc.robot.swerve.Swerve;
import java.util.function.Supplier;

/** Odometry subsystem. */
public class Odometry extends Subsystem {

  /** Odometry subsystem singleton. */
  private static Odometry instance = null;

  /** Gyroscope. */
  private final GyroscopeIO gyroscope;

  /** Gyroscope values. */
  private final GyroscopeIOValues gyroscopeValues = new GyroscopeIOValues();

  /** Supplies swerve module positions. */
  private final Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier;

  /** Supplies swerve velocity. */
  private final Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier;

  /** Pose estimator using the swerve drive. */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Limelight */
  private final LimelightsIO limelights;

  /** Field target position/angle supplier */
  private final FieldTargetsSupplier targetSupplier;

  /** Field. */
  private final Field2d field;

  /** Initializes the odometry subsystem and configures odometry hardware. */
  private Odometry() {
    gyroscope = OdometryFactory.createGyroscope(this);
    gyroscope.configure();

    swerveModulePositionsSupplier = () -> Swerve.getInstance().getModulePositions();

    swerveChassisSpeedsSupplier = () -> Swerve.getInstance().getChassisSpeeds();

    gyroscope.update(gyroscopeValues);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Swerve.getInstance().getKinematics(),
            Rotation2d.fromRotations(gyroscopeValues.yawRotations),
            swerveModulePositionsSupplier.get(),
            new Pose2d());

    limelights = OdometryFactory.createLimelights(new String[] {"limelight"}, poseEstimator);

    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      targetSupplier = new FieldTargetsSupplierBlue();
    } else {
      targetSupplier = new FieldTargetsSupplierRed();
    }

    field = new Field2d();
  }

  /**
   * Gets the instance of the odometry subsystem.
   *
   * @return the instance of the odometry subsystem.
   */
  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }

    return instance;
  }

  @Override
  public void periodic() {
    gyroscope.update(gyroscopeValues);

    poseEstimator.update(
        Rotation2d.fromRotations(gyroscopeValues.yawRotations),
        swerveModulePositionsSupplier.get());

    limelights.update(gyroscopeValues, this.getFieldRelativeHeading());

    field.setRobotPose(getPosition());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout position = Telemetry.addColumn(tab, "Position");

    position.addDouble("X (m)", () -> getPosition().getX());
    position.addDouble("Y (m)", () -> getPosition().getY());
    position.addDouble("Field Rotation (deg)", () -> getFieldRelativeHeading().getDegrees());
    position.addDouble("Driver Rotation (deg)", () -> getDriverRelativeHeading().getDegrees());

    ShuffleboardLayout velocity = Telemetry.addColumn(tab, "Velocity");

    velocity.addDouble("X Velocity (mps)", () -> getVelocity().dx);
    velocity.addDouble("Y Velocity (mps)", () -> getVelocity().dy);
    velocity.addDouble(
        "Rotation Velocity (dps)", () -> Units.radiansToDegrees((getVelocity().dtheta)));

    ShuffleboardLayout field = Telemetry.addColumn(tab, "Field");

    field.add("Field", this.field);
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return the position of the robot on the field.
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the rotation of the robot on the field where zero is away from the blue alliance wall.
   *
   * @return the rotation of the robot on the field where zero is away from the blue alliance wall.
   */
  public Rotation2d getFieldRelativeHeading() {
    // TODO Doesn't obey WPILib's coordinate system

    return getPosition().getRotation();
  }

  /**
   * Returns the rotation of the robot on the field where zero is away from the driver regardless of
   * alliance.
   *
   * @return the rotation of the robot on the field where zero is away from the driver regardless of
   *     alliance.
   */
  public Rotation2d getDriverRelativeHeading() {
    return Rotation2d.fromRotations(gyroscopeValues.yawRotations);
  }

  /**
   * Returns the angle to the alliance speaker
   * 
   * @return the angle to the alliance speaker
   */
  public double getYawToSpeaker() {
    return targetSupplier.getYawToSpeaker(Odometry.getInstance().getPosition());
  }

  /**
   * Sets the position of the robot on the field.
   *
   * @param position the position of the robot on the field.
   */
  public void setPosition(Pose2d position) {
    poseEstimator.resetPosition(
        Rotation2d.fromRotations(gyroscopeValues.yawRotations),
        swerveModulePositionsSupplier.get(),
        position);
  }

  /**
   * Sets the rotation of the robot on the field.
   *
   * @param rotation the rotation of the robot on the field.
   */
  public void setRotation(Rotation2d rotation) {
    Pose2d position = getPosition();

    setPosition(new Pose2d(position.getTranslation(), rotation));
  }

  /**
   * Zeroes the driver-relative rotation of the robot.
   *
   * @return a command that zeroes the driver-relative rotation of the robot.
   */
  public Command tare() {
    return Commands.runOnce(
        () -> {
          gyroscope.setYaw(0.0);
        });
  }

  /**
   * Gets the velocity of the robot on the field.
   *
   * @return the velocity of the robot on the field.
   */
  public Twist2d getVelocity() {
    // Guards against calling before the swerve pose estimator is
    // intialized
    if (poseEstimator == null) return new Twist2d();

    ChassisSpeeds chassisSpeeds = swerveChassisSpeedsSupplier.get();

    Rotation2d rotation = getPosition().getRotation();

    double xVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getCos()
            - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    double yVelocityMetersPerSecond =
        chassisSpeeds.vxMetersPerSecond * rotation.getSin()
            + chassisSpeeds.vyMetersPerSecond * rotation.getCos();

    return new Twist2d(
        xVelocityMetersPerSecond, yVelocityMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
  }
}
