// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is
   * useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
      * SdsModuleConfigurations.MK4_L1.getDriveReduction() * SdsModuleConfigurations.MK4_L1.getWheelDiameter()
      * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
      / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    // Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
    // and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
    // Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
    // class.

    // By default we will use Falcon 500s in standard configuration. But if you use
    // a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk3SwerveModuleHelper.GearRatio.STANDARD,
        // This is the ID of the drive motor
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case,
        // zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);

    backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);

    backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    if (navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(navx.getFusedHeading());
    }
    //
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public Rotation2d getGyroYaw() {
    return getGyroYawRaw();
  }

  private Rotation2d getGyroYawRaw() {
    if (navx.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(navx.getFusedHeading());
    }
    return Rotation2d.fromDegrees(navx.getYaw());
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    frontLeftModule.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());
}

  public void drive(double forward, double right, double rotation, boolean fieldRelative) {
    ChassisSpeeds speeds = new ChassisSpeeds(forward, right, rotation);
        if (fieldRelative) {
            setSwerveModuleStates(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw().times(-1))));
            return;
        }
        setSwerveModuleStates(kinematics.toSwerveModuleStates(speeds));
  }
}