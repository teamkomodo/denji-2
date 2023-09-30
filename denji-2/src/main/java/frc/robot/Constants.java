// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Controls
  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final double XBOX_JOYSTICK_THRESHOLD = 0.05D;
  public static final int JOYSTICK_PORT = 0;
  public static final int BUTTONS_PORT = 0; 
  public static final int SELECTOR_PORT = 0; 

  // Position in rotations of the motor shaft before gearbox
  public static final double JOINT_MIN_POSITION = 0; // Code stop
  public static final double JOINT_MAX_POSITION = 0; // Code stop
  public static final double JOINT_STOW_POSITION = 0;
  public static final double JOINT_DANGER_POSITION = 0; // Min position where joint doesn't hit limelight mount

  public static final double JOINT_CUBE_GROUND_POSITION = 0;
  public static final double JOINT_CUBE_LOW_POSITION = JOINT_STOW_POSITION;
  public static final double JOINT_CUBE_MID_POSITION = 0;
  public static final double JOINT_CUBE_HIGH_POSITION = 0;
  public static final double JOINT_CUBE_SHELF_POSITION = 0;
  public static final double JOINT_CUBE_LAUNCH_POSITION = 0;
  public static final double JOINT_CLAMP_POSITION = 0;

  public static final double TOF_DISTANCE_METERS_CUBE = 0;

  //Intake
  public static final int INTAKE_MOTOR_ID = 0;
  public static final int INTAKE_THRESHOLD_CURRENT = 0; // Amps

  //Launch
  public static final int LAUNCH_MOTOR_ID = 0;
  public static final int LAUNCH_THRESHOLD_CURRENT = 0; // Amps
    
  //Joint
  public static final int JOINT_MOTOR_ID = 0;
  public static final int JOINT_ZERO_SWITCH_CHANNEL = 0;
  public static final double JOINT_SLOW_MODE_MULTIPLIER = 0;

  //Drivetrain
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(294.1);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(356.48);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(327.3);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(168.65);
}
