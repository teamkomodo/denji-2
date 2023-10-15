// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final double JOINT_MIN_POSITION = 1.5; // Code stop
  public static final double JOINT_MAX_POSITION = 19.0; // Code stop
  public static final double JOINT_STOW_POSITION = 15;

  public static final double JOINT_CUBE_GROUND_POSITION = 18.0;
  public static final double JOINT_CUBE_RELEASE_POSITION = 15.0;
  public static final double JOINT_CUBE_LAUNCH_POSITION = 2;
  public static final double JOINT_CLAMP_POSITION = 0;

  public static final double[] JOINT_POSITIONS_ORDERED = { // Order in array corresponds to selector position
    JOINT_STOW_POSITION,
    JOINT_CUBE_GROUND_POSITION,
    JOINT_CUBE_RELEASE_POSITION,
    JOINT_CUBE_LAUNCH_POSITION
};


  public static final double TOF_DISTANCE_METERS_CUBE = 0;

  //Intake
  public static final int INTAKE_MOTOR_ID = 15;
  public static final int INTAKE_THRESHOLD_CURRENT = 0; // Amps

  //Launch
  public static final int LAUNCH_MOTOR_ID = 13;
  public static final int LAUNCH_THRESHOLD_CURRENT = 0; // Amps
    
  //Joint
  public static final int JOINT_MOTOR_ID = 14;
  public static final int JOINT_ZERO_SWITCH_CHANNEL = 0;
  public static final double JOINT_SLOW_MODE_MULTIPLIER = 0;

  //Drivetrain
  public static final boolean FIELD_RELATIVE_DRIVE = true;
  public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
  public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.2;

  public static final double DRIVETRAIN_WIDTH = 0.5969;
  public static final double DRIVETRAIN_LENGTH = 0.5969;

  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
  public static final int FRONT_LEFT_STEER_MOTOR_ID = 5;
  public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;
  public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(293.1);

  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
  public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
  public static final int FRONT_RIGHT_STEER_ENCODER_ID = 11;
  public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(357.78);

  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
  public static final int BACK_LEFT_STEER_MOTOR_ID = 3;
  public static final int BACK_LEFT_STEER_ENCODER_ID = 9;
  public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(327.3);

  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
  public static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
  public static final int BACK_RIGHT_STEER_ENCODER_ID = 8;
  public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(170.75);

  public static final double WHEEL_DIAMETER = 0.1016;
  public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
  public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);

  public static final double MAX_LINEAR_VELOCITY = 3.0;
  public static final double MAX_LINEAR_ACCEL = 6.0;

  public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI;
  public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI;

  public static final double FALCON_500_NOMINAL_VOLTAGE = 12.0;
  public static final double TALON_FX_TICKS_PER_ROTATION = 2048.0;
  
}
