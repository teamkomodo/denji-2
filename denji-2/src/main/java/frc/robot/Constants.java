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
  public static final int XBOX_CONTROLLER_PORT = ;
  public static final double XBOX_JOYSTICK_THRESHOLD = 0.05D;
  public static final int JOYSTICK_PORT = ;
  public static final int BUTTONS_PORT = ; 
  public static final int SELECTOR_PORT = ; 

  // Position in rotations of the motor shaft before gearbox
  public static final double JOINT_MIN_POSITION = ; // Code stop
  public static final double JOINT_MAX_POSITION = ; // Code stop
  public static final double JOINT_STOW_POSITION = ;
  public static final double JOINT_DANGER_POSITION = ; // Min position where joint doesn't hit limelight mount

  public static final double JOINT_CUBE_GROUND_POSITION = ;
  public static final double JOINT_CUBE_LOW_POSITION = JOINT_STOW_POSITION;
  public static final double JOINT_CUBE_MID_POSITION = ;
  public static final double JOINT_CUBE_HIGH_POSITION = ;
  public static final double JOINT_CUBE_SHELF_POSITION = ;
  public static final double JOINT_CUBE_LAUNCH_POSITION = ;
  public static final double JOINT_CLAMP_POSITION = ;

  public static final double TOF_DISTANCE_METERS_CUBE = ;

  //Intake
  public static final int INTAKE_MOTOR_ID = ;
  public static final int INTAKE_THRESHOLD_CURRENT = ; // Amps

  //Launch
  public static final int LAUNCH_MOTOR_ID = ;
  public static final int LAUNCH_THRESHOLD_CURRENT = ; // Amps
    
  //Joint
  public static final int JOINT_MOTOR_ID = ;
  public static final int JOINT_ZERO_SWITCH_CHANNEL = ;
  public static final double JOINT_SLOW_MODE_MULTIPLIER = ;
}
