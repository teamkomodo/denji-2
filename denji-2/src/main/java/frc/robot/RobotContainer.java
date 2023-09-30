// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;


/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
  private final Field2d field2d = new Field2d();
  
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Operator Information");
  
  
  // Subsystem definitions should be public for auto reasons
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LaunchSubsystem launchSubsystem = new LaunchSubsystem();
  
  private final CommandXboxController driverXBoxController = new CommandXboxController(
  XBOX_CONTROLLER_PORT);
  private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
  private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
  private final GenericHID selector = new GenericHID(SELECTOR_PORT);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Field", field2d);
    configureBindings();
  }
  
  private void configureBindings() {
    
    // Define all of the input devices on the Xbox Controller and Driver Station
    Trigger aButton = driverXBoxController.a();
    Trigger bButton = driverXBoxController.b();
    Trigger xButton = driverXBoxController.x();
    Trigger yButton = driverXBoxController.y();
    
    Trigger leftJoystickDown = driverXBoxController.leftStick();
    Trigger rightJoystickDown = driverXBoxController.rightStick();
    
    Trigger backButton = driverXBoxController.back();
    Trigger startButton = driverXBoxController.start();
    
    Trigger leftBumper = driverXBoxController.leftBumper();
    Trigger rightBumper = driverXBoxController.rightBumper();
    
    Trigger leftJoystickY = driverXBoxController
    .axisGreaterThan(XboxController.Axis.kLeftY.value, XBOX_JOYSTICK_THRESHOLD)
    .or(driverXBoxController.axisLessThan(XboxController.Axis.kLeftY.value, -XBOX_JOYSTICK_THRESHOLD));
    
    Trigger rightJoystickY = driverXBoxController
    .axisGreaterThan(XboxController.Axis.kRightY.value, XBOX_JOYSTICK_THRESHOLD)
    .or(driverXBoxController.axisLessThan(XboxController.Axis.kRightY.value, -XBOX_JOYSTICK_THRESHOLD));
    
    Trigger leftTrigger = driverXBoxController.leftTrigger();
    Trigger rightTrigger = driverXBoxController.rightTrigger();
    
    // Left Bumper starts outtake (spits cube out) 
    leftBumper.whileTrue(Commands.runEnd(() -> {
      launchSubsystem.setMotorDutyCycle(1.0);
    }, () -> {
      launchSubsystem.setMotorDutyCycle(0);
    }, launchSubsystem));
    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Drivetrain Commands
    // Drive command
    drivetrainSubsystem.setDefaultCommand(
      Commands.run(
        () -> drivetrainSubsystem.drive(
          -driverJoystick.getRawAxis(1)
          * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          -driverJoystick.getRawAxis(0)
          * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          driverJoystick.getRawAxis(2)
          * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
          true), drivetrainSubsystem));
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
