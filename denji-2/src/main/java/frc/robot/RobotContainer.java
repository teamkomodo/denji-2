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
import frc.robot.subsystems.IntakeSubsystem;



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
  public final JointSubsystem jointSubsystem = new JointSubsystem(mainTab);
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LEDStripSubsystem ledStripSubsystem = new LEDStripSubsystem();

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

    Trigger rightDriverJoystickButton = new JoystickButton(driverJoystick, 1);
    Trigger leftDriverJoystickButton = new JoystickButton(driverJoystick, 2);

    Trigger toggleSwitch1 = new JoystickButton(driverButtons, 1);
    Trigger toggleSwitch2 = new JoystickButton(driverButtons, 2);
    Trigger toggleSwitch3 = new JoystickButton(driverButtons, 3);

    Trigger whiteButton = new JoystickButton(driverButtons, 4);
    Trigger yellowButton = new JoystickButton(driverButtons, 5);
    Trigger blueButton = new JoystickButton(driverButtons, 7);
    Trigger blueTriButton = new JoystickButton(driverButtons, 6);

    //Xbox A Stows, Xbox B puts in low node, Xbox X puts in mid node, Xbox Y puts in high node
    aButton.onTrue(new StowCommand(jointSubsystem));
    bButton.onTrue(new LowNodeCommand(jointSubsystem, cubeMode));
    xButton.onTrue(new MidNodeCommand(jointSubsystem, cubeMode));
    yButton.onTrue(new HighNodeCommand(jointSubsystem, cubeMode));
    
    //Xbox Start grabs from shelf, Xbox Press Right Joystick sets arm on ground
    startButton.onTrue(
      new ShelfCommand(jointSubsystem, cubeMode));
    rightJoystickDown.onTrue(new GroundCommand(jointSubsystem, cubeMode));
    
    //DrivStat White Button does
    whiteButton.whileTrue(Commands.parallel(jointSubsystem.zeroCommand()));
    
    // Right Bumper starts intake (takes cube in)
    rightBumper.whileTrue(new IntakePieceCommand(intakeSubsystem, jointSubsystem, ledStripSubsystem));

    // Left Bumper starts outtake (spits cube out) 
    leftBumper.whileTrue(Commands.runEnd(() -> {
        ledStripSubsystem.setPattern(-0.01); // Color 1 Larson Scanner
        intakeSubsystem.setMotorDutyCycle(1.0);
    }, () -> {
        intakeSubsystem.setMotorDutyCycle(0);
    }, intakeSubsystem, ledStripSubsystem));

    // If the Joint is beyond the dangerous threshold, move it to the danger position 
    jointSubsystem.setDefaultCommand(Commands.run(() -> {
      if (elevatorSubsystem.getPosition() > ELEVATOR_JOINT_DANGER_THRESHOLD
          && jointSubsystem.getPosition() < JOINT_DANGER_POSITION) {
        jointSubsystem.setPosition(JOINT_DANGER_POSITION);
      }
    }, jointSubsystem));

  // Right Trigger moves joint up, then holds it
  rightTrigger.whileTrue(Commands.run(
    () -> jointSubsystem.setMotorPercent(-driverXBoxController.getRightTriggerAxis()),
    jointSubsystem));
  rightTrigger.onFalse(jointSubsystem.holdPositionCommand());

  // Left Trigger moves joint down, then holds it
  leftTrigger.whileTrue(Commands.run(
    () -> jointSubsystem.setMotorPercent(driverXBoxController.getLeftTriggerAxis()),
    jointSubsystem));
  leftTrigger.onFalse(jointSubsystem.holdPositionCommand());
  
  // If DrivStat switch 2 is on, disable limits
  toggleSwitch2.onTrue(jointSubsystem.disableLimitsCommand()).onFalse(jointSubsystem.enableLimitsCommand());

  // If DrivStat switch 3 is on, enable slow mode
  toggleSwitch3.onTrue(jointSubsystem.enableSlowModeCommand()).onFalse(jointSubsystem.disableSlowModeCommand());

  }
}
