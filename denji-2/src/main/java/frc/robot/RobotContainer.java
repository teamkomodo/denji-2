// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.JointSubsystem;

import static frc.robot.Constants.*;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


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
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
  public final JointSubsystem jointSubsystem = new JointSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  private final CommandXboxController driverXBoxController = new CommandXboxController(XBOX_CONTROLLER_PORT);
  private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
  private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
  private final GenericHID selector = new GenericHID(SELECTOR_PORT);

  private final HashMap<String, Command> eventMap = new HashMap<>();

    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrainSubsystem::getPose,
        drivetrainSubsystem::resetPose,
        drivetrainSubsystem.getKinematics(),
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(1.5, 0.0, 0.0),
        drivetrainSubsystem::setModuleStates,
        eventMap, true, drivetrainSubsystem
    );

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Field", field2d);
    configureAuto();
    configureBindings();
  }

  private void configureAuto() {

    eventMap.put("place", Commands.sequence(
        jointSubsystem.releasePositionCommand(),
        Commands.waitSeconds(1.0),
        Commands.runEnd(() -> intakeSubsystem.setMotorDutyCycle(1.0), () -> intakeSubsystem.setMotorDutyCycle(0), intakeSubsystem).withTimeout(0.2),
        Commands.run(() -> jointSubsystem.setPosition(JOINT_STOW_POSITION), jointSubsystem).withTimeout(0.5))
    );

    autoChooser.addOption("No Auto", null);
    autoChooser.addOption("Place Mobility Open", autoBuilder.fullAuto(PathPlanner.loadPath("Place Mobility Open", new PathConstraints(2.0, 1.5))));
    SmartDashboard.putData("Auto Selection", autoChooser);
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
    
    //leftTrigger.whileTrue(jointSubsystem.grabPositionCommand(leftTrigger));
    //leftTrigger.whileTrue(Commands.run(() -> jointSubsystem.setMotorPercent(0.5)));
    //leftTrigger.onFalse(Commands.run(() -> jointSubsystem.setMotorPercent(0.0)));
    //rightTrigger.whileTrue(Commands.run(() -> jointSubsystem.setMotorPercent(-0.5)));
    //rightTrigger.onFalse(Commands.run(() -> jointSubsystem.setMotorPercent(0.0)));
    aButton.onTrue(jointSubsystem.grabPositionCommand());
    leftTrigger.whileTrue(jointSubsystem.launchPositionCommand());
    rightTrigger.whileTrue(jointSubsystem.releasePositionCommand());
    xButton.onTrue(jointSubsystem.launchPositionCommand());
    bButton.onTrue(jointSubsystem.releasePositionCommand());
    //leftTrigger.whileTrue(Commands.print("working..."));
    //leftTrigger.whileTrue(jointSubsystem.grabPositionCommand(() -> (false)));
    // Left Bumper starts outtake (spits cube out) 
    leftBumper.whileTrue(Commands.runEnd(() -> {
      intakeSubsystem.setMotorDutyCycle(1.0);
    }, () -> {
      intakeSubsystem.setMotorDutyCycle(0);
    }, intakeSubsystem));
    rightBumper.whileTrue(Commands.runEnd(() -> {
      intakeSubsystem.setMotorDutyCycle(-1.0);
    }, () -> {
      intakeSubsystem.setMotorDutyCycle(0);
    }, intakeSubsystem));
    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Drivetrain Commands
    // Drive command
    drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.joystickDriveCommand(
        () -> driverXBoxController.getLeftY() * MAX_LINEAR_VELOCITY,
        () -> driverXBoxController.getLeftX() * MAX_LINEAR_VELOCITY,
        () -> -driverXBoxController.getRightX() * MAX_ANGULAR_VELOCITY)); // Negative because counter clockwise (left/-x on controller) should be positive

    yButton.onTrue(Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();}));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}