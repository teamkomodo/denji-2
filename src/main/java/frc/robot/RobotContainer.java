// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;


/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
	private final Field2d field2d = new Field2d();
	
	
	// Subsystem definitions should be public for auto reasons
	private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
	
	private final CommandXboxController driverXBoxController = new CommandXboxController(XBOX_CONTROLLER_PORT);
	
	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		SmartDashboard.putData("Field", field2d);
		configureBindings();
	}
	
	private void configureBindings() {
		// Set up the default command for the drivetrain.
		// The controls are for field-oriented driving:
		// Left stick Y axis -> forward and backwards movement
		// Left stick X axis -> left and right movement
		// Right stick X axis -> rotation
		// Drivetrain Commands
		// Drive command

		Trigger startButton = driverXBoxController.start();

		startButton.onTrue(Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();}));

		drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.joystickDriveCommand(
		() -> MathUtil.applyDeadband(driverXBoxController.getLeftY(), 0.1) * MAX_LINEAR_VELOCITY,
		() -> MathUtil.applyDeadband(driverXBoxController.getLeftX(), 0.1) * MAX_LINEAR_VELOCITY,
		() -> -MathUtil.applyDeadband(driverXBoxController.getRightX(), 0.1) * MAX_ANGULAR_VELOCITY)); // Negative because counter clockwise (left/-x on controller) should be positive
	}
	
	public Command getAutonomousCommand() {
		return null;
	}
}