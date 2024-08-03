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
import frc.robot.subsystems.TestMotorSubsystem;

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
  public final TestMotorSubsystem testMotorSubsystem = new TestMotorSubsystem();
  
  private final CommandXboxController driverXBoxController = new CommandXboxController(XBOX_CONTROLLER_PORT);
  private final GenericHID driverJoystick = new GenericHID(JOYSTICK_PORT);
  private final GenericHID driverButtons = new GenericHID(BUTTONS_PORT);
  private final GenericHID selector = new GenericHID(SELECTOR_PORT);

  private final HashMap<String, Command> eventMap = new HashMap<>();

    // private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    //     drivetrainSubsystem::getPose,
    //     drivetrainSubsystem::resetPose,
    //     drivetrainSubsystem.getKinematics(),
    //     new PIDConstants(5.0, 0.0, 0.0),
    //     new PIDConstants(1.5, 0.0, 0.0),
    //     drivetrainSubsystem::setModuleStates,
    //     eventMap, true, drivetrainSubsystem
    // );

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Field", field2d);
    // configureAuto();
    configureBindings();
  }

  // private void configureAuto() {

  //   eventMap.put("place", Commands.sequence(
  //       jointSubsystem.releasePositionCommand(),
  //       Commands.waitSeconds(1.0),
  //       Commands.runEnd(() -> intakeSubsystem.setMotorDutyCycle(1.0), () -> intakeSubsystem.setMotorDutyCycle(0), intakeSubsystem).withTimeout(0.2),
  //       Commands.run(() -> jointSubsystem.setPosition(JOINT_STOW_POSITION), jointSubsystem).withTimeout(0.5))
  //   );

  //   autoChooser.addOption("No Auto", null);
  //   autoChooser.addOption("Place Mobility Open", autoBuilder.fullAuto(PathPlanner.loadPath("Place Mobility Open", new PathConstraints(2.0, 1.5))));
  //   SmartDashboard.putData("Auto Selection", autoChooser);
  // }
  
  private void configureBindings() {
    
    
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}