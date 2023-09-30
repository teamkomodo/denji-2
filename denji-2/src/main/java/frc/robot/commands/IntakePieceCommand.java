package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.*;

public class IntakePieceCommand extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;

    private boolean holdingPiece;
    private long startTime;
    //clampStart means 
    private long clampStart;

    public IntakePieceCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        holdingPiece = false;
        startTime = RobotController.getFPGATime();
        clampStart = -1;
    }

    @Override
    public void end(boolean interrupted) {
        if(!holdingPiece)
            intakeSubsystem.setMotorDutyCycle(0);
    }

    @Override
    public void execute() {
        if(holdingPiece) {
            return;
        }

        intakeSubsystem.setMotorVelocity(-4000);

          if(clampStart == -1){
            clampStart = RobotController.getFPGATime();
          }

        if(Math.abs(intakeSubsystem.getSmoothCurrent()) > INTAKE_THRESHOLD_CURRENT && RobotController.getFPGATime() - startTime > 500000 && RobotController.getFPGATime() - clampStart > 500000) {
            intakeSubsystem.setMotorDutyCycle(-0.2);
            holdingPiece = true;
        }
    }
    
}