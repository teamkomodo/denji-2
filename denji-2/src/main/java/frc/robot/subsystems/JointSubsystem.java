// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

public class JointSubsystem extends SubsystemBase {
  
  //var
  private final CANSparkMax motor;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;
  private final DigitalInput reverseSwitch;
  
  private final ShuffleboardTab shuffleboardTab;

  private double p = 0;
  private double i = 0;
  private double d = 0;
  private double maxIAccum = 0;
  
  //limit
  private boolean atLimitSwitch = false;
  private boolean atMaxLimit = false;
  private boolean atMinLimit = false;

  private double commandedPosition = 0;

  private boolean useLimits = true;
  private boolean slowMode = false;

  private boolean zeroed = false;

  private double smoothCurrent = 0;

  public JointSubsystem() {
    motor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(30);

        reverseSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
        
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIMaxAccum(maxIAccum, 0);
        pidController.setReference(0, ControlType.kDutyCycle);

        shuffleboardTab = Shuffleboard.getTab("Joint");

        shuffleboardTab.addDouble("Motor Velocity", () -> encoder.getVelocity());
        shuffleboardTab.addDouble("Motor Current", () -> motor.getOutputCurrent());
        shuffleboardTab.addDouble("Smooth Current", () -> smoothCurrent);
    }

  public void teleopInit() {
      pidController.setReference(0, ControlType.kDutyCycle);
  } 

  public void checkLimitSwitch() {
    if(reverseSwitch.get()) {
        if(atLimitSwitch) {
            // reset encoder on falling edge incase the robot started up and the switch was pressed
            encoder.setPosition(0);
        }
        atLimitSwitch = false;
        return;
    }

    zeroed = true;
    if(!atLimitSwitch) {
        //stop motor and reset encoder on rising edge
        atLimitSwitch = true;
        encoder.setPosition(0);
        setPosition(0);
    }
    
  }

  public void checkMinLimit() {
    if(encoder.getPosition() > JOINT_MIN_POSITION) {
        atMinLimit = false;
        return;
    }

    if(!atMinLimit) {
        atMinLimit = true;
        setPosition(JOINT_MIN_POSITION);
    }
  }

  public void checkMaxLimit() {
    if(encoder.getPosition() < JOINT_MAX_POSITION) {
        atMaxLimit = false;
        return;
    }
            
    if(!atMaxLimit) {
        //stop motor on rising edge
        atMaxLimit = true;
        setPosition(JOINT_MAX_POSITION);
    }
  }

  public void setMotorPercent(double percent) {
    //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
    if(atMinLimit && percent < 0 && zeroed && useLimits)
        return;
    
    //at max or not yet zeroed and attempting to increase
    if((atMaxLimit || !zeroed) && percent > 0 && useLimits)
        return;
    pidController.setReference(percent * (slowMode? JOINT_SLOW_MODE_MULTIPLIER : 1) * 0.1, ControlType.kDutyCycle);
  }

  public void setPosition(double position) {
    //position out of bounds
    if(position < JOINT_MIN_POSITION || position > JOINT_MAX_POSITION)
        return;
    
    //not zeroed and moving away from limit switch
    if(!zeroed & position > encoder.getPosition())
        return;

    pidController.setReference(position, ControlType.kPosition);
    commandedPosition = position;
  }

  public void gotoSetPosition(int positionId) {
    setPosition(JOINT_POSITIONS_ORDERED[positionId]);
  }

//set motor positions
  public Command holdPositionCommand() {
    return this.runOnce(() -> setPosition(encoder.getPosition()));
  }

  public Command grabPositionCommand(BooleanSupplier cubeMode) {
    return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? JOINT_CUBE_GROUND_POSITION: JOINT_CUBE_GROUND_POSITION));
  }

  public Command releasePositionCommand(BooleanSupplier cubeMode) {
    return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? JOINT_CUBE_RELEASE_POSITION: JOINT_CUBE_RELEASE_POSITION));
  }

  public Command launchPositionCommand(BooleanSupplier cubeMode) {
    return this.runOnce(() -> setPosition(cubeMode.getAsBoolean()? JOINT_CUBE_LAUNCH_POSITION: JOINT_CUBE_LAUNCH_POSITION));
  }

  public Command zeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(-0.3), this),
        Commands.waitUntil(() -> (atLimitSwitch))
    );
}

public Command disableLimitsCommand() {
    return this.runOnce(() -> useLimits = false);
}

public Command enableLimitsCommand() {
    return this.runOnce(() -> useLimits = true);
}

@Override
public void periodic() {
    checkMinLimit();
    checkMaxLimit();
    checkLimitSwitch();
}

public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;

    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
}

public double getPosition() {
    return encoder.getPosition();
}

public boolean isZeroed() {
    return zeroed;
}

  public ShuffleboardTab getTab() {
      return shuffleboardTab;
  }
}
