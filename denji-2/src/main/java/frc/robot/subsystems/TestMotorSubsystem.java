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

import static frc.robot.Constants.*;

public class TestMotorSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
  
    private double p = 1.0;
    private double i = 0;
    private double d = 0;
    private double maxIAccum = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;
  
    public TestMotorSubsystem() {
      motor = new CANSparkMax(TEST_MOTOR_ID, MotorType.kBrushless); // CHANGE DEVICE ID
          motor.restoreFactoryDefaults();
          motor.setInverted(false);
          motor.setSmartCurrentLimit(30);
          
          encoder = motor.getEncoder();
          encoder.setPosition(0);
  
          pidController = motor.getPIDController();
          pidController.setP(p);
          pidController.setI(i);
          pidController.setD(d);
          pidController.setIMaxAccum(maxIAccum, 0);
          pidController.setReference(0, ControlType.kDutyCycle);
      }
  
    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + motor.getOutputCurrent() * (1-filterConstant);
    }
  
    public void teleopInit() {
        pidController.setReference(0, ControlType.kDutyCycle);
    }

    public void setMotorPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }
  
    public void setMotorDutyCycle(double dutyCycle) {
        pidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }
  
    public void setMotorVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity);
    }
  
    public void holdMotorPosition() {
        pidController.setReference(encoder.getPosition(), ControlType.kPosition);
    }
  
    public double getCurrent() {
        return motor.getOutputCurrent();
    }
  
    public double getSmoothCurrent() {
        return smoothCurrent;
    }
}
