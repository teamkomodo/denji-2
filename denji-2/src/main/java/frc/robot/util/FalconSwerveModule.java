package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.Constants.*;

public class FalconSwerveModule implements SwerveModule{

    private final double drivePositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> wheel travel in meters
    private final double driveVelocityConversionFactor = drivePositionConversionFactor * 10; // sensor ticks/100ms -> wheel speed in m/s

    private final double steerPositionConversionFactor = 2 * Math.PI * STEER_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> module rotation in radians
    // private final double steerVelocityConversionFactor = steerPositionConversionFactor * 10; // sensor ticks/100ms -> module rad/s

    private final double driveP = 1.0;
    private final double driveI = 0;
    private final double driveD = 0;

    private final double steerP = 2.0e-1;
    private final double steerI = 1.0e-6;
    private final double steerD = 1.0e-7;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder steerAbsoluteEncoder;

    private final PIDController driveController = new PIDController(driveP, driveI, driveD);
    
    // TODO Measure This
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.0); // V/M/s

    private SwerveModuleState desiredState;
    private double adjustedDesiredAngle = 0;

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, ShuffleboardContainer container) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        this.steerAbsoluteEncoder = new CANCoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        if(steerAbsoluteEncoder.configGetMagnetOffset() != steerOffset)
            steerAbsoluteEncoder.configMagnetOffset(Math.toDegrees(steerOffset));

        configureMotors();
        buildShuffleboard(container);
    }

    private void buildShuffleboard(ShuffleboardContainer container) {
        container.addNumber("Desired Velocity", () -> desiredState.speedMetersPerSecond);
        container.addNumber("Desired Rotation", () -> desiredState.angle.getDegrees());
        container.addNumber("Rotation Error", () -> Math.toDegrees(getSteerPosition() - adjustedDesiredAngle));
        container.addNumber("Velocity Error", () -> getDriveVelocity() - desiredState.speedMetersPerSecond);

        container.addNumber("Raw Absolute Rotation", () -> steerAbsoluteEncoder.getAbsolutePosition());
        // container.addNumber("Adjusted Absolute Rotation", () -> getAbsoluteModuleRotation().getDegrees());
        container.addNumber("Raw Relative Rotation", () -> Math.toDegrees(getSteerPosition()));
        // container.addNumber("Adjusted Relative Rotation", () -> getModuleRotation().getDegrees());
        container.addNumber("Velocity", () -> getDriveVelocity());
    }

    private void configureMotors() {

        //driveMotor.setSensorPhase(true); // invert the sensor
        driveMotor.setInverted(true); // invert the motor
        driveMotor.setNeutralMode(NeutralMode.Brake); // motor brakes when idle

        steerMotor.setSensorPhase(false);
        steerMotor.setInverted(false);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        steerMotor.config_kP(0, steerP);
        steerMotor.config_kI(0, steerI);
        steerMotor.config_kD(0, steerD);
        steerMotor.setSelectedSensorPosition(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) / steerPositionConversionFactor);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());

        final double driveOutput = driveController.calculate(getDriveVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        
        driveMotor.set(TalonFXControlMode.PercentOutput, (driveOutput + driveFeedforward) / FALCON_500_NOMINAL_VOLTAGE); // Output Voltage / Max Voltage = Percent Output

        double currentAngleMod = MathUtil.angleModulus(getSteerPosition());
        double adjustedAngle = optimizedState.angle.getRadians() + getSteerPosition() - currentAngleMod;

        if(optimizedState.angle.getRadians() - currentAngleMod > Math.PI) {
            adjustedAngle -= 2.0 * Math.PI;
        }else if(optimizedState.angle.getRadians() - currentAngleMod < -Math.PI) {
            adjustedAngle += 2.0 * Math.PI;
        }
        adjustedDesiredAngle = adjustedAngle;
        steerMotor.set(TalonFXControlMode.Position, adjustedAngle / steerPositionConversionFactor);
        //steerMotor.set(TalonFXControlMode.Position, 0);
        this.desiredState = optimizedState;

        //correctRelativeEncoder();
    }

    //@SuppressWarnings(value = { "unused" })
    private void correctRelativeEncoder() {
        double delta = getAbsoluteModuleRotation().getRadians()-getModuleRotation().getRadians();
        if(delta > Math.PI)
            delta -= 2 * Math.PI;

        if(delta < -Math.PI)
            delta += 2 * Math.PI;

        relativeSteerAdjustment += delta * relativeSteerAdjustmentFactor;

    }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(getSteerPosition() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + steerOffset + relativeSteerAdjustment)); // Handled by 
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition())));
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }

    private double getSteerPosition() {
        return steerMotor.getSelectedSensorPosition() * steerPositionConversionFactor;
    }
    
    private double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * drivePositionConversionFactor;
    }

    private double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * driveVelocityConversionFactor;
    }
}
