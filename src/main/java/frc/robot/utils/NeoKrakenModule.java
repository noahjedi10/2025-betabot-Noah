// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;;;
public class NeoKrakenModule {
    double NOMINAL_VOLTAGE = 12.8; // Can't be real

    TalonFX driveMotor;
    SparkMax steerMotor;
    CANcoder encoderObject;

    double offset, setPoint;

    SimpleMotorFeedforward ff;
    PIDController driveController, steerController;

    double VELOCITY_MULTIPLIER = (1 / 6.75 / 60) * (.1016 * Math.PI);
    double POSITION_MULTIPLIER = (1 / 6.75) * (.1016 * Math.PI);
    double CANCODER_POSITION_MULTIPLIER = 2 * Math.PI;


    private void configPIDinternal() {
        ff = new SimpleMotorFeedforward(0.015, .212);
        driveController = new PIDController(.01, 0, .00);
        steerController = new PIDController(.3, 0.0, 0.0); // P might be negative according to an old chief delphi
                                                           // thread

        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /* private void configDriveMotor(CANSparkMax target) {
        target.restoreFactoryDefaults();

        driveEncoder = target.getEncoder();
        driveEncoder.setVelocityConversionFactor((1 / 6.75 / 60) * (.1016 * Math.PI));
        driveEncoder.setPositionConversionFactor((1 / 6.75) * (.1016 * Math.PI));
        target.setInverted(false);
        target.setIdleMode(IdleMode.kBrake);
        target.enableVoltageCompensation(NOMINAL_VOLTAGE);
        target.setSmartCurrentLimit(35);

        target.burnFlash();
    }
    */

    private void configSteerMotor(SparkMax target) {
        SparkMaxConfig c = new SparkMaxConfig();
        c.inverted(false);
        c.voltageCompensation(NOMINAL_VOLTAGE);
        c.idleMode(IdleMode.kBrake);
        c.smartCurrentLimit(80);
        target.configure(c, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPercentOutput(double speed) {
        steerMotor.set(speed);
    }

    private void setupEncoder(CANcoder encoder) {
        CANcoderConfigurator configPls = encoder.getConfigurator();
        CANcoderConfiguration canCon = new CANcoderConfiguration();
        canCon.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //[-.5, .5)

        configPls.apply(canCon);
    }

    /*  public void configureRampRate(double ramp) {
        driveMotor.setOpenLoopRampRate(ramp);
    } 
    */

    public NeoKrakenModule(int drive, int steer, int encoderId, double offsetValue) {
        driveMotor = new TalonFX(drive);
        steerMotor = new SparkMax(steer, MotorType.kBrushless);
        encoderObject = new CANcoder(encoderId);
        offset = offsetValue;
        setupEncoder(encoderObject);
        configDriveMotor(driveMotor);
        configSteerMotor(steerMotor);
        configPIDinternal();
    }

    public NeoKrakenModule(int drive, int steer, int encoderId, double offsetValue, String can) {
        driveMotor = new TalonFX(drive, can);
        steerMotor = new SparkMax(steer, MotorType.kBrushless);
        encoderObject = new CANcoder(encoderId, can);
        offset = offsetValue;

        setupEncoder(encoderObject);
        configDriveMotor(driveMotor);
        configSteerMotor(steerMotor);
        configPIDinternal();
    }

    // Sets the swerve module to the provided state
    public void setModuleState(SwerveModuleState state) {
        double currentMeasurement = getEncoderPosition();
        state.optimize(Rotation2d.fromRadians(currentMeasurement));

        double speed = state.speedMetersPerSecond;
        double theta = state.angle.getRadians();
        setPoint = theta;

        SmartDashboard.putString("M -> S", String.valueOf(getEncoderPosition()) + " " + String.valueOf(theta));

        double drivePercent = driveController.calculate(getVelocity(), speed);
        double steerPercent = steerController.calculate(getEncoderPosition(), theta);

        driveMotor.set(drivePercent + ff.calculate(speed)); // COMMENT THESE OUT TO
        // DO OFFSETS!!!!!!
        steerMotor.set(-steerPercent);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(),
                Rotation2d.fromRadians(getEncoderPosition()));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getPosition(),
                Rotation2d.fromRadians(Math.PI + getEncoderPosition()));
    }

    private void configDriveMotor(TalonFX talonFX) {
        TalonFXConfigurator tfxc = talonFX.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        // config.Feedback.SensorToMechanismRatio = (1 / 6.75 / 60) * (.1016 * Math.PI);
        currentLimitsDrive(config);
        tfxc.apply(config);
    }

    private void currentLimitsDrive(TalonFXConfiguration config) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.StatorCurrentLimit = 120;
    }
    
     public double getEncoderPosition() {
        StatusSignal<Angle> angle = encoderObject.getAbsolutePosition();
        return (angle.getValueAsDouble() * CANCODER_POSITION_MULTIPLIER) - offset;
    }

    public double getVelocity() {
        StatusSignal<AngularVelocity> velocity = driveMotor.getVelocity();
        return velocity.getValueAsDouble() * VELOCITY_MULTIPLIER;
    }

    public double getPosition() {
        StatusSignal<Angle> position = driveMotor.getPosition();
        return position.getValueAsDouble() * POSITION_MULTIPLIER;
    }
}
