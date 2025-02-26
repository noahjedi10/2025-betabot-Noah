// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase {


  SparkMax primary = new SparkMax(ElevatorSubsystemConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  SparkMax secondary = new SparkMax(ElevatorSubsystemConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  SparkMax spinGrabber = new SparkMax(ElevatorSubsystemConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);
  
  ColorSensorV3 coralSensor = new ColorSensorV3(Port.kOnboard);

  RelativeEncoder rightEncoder = primary.getEncoder();

  SparkClosedLoopController onboardClosedLoop = primary.getClosedLoopController();

  double currentSetpoint = 0.0;

  // ElevatorFeedforward eff = new ElevatorFeedforward(getElevatorCurrentDraw(), getPosition(), getElevatorCurrentDraw())

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    SparkMaxConfig smc = new SparkMaxConfig();
    smc.follow(primary, false);
    smc.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    secondary.configure(smc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig.smartCurrentLimit(ElevatorSubsystemConstants.NEO550_CURRENT_LIMIT);
    neoConfig.inverted(true);
    neoConfig.idleMode(IdleMode.kBrake);
    neoConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    spinGrabber.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig primaryConfig = new SparkMaxConfig();
    primaryConfig.encoder.positionConversionFactor(1.0);
    primaryConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

    primaryConfig.closedLoop.pid(0.0, 0.0, 0.0);
    primaryConfig.closedLoop.maxMotion
      .maxAcceleration(ElevatorSubsystemConstants.MAX_ACCELERATION)
      .maxVelocity(ElevatorSubsystemConstants.MAX_VELOCITY)
      .allowedClosedLoopError(0.5);
    primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightEncoder.setPosition(0.0);
  }

  public void setSpin(double percent)
  {
    primary.set(percent);
  }

  public void setGrabber(double percent)
  {
    spinGrabber.set(percent);
  }

  public double getPosition()
  {
    return rightEncoder.getPosition();
  }

  public void setPosition(double position) {
    currentSetpoint = position;
    onboardClosedLoop.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ElevatorSubsystemConstants.ARBITRARY_FEEDFORWARD, ArbFFUnits.kPercentOut);
  }

  public void zeroEncoder() {
    rightEncoder.setPosition(0.0);
  }

  public void stopAll() {
    setSpin(0.0);
    setGrabber(0.0);    
  }

  public boolean getIsCoralInHoldingPosition() {
    return coralSensor.getProximity() > ElevatorSubsystemConstants.CORAL_SENSOR_PROXIMITY_THRESHOLD;
  }

  public boolean isElevatorPIDAtSetpoint() {
    return Math.abs(getPosition() - currentSetpoint) < ElevatorSubsystemConstants.AT_SETPOINT_TOLERANCE;
  }

  public double getElevatorCurrentDraw() {
    return primary.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorPos",getPosition());
    SmartDashboard.putNumber("CurrentDrawElevator", getElevatorCurrentDraw());
    SmartDashboard.putNumber("sensorProximity", coralSensor.getProximity());
  }
}
