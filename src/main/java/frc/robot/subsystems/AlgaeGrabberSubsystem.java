// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.Constants.RobotConstants;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  SparkMax pivotMotor = new SparkMax(AlgaeGrabberSubsystemConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  SparkMax spinMotor = new SparkMax(AlgaeGrabberSubsystemConstants.SPIN_MOTOR_ID, MotorType.kBrushless);

  DutyCycleEncoder thruBore = new DutyCycleEncoder(AlgaeGrabberSubsystemConstants.THRU_BORE_ENCODER_ID);

  PIDController controller = new PIDController(1.0, 0.0, 0);

  public AlgaeGrabberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    config.idleMode(IdleMode.kBrake);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig spinConfig = new SparkMaxConfig();
    spinConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    spinConfig.idleMode(IdleMode.kBrake);
    spinConfig.openLoopRampRate(.25);
    spinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller.setTolerance(.1);
  }

  public void setSpinMotor(double percent) {
    spinMotor.set(-percent);
  }

  public void setPivotMotor(double percent) {
    pivotMotor.set(percent);
  }

  private double linearizeEncoderOutput(double currentPosition) { //This is stupid.
    if(currentPosition > .5) {
      return currentPosition - 1;
    }
    return currentPosition;
  }

  public void setPosition(double position) {
    double currentPosition = linearizeEncoderOutput(getPosition());
    double speed = controller.calculate(currentPosition, position);
    setPivotMotor(speed);
  }


  public void stopAll() {
    spinMotor.set(0.0);
    pivotMotor.set(0.0);
  }

  public double getPosition() {
    return thruBore.get();
  }

  public double getLinearizedPosition() {
    return linearizeEncoderOutput(getPosition());
  }

  public double getSpinMotorCurrentDraw() {
    return spinMotor.getOutputCurrent();
  }

  public boolean isAlgaeGrabberPIDAtSetpoint() {
    // double at = linearizeEncoderOutput(getPosition());
    // return Math.abs(at - setpoint) < .1;
    return controller.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AlgaeGrabberEncoder", linearizeEncoderOutput(getPosition()));
    SmartDashboard.putNumber("AlgaeGrabberCurrentDraw", getSpinMotorCurrentDraw());
    // This method will be called once per scheduler run
  }
}
