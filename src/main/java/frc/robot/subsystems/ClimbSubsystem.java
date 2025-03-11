// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.RobotConstants;

public class ClimbSubsystem extends SubsystemBase {
  SparkMax primary = new SparkMax(ClimbSubsystemConstants.PRIMARY_ID, MotorType.kBrushless);
  SparkMax follower = new SparkMax(ClimbSubsystemConstants.FOLLOWER_ID, MotorType.kBrushless);
  RelativeEncoder enc = primary.getEncoder();

  PIDController pid = new PIDController(.1, 0, 0);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    SparkMaxConfig primaryConfig = new SparkMaxConfig();
    primaryConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    primaryConfig.idleMode(IdleMode.kBrake);
    primaryConfig.smartCurrentLimit(120);
    primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig secondaryConfig = new SparkMaxConfig();
    secondaryConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    secondaryConfig.follow(primary, true);
    secondaryConfig.idleMode(IdleMode.kBrake);
    secondaryConfig.smartCurrentLimit(120);
    follower.configure(secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    enc.setPosition(0.0); //zero climb angle motor on startup.
  }

  public void setSpin(double spin) {
    primary.set(spin);
  }

  public void setPosition(double position) {
    double speed = pid.calculate(getPosition(), position);
    setSpin(speed);
  }

  public void stopAll() {
    setSpin(0.0);
  }
  public double getPosition() {
    return enc.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encposclimb", getPosition());
    // This method will be called once per scheduler run
  }
}
