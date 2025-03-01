// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveSubsystemConstants;
import frc.robot.utils.CavbotsPoseEstimator;
import frc.robot.utils.NeoKrakenModule;

public class DriveSubsystem extends SubsystemBase {
  NeoKrakenModule fleft, fright, bleft, bright;
  Pigeon2 pigeon = new Pigeon2(DriveSubsystemConstants.PIGEON_ID, DriveSubsystemConstants.CANIVORE_NAME);
  CavbotsPoseEstimator poseEstimator;
  Rotation2d driverGyroOffset = new Rotation2d();

  Field2d field = new Field2d();

  public DriveSubsystem() {
    fleft = new NeoKrakenModule(DriveSubsystemConstants.FLEFT_DRIVE_ID, DriveSubsystemConstants.FLEFT_STEER_ID, DriveSubsystemConstants.FLEFT_CANCODER, DriveSubsystemConstants.FLEFT_OFFSET, DriveSubsystemConstants.CANIVORE_NAME);
    fright = new NeoKrakenModule(DriveSubsystemConstants.FRIGHT_DRIVE_ID, DriveSubsystemConstants.FRIGHT_STEER_ID, DriveSubsystemConstants.FRIGHT_CANCODER, DriveSubsystemConstants.FRIGHT_OFFSET, DriveSubsystemConstants.CANIVORE_NAME);
    bleft = new NeoKrakenModule(DriveSubsystemConstants.BLEFT_DRIVE_ID, DriveSubsystemConstants.BLEFT_STEER_ID, DriveSubsystemConstants.BLEFT_CANCODER, DriveSubsystemConstants.BLEFT_OFFSET, DriveSubsystemConstants.CANIVORE_NAME);
    bright = new NeoKrakenModule(DriveSubsystemConstants.BRIGHT_DRIVE_ID, DriveSubsystemConstants.BRIGHT_STEER_ID, DriveSubsystemConstants.BRIGHT_CANCODER, DriveSubsystemConstants.BRIGHT_OFFSET, DriveSubsystemConstants.CANIVORE_NAME);

    poseEstimator = new CavbotsPoseEstimator(this, new Pose2d(9.506, 4.067, Rotation2d.fromDegrees(0)));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    fleft.setModuleState(states[0]);
    fright.setModuleState(states[1]);
    bleft.setModuleState(states[2]);
    bright.setModuleState(states[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveSubsystemConstants.M_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void autoDrive(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond *= -1;
    speeds.vxMetersPerSecond *= -1;
    speeds.vyMetersPerSecond *= -1;
    SwerveModuleState[] states = DriveSubsystemConstants.M_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void setYaw(Rotation2d rot) {
    pigeon.setYaw(rot.getDegrees());
  }

  public void driverGyroZero() {
    driverGyroOffset = getAngle();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      fleft.getSwerveModulePosition(),
      fright.getSwerveModulePosition(),
      bleft.getSwerveModulePosition(),
      bright.getSwerveModulePosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      fleft.getSwerveModuleState(),
      fright.getSwerveModuleState(),
      bleft.getSwerveModuleState(),
      bright.getSwerveModuleState()
    };
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return DriveSubsystemConstants.M_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
  }

  public Rotation2d getAngle() {
    return pigeon.getRotation2d();
  }

  public Rotation2d getDriverGyroAngle() {
    return getAngle().minus(driverGyroOffset);
  }

  public CavbotsPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FLEFT", fleft.getEncoderPosition());
    SmartDashboard.putNumber("FRIGHT", fright.getEncoderPosition());
    SmartDashboard.putNumber("BLEFT", bleft.getEncoderPosition());
    SmartDashboard.putNumber("BRIGHT", bright.getEncoderPosition());

    poseEstimator.updateWithVisionAndOdometry(getAngle(), getModulePositions());
    field.setRobotPose(poseEstimator.getPose2d());
    SmartDashboard.putData("field", field);
  }
}
