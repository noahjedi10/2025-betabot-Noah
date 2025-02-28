// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SlowFieldDriveCommand extends Command {
  DoubleSupplier x, y, rotX;
  DriveSubsystem driveSubsystem;
  public SlowFieldDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotX) {
    this.x = x;
    this.y = y;
    this.rotX = rotX;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Slow drive running");
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(y.getAsDouble() * 0.5, x.getAsDouble() * 0.5, rotX.getAsDouble() * Math.toRadians(180), driveSubsystem.getDriverGyroAngle());
    driveSubsystem.drive(fieldRelativeSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
