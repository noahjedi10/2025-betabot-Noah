// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPrecisePathCommand extends Command {
  DriveSubsystem driveSubsystem;
  Pose2d goalPose;

  PIDController X_PRECISE_PATH_PID = new PIDController(3.0, 0.0, 0.0);
  PIDController Y_PRECISE_PATH_PID = new PIDController(3.0, 0.0, 0.0);
  PIDController ROTATE_PRECISE_PATH_PID = new PIDController(2.0, 0.0, 0.0);

  Field2d setpointField = new Field2d();

  public FollowPrecisePathCommand(DriveSubsystem driveSubsystem, Pose2d goalPose) {
    this.driveSubsystem = driveSubsystem;
    this.goalPose = goalPose;

    X_PRECISE_PATH_PID.setTolerance(.01);
    Y_PRECISE_PATH_PID.setTolerance(.01);
    ROTATE_PRECISE_PATH_PID.setTolerance(.2);

    ROTATE_PRECISE_PATH_PID.enableContinuousInput(0, Math.PI * 2);

    SmartDashboard.putData("spfield", setpointField);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving");
    Pose2d currentPose = driveSubsystem.getPoseEstimator().getPose2d();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double currentRotRads = currentPose.getRotation().getRadians();

    double goalX = goalPose.getX();
    double goalY = goalPose.getY();
    double goalRotRads = goalPose.getRotation().getRadians();

    double xChassisSpeeds = X_PRECISE_PATH_PID.calculate(currentX, goalX);
    double yChassisSpeeds = Y_PRECISE_PATH_PID.calculate(currentY, goalY);
    double omegaRads = ROTATE_PRECISE_PATH_PID.calculate(currentRotRads, goalRotRads);

    driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-xChassisSpeeds, -yChassisSpeeds, -omegaRads, currentPose.getRotation()));
    setpointField.setRobotPose(goalPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (X_PRECISE_PATH_PID.atSetpoint() && Y_PRECISE_PATH_PID.atSetpoint() && ROTATE_PRECISE_PATH_PID.atSetpoint());
    return true;
  }
}
