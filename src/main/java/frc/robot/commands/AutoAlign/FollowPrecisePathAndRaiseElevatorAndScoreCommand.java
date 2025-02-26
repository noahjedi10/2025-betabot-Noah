// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPrecisePathAndRaiseElevatorAndScoreCommand extends FollowPrecisePathCommand {
  ElevatorSubsystem elevatorSubsystem;
  double encoderPosition;
  double grabberSpeed = ElevatorSubsystemConstants.GRABBER_SPEED;
  boolean pidReachedSetpoint = false;
  
  public FollowPrecisePathAndRaiseElevatorAndScoreCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, double encoderPosition, Pose2d goalPose) {
    super(driveSubsystem, goalPose);
    this.elevatorSubsystem = elevatorSubsystem;
    this.encoderPosition = encoderPosition;
    addRequirements(elevatorSubsystem);
  }

  public FollowPrecisePathAndRaiseElevatorAndScoreCommand(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, double encoderPosition, Pose2d goalPose, double grabberSpeed) {
    super(driveSubsystem, goalPose);
    this.elevatorSubsystem = elevatorSubsystem;
    this.encoderPosition = encoderPosition;
    this.grabberSpeed = grabberSpeed;
    addRequirements(elevatorSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Scoring in parallel");
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
    
    elevatorSubsystem.setPosition(encoderPosition);

    if(isDriveLoopAtSetpoint() && elevatorSubsystem.isElevatorPIDAtSetpoint()) {
      System.out.println("Setting grabber");
      elevatorSubsystem.setGrabber(grabberSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
    elevatorSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isDriveLoopAtSetpoint() {
    return (this.X_PRECISE_PATH_PID.atSetpoint() && this.Y_PRECISE_PATH_PID.atSetpoint() && this.ROTATE_PRECISE_PATH_PID.atSetpoint());
  }
}
