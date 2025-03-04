// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.AutoAlignCommandFactory;
import frc.robot.utils.PathLoader;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreL4Command extends Command {
  DriveSubsystem driveSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  BooleanSupplier onLeftSide;
  double scoringPosition;
  double grabberSpeed = ElevatorSubsystemConstants.GRABBER_SPEED;

  public AutoScoreL4Command(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, double scoringPosition, BooleanSupplier onLeftSide) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.scoringPosition = scoringPosition;
    this.onLeftSide = onLeftSide;
    addRequirements(driveSubsystem, elevatorSubsystem);
  }

  public AutoScoreL4Command(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, double scoringPosition, BooleanSupplier onLeftSide, double grabberSpeed) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.scoringPosition = scoringPosition;
    this.onLeftSide = onLeftSide;
    this.grabberSpeed = grabberSpeed;
    addRequirements(driveSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(scoringPosition);
    System.out.println("Running auto score command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {    
    System.out.println("Ran!");
    Command alignmentCommand = AutoAlignCommandFactory.getAutoAlignAndScoreCommand(
      driveSubsystem.getPoseEstimator().getPose2d(), 
      elevatorSubsystem,
      driveSubsystem,
      scoringPosition, 
      PathLoader.getShouldFlipPath(),
      onLeftSide.getAsBoolean(),
      grabberSpeed
    );
    
    alignmentCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
