// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeGrabberStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowAlgaeCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  AlgaeGrabberSubsystem algaeGrabberSubsystem;

  double elevatorHoldPosition = AlgaeGrabberSubsystemConstants.MINIMUM_SAFE_ELEVATOR_ENCODER_POSITION;

  public StowAlgaeCommand(AlgaeGrabberSubsystem algaeGrabberSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    addRequirements(elevatorSubsystem, algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = elevatorSubsystem.getPosition();
    double minimumPosition = AlgaeGrabberSubsystemConstants.MINIMUM_SAFE_ELEVATOR_ENCODER_POSITION;
    elevatorHoldPosition = (currentPosition < minimumPosition) ? minimumPosition: currentPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(elevatorHoldPosition);
    if(elevatorSubsystem.isElevatorPIDAtSetpoint()) {
      algaeGrabberSubsystem.setPosition(AlgaeGrabberSubsystemConstants.RETRACTED_ENCODER_POSITION);
    } else {
      algaeGrabberSubsystem.setPivotMotor(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopAll();
    algaeGrabberSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
