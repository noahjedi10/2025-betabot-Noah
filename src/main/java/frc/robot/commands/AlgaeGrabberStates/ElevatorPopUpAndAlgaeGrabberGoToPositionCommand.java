// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeGrabberStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPopUpAndAlgaeGrabberGoToPositionCommand extends Command {
  AlgaeGrabberSubsystem algaeGrabberSubsystem;
  ElevatorSubsystem elevatorSubsystem;

  double homePosition = AlgaeGrabberSubsystemConstants.MINIMUM_SAFE_ELEVATOR_ENCODER_POSITION;
  double algaeGrabberEncoderPosition;

  public ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(AlgaeGrabberSubsystem algaeGrabberSubsystem, ElevatorSubsystem elevatorSubsystem, double algaeGrabberEncoderPosition) {
    System.out.println(algaeGrabberEncoderPosition);
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.algaeGrabberEncoderPosition = algaeGrabberEncoderPosition;
    addRequirements(elevatorSubsystem, algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = elevatorSubsystem.getPosition();
    double minPosition = AlgaeGrabberSubsystemConstants.MINIMUM_SAFE_ELEVATOR_ENCODER_POSITION;
    algaeGrabberSubsystem.setPosition(algaeGrabberEncoderPosition);
    homePosition = (currentPosition < minPosition) ? minPosition: currentPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Setting elevator for pop up");
    elevatorSubsystem.setPosition(homePosition);
    if(elevatorSubsystem.isElevatorPIDAtSetpoint()) {
      System.out.println(algaeGrabberEncoderPosition);
      algaeGrabberSubsystem.setPosition(algaeGrabberEncoderPosition);
    } else {
      algaeGrabberSubsystem.setPivotMotor(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(algaeGrabberEncoderPosition);
    // System.out.println("Ended "+ String.valueOf(interrupted));
    elevatorSubsystem.stopAll();
    algaeGrabberSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeGrabberSubsystem.isAlgaeGrabberPIDAtSetpoint();
    // return false;
  }
}
