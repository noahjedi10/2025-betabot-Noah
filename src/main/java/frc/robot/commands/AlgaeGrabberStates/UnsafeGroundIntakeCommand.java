// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeGrabberStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

//If the algae grabber isn't in the correct position prior to this command being scheduled, it will break it.

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnsafeGroundIntakeCommand extends Command {
  AlgaeGrabberSubsystem algaeGrabberSubsystem;
  ElevatorSubsystem elevatorSubsystem;

  public UnsafeGroundIntakeCommand(AlgaeGrabberSubsystem algaeGrabberSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    addRequirements(elevatorSubsystem, algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(ElevatorSubsystemConstants.GROUND_INTAKE_POSITION);
    algaeGrabberSubsystem.setPosition(AlgaeGrabberSubsystemConstants.GROUND_INTAKE_ENCODER_POSITION);
    
    algaeGrabberSubsystem.setSpinMotor(AlgaeGrabberSubsystemConstants.INTAKE_MOTOR_SPEED);
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
    // return false;
    return algaeGrabberSubsystem.getSpinMotorCurrentDraw() > AlgaeGrabberSubsystemConstants.INTAKE_CURRENT_DRAW;
  }
}
