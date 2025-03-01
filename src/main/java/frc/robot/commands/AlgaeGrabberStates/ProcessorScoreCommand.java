// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeGrabberStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProcessorScoreCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  AlgaeGrabberSubsystem algaeGrabberSubsystem;

  double elevatorPosition;
  double algaeGrabberPosition;

  BooleanSupplier runOuttake;

  public ProcessorScoreCommand(ElevatorSubsystem elevatorSubsystem, AlgaeGrabberSubsystem algaeGrabberSubsystem, double elevatorPosition, double algaeGrabberPosition, BooleanSupplier runOuttake) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;

    this.elevatorPosition = elevatorPosition;
    this.algaeGrabberPosition = algaeGrabberPosition;

    this.runOuttake = runOuttake;

    addRequirements(elevatorSubsystem, algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Raising elevator with auto end");
    elevatorSubsystem.setPosition(elevatorPosition);

    if(elevatorSubsystem.getPosition() > AlgaeGrabberSubsystemConstants.MINIMUM_SAFE_ELEVATOR_ENCODER_POSITION) {
      algaeGrabberSubsystem.setPosition(algaeGrabberPosition);
    } else {
      algaeGrabberSubsystem.setPivotMotor(0.0);
    }

    algaeGrabberSubsystem.setSpinMotor((runOuttake.getAsBoolean()) ? -AlgaeGrabberSubsystemConstants.INTAKE_MOTOR_SPEED: 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabberSubsystem.stopAll();
    elevatorSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return algaeGrabberSubsystem.getSpinMotorCurrentDraw() > AlgaeGrabberSubsystemConstants.INTAKE_CURRENT_DRAW;
  }
}
