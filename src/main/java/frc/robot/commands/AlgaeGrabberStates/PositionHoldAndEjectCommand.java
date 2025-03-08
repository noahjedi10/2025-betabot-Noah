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
public class PositionHoldAndEjectCommand extends Command {
  AlgaeGrabberSubsystem algaeGrabberSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  BooleanSupplier runExtrudeBooleanSupplier;
  
  double currentElevatorPosition = 5.0;
  double currentAlgaeGrabberPosition = 0.3;

  public PositionHoldAndEjectCommand(AlgaeGrabberSubsystem algaeGrabberSubsystem, ElevatorSubsystem elevatorSubsystem, BooleanSupplier runExtrudeBooleanSupplier) {
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.runExtrudeBooleanSupplier = runExtrudeBooleanSupplier;

    addRequirements(algaeGrabberSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentElevatorPosition = elevatorSubsystem.getPosition();
    this.currentAlgaeGrabberPosition = algaeGrabberSubsystem.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Holding position with optional extrude");
    algaeGrabberSubsystem.setPosition(currentAlgaeGrabberPosition);
    elevatorSubsystem.setPosition(currentElevatorPosition);
    algaeGrabberSubsystem.setSpinMotor((runExtrudeBooleanSupplier.getAsBoolean()) ? -AlgaeGrabberSubsystemConstants.INTAKE_MOTOR_SPEED: 0.0);
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
  }
}
