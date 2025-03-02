// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Failsafes;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

//Gives operator direct manual control over elevator height, just in case something bad happens
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OperatorFailsafeCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier operatorJoystickInputSupplier;
  AlgaeGrabberSubsystem algaeGrabberSubsystem;

  public OperatorFailsafeCommand(ElevatorSubsystem elevatorSubsystem, AlgaeGrabberSubsystem algaeGrabberSubsystem, DoubleSupplier operatorJoystickInputSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.operatorJoystickInputSupplier = operatorJoystickInputSupplier;

    addRequirements(elevatorSubsystem, algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.stopAll();
    algaeGrabberSubsystem.stopAll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setSpin(operatorJoystickInputSupplier.getAsDouble() * .15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
