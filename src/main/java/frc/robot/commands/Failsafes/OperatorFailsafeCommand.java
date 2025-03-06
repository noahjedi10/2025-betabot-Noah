// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Failsafes;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

//Gives operator direct manual control over elevator height, algae grabber orientation, and algae grabber flywheels, just in case something bad happens
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OperatorFailsafeCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  AlgaeGrabberSubsystem algaeGrabberSubsystem;

  DoubleSupplier elevatorSupplier;
  DoubleSupplier algaeGrabberPivotSupplier;
  BooleanSupplier runExtruder;

  public OperatorFailsafeCommand(ElevatorSubsystem elevatorSubsystem, AlgaeGrabberSubsystem algaeGrabberSubsystem, DoubleSupplier elevatorSupplier, DoubleSupplier algaeGrabberPivotSupplier, BooleanSupplier runExtruder) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;

    this.elevatorSupplier = elevatorSupplier;
    this.algaeGrabberPivotSupplier = algaeGrabberPivotSupplier;
    this.runExtruder = runExtruder;

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
  public void execute() { //Will this help? Will drivers even think to use it should an algae intake go awry? Many questions. Questions like: Where is the climb?? Where is the aegis chain mount? Why do the pulleys squeak? Can we reliably do L4??
    elevatorSubsystem.setSpin(elevatorSupplier.getAsDouble() * 0.15);
    algaeGrabberSubsystem.setPivotMotor(algaeGrabberPivotSupplier.getAsDouble() * 0.5);
    algaeGrabberSubsystem.setSpinMotor((runExtruder.getAsBoolean()) ? -AlgaeGrabberSubsystemConstants.INTAKE_MOTOR_SPEED: 0.0);
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
