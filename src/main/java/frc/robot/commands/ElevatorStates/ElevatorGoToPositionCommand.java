// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorStates;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToPositionCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  BooleanSupplier runCoralExtruder;
  double positionSetpoint;
  public ElevatorGoToPositionCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier runCoralExtruder, double positionSetpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.runCoralExtruder = runCoralExtruder;
    this.positionSetpoint = positionSetpoint;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Ran elevator go to position");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Sending elevator to position");
    boolean runExtruder = runCoralExtruder.getAsBoolean();
    elevatorSubsystem.setPosition(positionSetpoint);
    elevatorSubsystem.setGrabber((runExtruder == true) ? ElevatorSubsystemConstants.GRABBER_SPEED: 0.0);
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
