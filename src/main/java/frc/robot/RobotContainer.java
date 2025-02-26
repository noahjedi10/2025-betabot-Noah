// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.AutoAlign.AutoAlgaeCommand;
import frc.robot.commands.AutoAlign.AutoScoreCommand;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  XboxController driver = new XboxController(0);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  AlgaeGrabberSubsystem algaeGrabberSubsystem = new AlgaeGrabberSubsystem();

  Command defaultDriveCommand = new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX);

  boolean scoringOnLeft = true;

  public RobotContainer() {
    configureDefaultBindings();
    configureBindings();
  }

  private void configureBindings() {
    configureDriveBindings();
    configureElevatorBindings();
    configureAlgaeGrabberBindings();
    configureSideSelectorBindings();
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
  }

  private void configureDriveBindings() {
    JoystickButton zeroDriverGyro = new JoystickButton(driver, 4);
    zeroDriverGyro.onTrue(new InstantCommand(driveSubsystem::driverGyroZero));
  }

  private void configureElevatorBindings() {
    POVButton l2Score = new POVButton(driver, 90);
    POVButton l3Score = new POVButton(driver, 0);
    POVButton l4Score = new POVButton(driver, 270);
    POVButton scoreCancel = new POVButton(driver, 180);

    BooleanSupplier scoringOnLeftBooleanSupplier = this::getScoringOnLeft;

    l2Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION, scoringOnLeftBooleanSupplier));
    l3Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION, scoringOnLeftBooleanSupplier));
    l4Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L4_ENCODER_POSITION, scoringOnLeftBooleanSupplier));

    scoreCancel.onTrue(new ElevatorRetractCommand(elevatorSubsystem));
  }

  private void configureAlgaeGrabberBindings() {
    JoystickButton intakeAlgae = new JoystickButton(driver, 1);
    intakeAlgae.onTrue(new AutoAlgaeCommand(driveSubsystem, elevatorSubsystem, algaeGrabberSubsystem));
  }

  private void configureSideSelectorBindings() {
    JoystickButton leftSelector = new JoystickButton(driver, 3);
    JoystickButton rightSelector = new JoystickButton(driver, 2);

    leftSelector.onTrue(new InstantCommand(() -> {
      System.out.println("Left selector pressed");
      scoringOnLeft = true;
    }));
    rightSelector.onTrue(new InstantCommand(() -> {scoringOnLeft = false;}));
  }

  public boolean getScoringOnLeft() {
    return scoringOnLeft;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
