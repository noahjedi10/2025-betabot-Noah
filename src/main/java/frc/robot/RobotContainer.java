// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.AlgaeGrabberStates.AlgaeGrabberGoToPositionCommand;
import frc.robot.commands.AlgaeGrabberStates.ElevatorPopUpAndAlgaeGrabberGoToPositionCommand;
import frc.robot.commands.AlgaeGrabberStates.AutonomousAlgaeGrabberCommands.AlgaeGrabberAndElevatorPositionAndIntakeManualEndCommand;
import frc.robot.commands.AutoAlign.AutoAlgaeCommand;
import frc.robot.commands.AutoAlign.AutoScoreCommand;
import frc.robot.commands.ElevatorStates.ElevatorReturnToHomeAndZeroCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorStates.ElevatorRetractCommand;
import frc.robot.commands.ElevatorStates.ElevatorHPIntakeCommand;
import frc.robot.commands.ElevatorStates.ElevatorGoToPositionCommand;
import frc.robot.commands.SlowFieldDriveCommand;


public class RobotContainer {
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  AlgaeGrabberSubsystem algaeGrabberSubsystem = new AlgaeGrabberSubsystem();

  Command defaultDriveCommand = new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX);
  Command algaeGrabberDefaultCommand = new AlgaeGrabberGoToPositionCommand(algaeGrabberSubsystem, AlgaeGrabberSubsystemConstants.RETRACTED_ENCODER_POSITION);

  Command intakeCommand = new ElevatorHPIntakeCommand(elevatorSubsystem);

  Command homeElevatorAndDontBreakAlgaeGrabber = new SequentialCommandGroup(
    new ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(algaeGrabberSubsystem, elevatorSubsystem, AlgaeGrabberSubsystemConstants.RETRACTED_ENCODER_POSITION),
    new ElevatorReturnToHomeAndZeroCommand(elevatorSubsystem)
  );

  boolean scoringOnLeft = true;
  boolean ejectAlgae = false;

  public RobotContainer() {
    configureDefaultBindings();
    configureBindings();
  }

  private void configureBindings() {
    configureDriveBindings();
    configureElevatorBindings();
    configureAlgaeGrabberBindings();
    // configureSideSelectorBindings();
    // configureAlgaeEjectOrRetainBindings();
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
    // algaeGrabberSubsystem.setDefaultCommand(algaeGrabberDefaultCommand);
    elevatorSubsystem.setDefaultCommand(new ElevatorRetractCommand(elevatorSubsystem));
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
    BooleanSupplier runElevatorExtruder = () -> driver.getRightTriggerAxis() > .25;

    // l2Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L2_ENCODER_POSITION, scoringOnLeftBooleanSupplier));
    // l3Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L3_ENCODER_POSITION, scoringOnLeftBooleanSupplier));
    // l4Score.onTrue(new AutoScoreCommand(driveSubsystem, elevatorSubsystem, ElevatorSubsystemConstants.L4_ENCODER_POSITION, scoringOnLeftBooleanSupplier));

    ParallelCommandGroup l2CommandManual = new ParallelCommandGroup(
      new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L2_ENCODER_POSITION),
      new SlowFieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX)
    );

    ParallelCommandGroup l3CommandManual = new ParallelCommandGroup(
      new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L3_ENCODER_POSITION),
      new SlowFieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX)
    );

    ParallelCommandGroup l4CommandManual = new ParallelCommandGroup(
      new ElevatorGoToPositionCommand(elevatorSubsystem, runElevatorExtruder, ElevatorSubsystemConstants.L4_ENCODER_POSITION),
      new SlowFieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX)
    );

    l2Score.onTrue(l2CommandManual);
    l3Score.onTrue(l3CommandManual);
    l4Score.onTrue(l4CommandManual);

    scoreCancel.onTrue(homeElevatorAndDontBreakAlgaeGrabber);

    JoystickButton hpIntakeButton = new JoystickButton(driver, 6);
    hpIntakeButton.toggleOnTrue(intakeCommand);
  }

  private void configureAlgaeGrabberBindings() {
    POVButton highAlgae = new POVButton(operator, 0);
    POVButton lowAlgae = new POVButton(operator, 90);
    POVButton cancelAlgaeGrab = new POVButton(operator, 180);

    highAlgae.onTrue(new AlgaeGrabberAndElevatorPositionAndIntakeManualEndCommand(elevatorSubsystem, algaeGrabberSubsystem, ElevatorSubsystemConstants.HIGH_ALGAE_POSITION, AlgaeGrabberSubsystemConstants.ALGAE_REMOVAL_ENCODER_POSITION));
    lowAlgae.onTrue(new AlgaeGrabberAndElevatorPositionAndIntakeManualEndCommand(elevatorSubsystem, algaeGrabberSubsystem, ElevatorSubsystemConstants.LOW_ALGAE_POSITION, AlgaeGrabberSubsystemConstants.ALGAE_REMOVAL_ENCODER_POSITION));
    cancelAlgaeGrab.onTrue(homeElevatorAndDontBreakAlgaeGrabber);

    JoystickButton intakeAlgae = new JoystickButton(driver, 1);
    intakeAlgae.onTrue(new AutoAlgaeCommand(driveSubsystem, elevatorSubsystem, algaeGrabberSubsystem, this::getEjectAlgae));
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

  private void configureAlgaeEjectOrRetainBindings() {
    JoystickButton ejectAlgaeButton = new JoystickButton(operator, 2);
    JoystickButton retainAlgaeButton = new JoystickButton(operator, 3);

    ejectAlgaeButton.onTrue(new InstantCommand(() -> {
      ejectAlgae = true;
    }));

    retainAlgaeButton.onTrue(new InstantCommand(() -> {
      ejectAlgae = false;
    }));
  }

  public boolean getScoringOnLeft() {
    return scoringOnLeft;
  }

  public boolean getEjectAlgae() {
    return ejectAlgae;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
