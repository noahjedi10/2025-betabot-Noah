// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  XboxController driver = new XboxController(0);

  DriveSubsystem driveSubsystem = new DriveSubsystem();

  Command defaultDriveCommand = new FieldDriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX);

  public RobotContainer() {
    configureDefaultBindings();
    configureBindings();
  }

  private void configureBindings() {
    configureDriveBindings();
  }

  private void configureDefaultBindings() {
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
  }

  private void configureDriveBindings() {
    JoystickButton zeroDriverGyro = new JoystickButton(driver, 4);
    zeroDriverGyro.onTrue(new InstantCommand(driveSubsystem::driverGyroZero));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
