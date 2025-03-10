// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.OperatorConstants;


import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
//import swervelib.SwerveInputStream;

import frc.robot.commands.ElevJoystickCmd;
import frc.robot.commands.ElevatorPIDCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final CommandXboxController m_driverController =
    new CommandXboxController(0);

  public RobotContainer()
  {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings()
  {
    // Configure your button bindings here

    // Elevator goes up with Y button, down with A button
    m_driverController.y().whileTrue(new ElevJoystickCmd(elevatorSubsystem, 0.5));
    m_driverController.a().whileTrue(new ElevJoystickCmd(elevatorSubsystem, -0.5));

    // Elevator goes to 1.2 meters with Y button, 0 meters with A button
    m_driverController.b().whileTrue(new ElevatorPIDCmd(elevatorSubsystem, 1.2));
    m_driverController.x().whileTrue(new ElevatorPIDCmd(elevatorSubsystem, 0));
  }
}