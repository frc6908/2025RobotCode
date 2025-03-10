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
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
//import swervelib.SwerveInputStream;

import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ElevatorPID.CoralSetpoints;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  private final CommandXboxController m_driverController =
    new CommandXboxController(0);

  public RobotContainer()
  {
    //default commands
    coralSubsystem.setDefaultCommand(new CoralIntakeCmd(coralSubsystem, true));
    elevatorSubsystem.setDefaultCommand(new ElevatorCmd(elevatorSubsystem, 0));

    // Configure the button bindings
    configureButtonBindings();

  }

  private void configureButtonBindings()
  {
    // Elevator goes up with Y button, down with A button
    m_driverController.y().whileTrue(new ElevatorCmd(elevatorSubsystem, 0.5));
    m_driverController.a().whileTrue(new ElevatorCmd(elevatorSubsystem, -0.5));

    // Elevator goes to L4 meters with Y button, 0 meters with A button
    m_driverController.b().whileTrue(new ElevatorPID(elevatorSubsystem, CoralSetpoints.L4));
    m_driverController.x().whileTrue(new ElevatorPID(elevatorSubsystem, 0));

    // Coral outtakes with right bumper button
    m_driverController.rightBumper().whileTrue(new CoralIntakeCmd(coralSubsystem, false));
  }

  /*public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Commands.parallel(
      new ElevatorPID(elevatorSubsystem, 1.2),
      new CoralIntakeCmd(coralSubsystem, false)
    );
  }*/
}