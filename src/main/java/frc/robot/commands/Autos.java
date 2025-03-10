// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(SwerveSubsystem subsystem) {
    // return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    
    return Commands.sequence(
      new SwerveJoystickCmd(subsystem, () -> .1, () -> 0, () -> 0).withTimeout(5), 
      new SwerveJoystickCmd(subsystem, () -> .2, () -> 0, () -> 0).withTimeout(5),
      new SwerveJoystickCmd(subsystem, () -> .3, () -> 0, () -> 0).withTimeout(5),
      new SwerveJoystickCmd(subsystem, () -> .4, () -> 0, () -> 0).withTimeout(5),
      new SwerveJoystickCmd(subsystem, () -> .5, () -> 0, () -> 0).withTimeout(5),
      new SwerveJoystickCmd(subsystem, () -> .6, () -> 0, () -> 0).withTimeout(5),
      new SwerveJoystickCmd(subsystem, () -> .7, () -> 0, () -> 0).withTimeout(5));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
