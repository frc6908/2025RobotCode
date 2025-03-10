package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class CoralIntakeCmd extends Command {
  
    private final CoralSubsystem coralSubsystem;
    private final boolean open;

    public CoralIntakeCmd(CoralSubsystem coralSubsystem, boolean open) {
        this.open = open;
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Coral Intake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // coralIntakeSys.setPosition(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // coralIntakeSys.stop();
    System.out.println("Coral Intake Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
