package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCmd extends Command{

    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;
    
    // Elevator PID Constants
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
      
        this.pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);
    }

    public void initialize() {
        pidController.reset();
        //System.out.println("Elevator PID Command Initialized");
    }

    public void execute() {
        double speed = pidController.calculate(elevatorSubsystem.getLEncoderMeters());
        elevatorSubsystem.setMotor(speed);
    }

    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
        //System.out.println("Elevator PID Command Ended");
    }

    public boolean isFinished() {
        return false;
    }
}
