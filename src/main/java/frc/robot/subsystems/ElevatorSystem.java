/* 
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final CANPIDController elevatorPIDController;

    // Elevator PID Constants (Tune these values!)
    private static final double kP = 0.1; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain
    private static final double kFF = 0.0; // Feedforward

    private static final int SMART_CURRENT_LIMIT = 60; // Max current limit (amps)
    private static final double MAX_HEIGHT = 50.0; // Maximum height in inches
    private static final double MIN_HEIGHT = 0.0; // Minimum height in inches

    public ElevatorSystem() {
        // Initialize the motor (set CAN ID to 9, assuming a NEO brushless motor)
        elevatorMotor = new SparkMax(9, MotorType.kBrushless);

        // Reset to factory defaults for safety
        //elevatorMotor.setIdleMode(IdleMode.kBrake); // Brake mode for stability
        //elevatorMotor.setSmartCurrentLimit(SMART_CURRENT_LIMIT); // Prevents overheating

        // Manually reset to default values (if needed)
        //elevatorMotor.setOpenLoopRampRate(0.0); // Set ramp rate to 0 for immediate response

        // Setup encoder (built-in NEO encoder)
        elevatorEncoder = elevatorMotor.getEncoder();

        // Setup PID Controller
        elevatorPIDController = elevatorMotor.getPIDController();
        elevatorPIDController.setP(kP);
        elevatorPIDController.setI(kI);
        elevatorPIDController.setD(kD);
        elevatorPIDController.setFF(kFF);
        elevatorPIDController.setOutputRange(-1, 1);

        // Reset encoder position
        resetEncoder();
    }

    /** Move the elevator manually using a joystick 
    public void moveElevator(double speed) {
        // Prevent elevator from exceeding limits
        double currentPosition = getHeight();
        if ((speed > 0 && currentPosition >= MAX_HEIGHT) || (speed < 0 && currentPosition <= MIN_HEIGHT)) {
            stopElevator();
        } else {
            elevatorMotor.set(speed);
        }
    }

    /** Set elevator to a specific height using PID 
    public void setElevatorHeight(double targetHeight) {
        if (targetHeight > MAX_HEIGHT) {
            targetHeight = MAX_HEIGHT;
        } else if (targetHeight < MIN_HEIGHT) {
            targetHeight = MIN_HEIGHT;
        }
        elevatorPIDController.setReference(targetHeight, ControlType.kPosition);
    }

    /** Get the current height of the elevator 
    public double getHeight() {
        return elevatorEncoder.getPosition();
    }

    /** Stops the elevator motor 
    public void stopElevator() {
        elevatorMotor.set(0);
    }

    /** Resets the encoder to zero 
    public void resetEncoder() {
        elevatorEncoder.setPosition(0);
    }

    /** Runs periodically to update SmartDashboard 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getHeight());
    }
}
*/
