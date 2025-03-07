package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

    public enum ElevatorPosition {
        DOWN(Constants.downPos),
        POSITION_1(Constants.L1ElevatorPosition),
        POSITION_2(Constants.L2ElevatorPosition),
        POSITION_3(Constants.L3ElevatorPosition),
        POSITION_4(Constants.L4ElevatorPosition);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public Elevator() {
        primaryMotor = new SparkMax(Constants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(Constants.rightElevatorID, MotorType.kBrushless);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);

        // Configure follower
        followerMotor.configure(followerConfig, null, null); 
        
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(Constants.elevatorLimitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        constraints = new TrapezoidProfile.Constraints(
            Constants.elevatorMaxVelocity,
            Constants.elevatormaxAcceleration
        );
        
        pidController = new PIDController(
            Constants.elevatormaxAcceleration,
            Constants.elevatorkI,
            Constants.elevatorkD
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
        
        configureMotors();
    }

    private void configureMotors() {
        // Primary motor configuration
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        
        // Follower motor configuration
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
    public void periodic() {

        currentPos = encoder.getPosition() / Constants.countsPerInch;
        
        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (bottomLimit.get()) {
            handleBottomLimit();
        }

        if (getHeightInches() > Constants.maxPos) {
            stopMotors();
        }

        // Only run control if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -Constants.max_output,
                Constants.max_output
            );
            
            primaryMotor.set(outputPower);
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    private void handleBottomLimit() {
        stopMotors();
        encoder.setPosition(Constants.bottomPos * Constants.countsPerInch);
        isHomed = true;
        setpoint = Constants.bottomPos;
        currentState = new TrapezoidProfile.State(Constants.bottomPos, 0);
        goalState = new TrapezoidProfile.State(Constants.bottomPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < Constants.posTolerance;
    }
    

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return Constants.kS * Math.signum(state.velocity) +
               Constants.kG +
               Constants.kV * state.velocity;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            Constants.minPos,
            Constants.maxPos
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
    }

    public double getHeightInches() {
        return encoder.getPosition() / Constants.countsPerInch;
    }

    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (bottomLimit.get()) {
            handleBottomLimit();
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= Constants.maxPos && power > 0) {
            power = 0;
        }
        
        if (bottomLimit.get() && power < 0) {
            power = 0;
        }
        
        primaryMotor.set(MathUtil.clamp(power, -Constants.max_output, Constants.max_output));
    }
}
