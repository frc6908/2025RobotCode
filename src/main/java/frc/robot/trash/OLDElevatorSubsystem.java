/*package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorOldSubsystem extends SubsystemBase {
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
    private final SparkMax m_motor1 = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    // simulator private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
    private final ProfiledPIDController m_Controller = new ProfiledPIDController(ElevatorConstants.kElevatorkP, 
                                                                                ElevatorConstants.kElevatorkI, 
                                                                                ElevatorConstants.kElevatorkD, 
                                                                                new Constraints(ElevatorConstants.kMaxVelocity, 
                                                                                    ElevatorConstants.kMaxAcceleration));
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS, 
                                                                            ElevatorConstants.kElevatorkG, 
                                                                            ElevatorConstants.kElevatorkV, 
                                                                            ElevatorConstants.kElevatorkA);
    
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(40)
            .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
            .closedLoop
            .outputRange(-1, 1);
         m_motor1.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
         m_motor2.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
         //m_motor2.follow(m_motor1);
    }

    public double getPositionMeters() {
        return m_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (m_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(), m_Controller.getSetpoint().velocity)
                + m_Controller.calculate(getPositionMeters(), goal),
                -7,
                7);
        m_motor1.setVoltage(voltsOutput);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

    public static Distance convertRotationsToDistance(Angle rotations){
      return Meters.of(rotations.in(Rotations) *
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing);
    }

    public static Angle convertDistanceToRotations(Distance distance){
      return Rotations.of(distance.in(Meters) /
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) * ElevatorConstants.kElevatorGearing);
    }

    public void stop()
    {
        m_motor1.set(0.0);
    }

    public void updateTelemetry()
    {
    }

    public void periodic()
    {
    }
}*/
