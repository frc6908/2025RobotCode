package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

public class AlgaeMechanism extends SubsystemBase {
    private final SparkMax ioSpark;
    private final SparkMax algaeArmSpark;
    // private final RelativeEncoder algaeArmEncoder;

    public AlgaeMechanism(
        int ioSparkPort,
        int algaeArmSparkPort
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        algaeArmSpark = new SparkMax(algaeArmSparkPort, MotorType.kBrushless);
        // algaeArmEncoder = algaeArmSpark.getAlternateEncoder();

        configureMotor(ioSpark, IdleMode.kBrake);
        configureMotor(algaeArmSpark, IdleMode.kBrake);
    }

    public void configureMotor(
        SparkMax spark,
        IdleMode idleMode
        ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIOSpark(double speed) {
        ioSpark.set(speed);
    }

    public void stopIOSpark() {
        ioSpark.stopMotor();
    }

    public void setAlgaeArmSpark(double speed) {
        algaeArmSpark.set(speed);
    }

    public void stopAlgaeArmSpark() {
        algaeArmSpark.stopMotor();
    }
}
