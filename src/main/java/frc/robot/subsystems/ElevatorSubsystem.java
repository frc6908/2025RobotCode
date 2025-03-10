// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  // Motors and Encoder
  private final SparkMax elevLeftMotor = new SparkMax(13, MotorType.kBrushless);
  private final SparkMax elevRightMotor = new SparkMax(14, MotorType.kBrushless);

  private final RelativeEncoder elevRightEncoder = elevRightMotor.getEncoder();

  public ElevatorSubsystem() {

  }
  
  // Measuring elevator height
  private final double kEncoderTick2Meter = 1.0 / 4096.0 * 2.0 * Math.PI * 0.0254; //chat this right?
  
  public double getLEncoderMeters() {
    return elevRightEncoder.getPosition() * kEncoderTick2Meter; 
  }
  
  public double getREncoderMeters() {
    return elevRightEncoder.getPosition()* kEncoderTick2Meter;
  }

    //or just this
  public double getEncoderElevatorPosition() {
    return (elevRightEncoder.getPosition());
  }

  // Set motor speed
  public void setMotor (double speed) {
    elevLeftMotor.set(speed);
    elevRightMotor.set(-speed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator L Height", getLEncoderMeters());
    SmartDashboard.putNumber("Elevator R Height", getREncoderMeters());

    SmartDashboard.putNumber("Elevator Right Encoder Position", getEncoderElevatorPosition());
  }
}