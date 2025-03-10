// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  
  private final SparkMax coralIntakeMotor1 = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax coralIntakeMotor2 = new SparkMax(16, MotorType.kBrushless);

  public CoralSubsystem() {

  }

  public void setPosition (boolean open) {
    if (open) {
      coralIntakeMotor1.set(0.5);
      coralIntakeMotor2.set(-0.5);
    } else {
      coralIntakeMotor1.set(-0.5);
      coralIntakeMotor2.set(0.5);
    }
  }

  //public void stop() {
  //  coralIntakeMotor1.set(0);
    //coralIntakeMotor2.set(0);
  //}
}
