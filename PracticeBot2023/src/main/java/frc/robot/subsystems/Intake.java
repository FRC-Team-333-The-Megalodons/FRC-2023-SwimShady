// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.GenericHID;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax motor;
  Joystick stick;

  public Intake() {
    motor = new CANSparkMax(9, MotorType.kBrushless);
    stick = new Joystick(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (stick.getPOV() == 0) {
      System.out.println("Extend");
    } else if (stick.getPOV() == 180) {
      System.out.println("Reverse");
    } else {
      System.out.println("Stop");
    }
  }
}
