// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rihgtmotor, leftmotor;
  Joystick stick;
  double espeed = .4;
  frc.robot.utils.PIDController ePidController;

  public Elevator() {
    rihgtmotor = new CANSparkMax(7, MotorType.kBrushless);
    leftmotor = new CANSparkMax(8, MotorType.kBrushless);
    stick = new Joystick(0);
    ePidController = new frc.robot.utils.PIDController(.008, .0006, 0, 0, 10, 10);

    rihgtmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
  }

  public void ePID_Up() {
    leftmotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
    rihgtmotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
  }

  public void eUp() {
    rihgtmotor.set(espeed);
    leftmotor.set(-espeed);
  }

  public void eDown() {
    rihgtmotor.set(-espeed);
    leftmotor.set(espeed);
  }

  public void stop() {
    rihgtmotor.set(0);
    leftmotor.set(0);
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(0);
    rihgtmotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (stick.getRawButton(4)) {
      eUp();
    } else if (stick.getRawButton(3)) {
      eDown();
    } else {
      stop();
    }

    if (stick.getRawButton(2)) {
      resetEncoders();
    }

    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", rihgtmotor.getEncoder().getPosition());
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
