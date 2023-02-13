// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax  leftMotor, rightMotor;
  Joystick stick;

  public PIDController elevatorController;

  private final double ELEVATOR_SPEED = 0.333;

  public Elevator() {
    leftMotor = new CANSparkMax(7, MotorType.kBrushless);
    rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    stick = new Joystick(0);
  }
  public void elevate(){
    leftMotor.set(ELEVATOR_SPEED);
    rightMotor.set(ELEVATOR_SPEED);
  }
  public void descelate(){
    leftMotor.set(-ELEVATOR_SPEED);
    rightMotor.set(-ELEVATOR_SPEED);
  }
  public void stop(){
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void resetEncoders() {
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  public double getPosMeters() {
    return -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(stick.getRawButton(4)){
      elevate();
    }else if(stick.getRawButton(3)){
      descelate();
    }else {
      stop();
    }
  }
}
