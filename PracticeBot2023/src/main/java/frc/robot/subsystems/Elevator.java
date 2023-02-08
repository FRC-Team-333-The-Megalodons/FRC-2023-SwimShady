// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax  rihgtmotor, leftmotor;
  Joystick stick;
  double espeed = 0.333;

  public Elevator() {
    rihgtmotor = new CANSparkMax(7, MotorType.kBrushless);
    leftmotor = new CANSparkMax(8, MotorType.kBrushless);
    stick = new Joystick(0);
  }
  public void eUp(){
    rihgtmotor.set(espeed);
    leftmotor.set(-espeed);
  }
  public void eDown(){
    rihgtmotor.set(-espeed);
    leftmotor.set(espeed);
  }
  public void stop(){
    rihgtmotor.set(0);
    leftmotor.set(0);
  }







































  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(stick.getRawButton(4)){
      eUp();
    }else if(stick.getRawButton(3)){
      eDown();
    }else {
      stop();
    }
  }
}
