// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.GenericHID;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2;
  MotorControllerGroup wrist;
  Joystick stick;

  public Intake() {
    wristMotor1 = new CANSparkMax(9, MotorType.kBrushless);
    wristMotor1.setInverted(true);
    wristMotor2 = new CANSparkMax(10, MotorType.kBrushless);
    stick = new Joystick(0);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);
  }

  public void resetEncoder(int val){
    wristMotor1.getEncoder().setPosition(val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (stick.getPOV() == 0) {
      wrist.set(-.1);
    } else if (stick.getPOV() == 180) {
      wrist.set(.1);
    } else {
      wrist.set(0);
    }

    if(stick.getRawButton(12)){
      resetEncoder(0);
    }

    SmartDashboard.putNumber("wrist", wristMotor1.getEncoder().getPosition());
  }

  /*
   * -15 is straight
   */
}
