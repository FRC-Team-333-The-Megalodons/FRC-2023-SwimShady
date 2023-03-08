// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.w3c.dom.css.RGBColor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  CANSparkMax rightmotor1, rightmotor2, rightmotor3;
  CANSparkMax leftmotor1, leftmotor2, leftmotor3;
  MotorControllerGroup rightleader;
  MotorControllerGroup leftleader;
  Joystick stick;
  DifferentialDrive drive;

  public Chassis() {
    rightmotor1  = new CANSparkMax(1, MotorType.kBrushless);
    rightmotor2  = new CANSparkMax(2, MotorType.kBrushless);
    rightmotor3  = new CANSparkMax(3, MotorType.kBrushless);

    rightleader = new MotorControllerGroup(rightmotor1, rightmotor2, rightmotor3);

    leftmotor1 = new CANSparkMax(4, MotorType.kBrushless);
    leftmotor2 = new CANSparkMax(5, MotorType.kBrushless);
    leftmotor3 = new CANSparkMax(6, MotorType.kBrushless);

    leftleader = new MotorControllerGroup(leftmotor1, leftmotor2, leftmotor3);

    rightleader.setInverted(true);

    stick = new Joystick(0);
    drive = new DifferentialDrive(leftleader, rightleader);
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    drive.arcadeDrive(-stick.getX(), -stick.getY());
  }
}