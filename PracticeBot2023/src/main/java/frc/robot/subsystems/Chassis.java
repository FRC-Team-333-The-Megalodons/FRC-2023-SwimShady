// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  CANSparkMax rightmotor1, rightmotor2, rightmotor3;
  CANSparkMax leftmotor1, leftmotor2, leftmotor3;
  MotorControllerGroup rightleader;
  MotorControllerGroup leftleader;
  Joystick stick;
  DifferentialDrive drive;

  public Chassis() {
    rightmotor1 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_LEADER, MotorType.kBrushless);
    rightmotor2 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER1, MotorType.kBrushless);
    rightmotor3 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER2, MotorType.kBrushless);

    rightleader = new MotorControllerGroup(rightmotor1, rightmotor2, rightmotor3);

    leftmotor1 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_LEADER, MotorType.kBrushless);
    leftmotor2 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER1, MotorType.kBrushless);
    leftmotor3 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER2, MotorType.kBrushless);

    leftleader = new MotorControllerGroup(leftmotor1, leftmotor2, leftmotor3);

    stick = new Joystick(0);
    drive = new DifferentialDrive(leftleader, rightleader);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drive.arcadeDrive(-stick.getX(), -stick.getY());
  }
}
