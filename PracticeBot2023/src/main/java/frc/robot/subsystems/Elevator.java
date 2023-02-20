// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rihgtmotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  double espeed = .4;
  frc.robot.utils.PIDController ePidController;
  RobotStates.ElevatorState elevatorState;//todo let the robot know when it's at low medium or high and add it as a state

  public Elevator() {
    rihgtmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    ePidController = new frc.robot.utils.PIDController(.008, .0006, 0, 0, 10, 10,69.5);

    rihgtmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rihgtmotor, leftmotor);
  }

  public void ePID_Up() {
    leftmotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
    rihgtmotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
    elevatorState = RobotStates.ElevatorState.TRAVERSING_UP;
  }

  public void eUp() {
    elevator.set(espeed);
    elevatorState = RobotStates.ElevatorState.TRAVERSING_UP;
  }

  public void eDown() {
    elevator.set(-espeed);
    elevatorState = RobotStates.ElevatorState.TRAVERSING_DOWN;
  }

  public void stop() {
    elevator.set(0);
    elevatorState = RobotStates.ElevatorState.MOTORS_STOPPED;
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(0);
    rihgtmotor.getEncoder().setPosition(0);
  }

  public void teleopPeriodic()
  {
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

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", rihgtmotor.getEncoder().getPosition());
    SmartDashboard.putString("Elevator State", elevatorState+"");
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
