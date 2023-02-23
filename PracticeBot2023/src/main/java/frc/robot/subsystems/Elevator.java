// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rightMotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  double espeed = 0.4;

  frc.robot.utils.PIDController ePidController;
  RobotStates.ElevatorState elevatorState; //todo let the robot know when it's at low medium or high and add it as a state

  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;

  public Elevator() {
    rightMotor = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    ePidController = new frc.robot.utils.PIDController(.008, .0006, 0, 0, 7, 7,50);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rightMotor, leftmotor);

    lowerLimitSwitch = new DigitalInput(0);
    upperLimitSwitch = new DigitalInput(1);
  }

  public void ePID_Up() {
    leftmotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
    rightMotor.set(ePidController.getOutput(leftmotor.getEncoder().getPosition()));
    elevatorState = RobotStates.ElevatorState.TRAVERSING_UP;
  }

  public void eUp() {
    elevator.set(ePidController.getOutput(rightMotor.getEncoder().getPosition()));
    elevatorState = RobotStates.ElevatorState.TRAVERSING_UP;
  }

  public void eDown() {
    elevator.set(espeed);
    elevatorState = RobotStates.ElevatorState.TRAVERSING_DOWN;
    if(lowerLimitSwitch.get() == true){
      elevator.set(0);
      rightMotor.getEncoder().setPosition(0);
    }
  }

  public void stop() {
    elevator.set(0);
    elevatorState = RobotStates.ElevatorState.MOTORS_STOPPED;
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  public double averageEncoderDistance() {
    return (leftmotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2;
  }

  public void teleopPeriodic() {

    if (stick.getRawButton(4)) {
      eUp();
    } else if (stick.getRawButton(3)) {
      eDown();
    } else {
      stop();
    }

    if (!lowerLimitSwitch.get()) {
      //stop();
      resetEncoders();
    } 

    if (!upperLimitSwitch.get()) {
      //stop();
    }

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Aveage Encoder Distance", averageEncoderDistance());
    SmartDashboard.putString("Elevator State", elevatorState+"");
    SmartDashboard.putBoolean("Lower Switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Upper Switch", upperLimitSwitch.get());
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
