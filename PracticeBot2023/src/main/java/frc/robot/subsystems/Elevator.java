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
import frc.robot.RobotStates.ElevatorState;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rightMotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  double espeed = 1;

  frc.robot.utils.PIDController ePidController;
  RobotStates.ElevatorState elevatorState; //todo let the robot know when it's at low medium or high and add it as a state

  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;

  public Elevator() {
    rightMotor = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    ePidController = new frc.robot.utils.PIDController(.015, .005, 0, 10, 2, 2,50);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rightMotor, leftmotor);
    elevator.setInverted(true);

    lowerLimitSwitch = new DigitalInput(0);
    upperLimitSwitch = new DigitalInput(1);
  }

  private void setState(double output){
    if(output > 0){
        if(-rightMotor.getEncoder().getPosition() > 52){
          elevatorState = ElevatorState.TRAVERSING_HIGH_FROM_MID;
        }else if(-rightMotor.getEncoder().getPosition() < 48){
          elevatorState = ElevatorState.TRAVERSING_UP_FROM_LOW;
        }
    }else{
      if(-rightMotor.getEncoder().getPosition() > 52){
        elevatorState = ElevatorState.TRAVERSING_DOWN_FROM_HIGH;
      }else if(-rightMotor.getEncoder().getPosition() < 48){
        elevatorState = ElevatorState.TRAVERSING_DOWN_FROM_MID;
      }
    }

    if(-rightMotor.getEncoder().getPosition() <= 52 || -rightMotor.getEncoder().getPosition() >= 48){
      elevatorState = ElevatorState.MEDIUM;
    }
    if(rightMotor.getEncoder().getPosition() == 0){
      elevatorState = ElevatorState.LOW;
    }
  }

  public void eUp() {
    if(lowerLimitSwitch.get() == false){
      elevator.set(0);
      elevatorState = ElevatorState.HIGH;
      return;
    }else{
      elevator.set(espeed);
      setState(espeed);
    }
  }

  public void e_Mid(){
    if(lowerLimitSwitch.get() == false){
      elevator.set(0);//acts as an emegency stop in case the pid fails
      return;
    }else{
      elevator.set(ePidController.getOutput(-rightMotor.getEncoder().getPosition()));
      setState(ePidController.getOutput(-rightMotor.getEncoder().getPosition()));
    }
  }

  public void eDown() {
    if(upperLimitSwitch.get() == false){
      elevator.set(0);
      rightMotor.getEncoder().setPosition(0);
      elevatorState = RobotStates.ElevatorState.LOW;
      return;
    }else if(stick.getRawAxis(3) < 1){
      elevator.set(-espeed);
      setState(-espeed);
    }else{
      stop();
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

  public ElevatorState getState(){
    return elevatorState;
  }

  public void teleopPeriodic() {

    if (stick.getRawButton(4)) {
      eUp();
    } else if (stick.getRawButton(3)) {
      e_Mid();
    } else {
      eDown();
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", -rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Average Encoder Distance", averageEncoderDistance());
    SmartDashboard.putString("Elevator State", elevatorState+"");
    SmartDashboard.putBoolean("Lower Switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Upper Switch", upperLimitSwitch.get());
    SmartDashboard.putNumber("Stick lever", stick.getRawAxis(3));
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
