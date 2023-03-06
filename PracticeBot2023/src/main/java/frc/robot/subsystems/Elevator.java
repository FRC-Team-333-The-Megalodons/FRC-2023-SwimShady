// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ElevatorState;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rightMotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  XboxController controller;
  final double espeed = 1;

  frc.robot.utils.PIDController ePidController;
  RobotStates.ElevatorState elevatorState; //todo let the robot know when it's at low medium or high and add it as a state

  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;

  Intake intake;

  public Elevator(Intake intake) {
    rightMotor = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    controller = new XboxController(1);
    ePidController = new frc.robot.utils.PIDController(.05, .005, 0, 80, 4, 5,95);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rightMotor, leftmotor);
    elevator.setInverted(true);

    lowerLimitSwitch = new DigitalInput(1);
    upperLimitSwitch = new DigitalInput(0);

    this.intake = intake;
  }

  public void stop() {
    elevator.set(0);
    //elevatorState = RobotStates.ElevatorState.MOTORS_STOPPED;
  }

  public void manualUp(){
    if(upperLimitSwitch.get() == false){
      elevator.set(0);
      //elevatorState = ElevatorState.HIGH;
      return;
    }else{
      elevator.set(espeed);
    }
  }

  public void manualDown(){
    if(lowerLimitSwitch.get() == false){
      elevator.set(0);
      rightMotor.getEncoder().setPosition(0);
      //elevatorState = RobotStates.ElevatorState.LOW;
      return;
    }else {
      elevator.set(-(espeed-.2));
    }
  }

  public void e_Mid(){
    if(upperLimitSwitch.get() == false ){
      elevator.set(0);//acts as an emegency stop in case the pid fails
      return;
    }else{
      elevator.set(ePidController.getOutput(-rightMotor.getEncoder().getPosition()));
    }
  }

  public boolean isControllerOnTarget(){
    return ePidController.isOnTarget();
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  public ElevatorState getState(){
    return elevatorState;
  }

  public boolean isAtMaxUp(){return !upperLimitSwitch.get();}
  public boolean isAtMaxDown(){return !lowerLimitSwitch.get();}

  public void teleopPeriodic() {
    if(!RobotContainer.TWO_DRIVER_MODE){
      if (stick.getRawButton(4)) {
        manualUp();
      } else if (stick.getRawButton(3)) {
        if(stick.getRawAxis(3) < .6){
          e_Mid();
        }else{
          manualDown();
        }
      }else{
        if(stick.getRawAxis(3) < .6){
          manualDown();
        }else{
          elevator.set(0);
        }
      }
    }else{
      if(controller.getYButton()){
        manualUp();
      }else if(controller.getAButton() && controller.getLeftTriggerAxis() < .5){
        if(stick.getRawAxis(3) < .6){
          e_Mid();
        }else{
          manualDown();
        }
      }else{
        if(stick.getRawAxis(3) < .6 && controller.getLeftTriggerAxis() < .5){
          manualDown();
        }else{
          elevator.set(0);
        }
      }
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", -rightMotor.getEncoder().getPosition());
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
