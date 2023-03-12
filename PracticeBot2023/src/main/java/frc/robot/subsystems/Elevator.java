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
import frc.robot.utils.PIDController;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rightMotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  XboxController controller;
  final double MAX_ESPEED = 1;

  final double ELEVATOR_BOTTOM = 0;
  final double ELEVATOR_TOP = 215;

  frc.robot.utils.PIDController eMidPidController, eGroundPidController;
  RobotStates.ElevatorState elevatorState; //todo let the robot know when it's at low medium or high and add it as a state

  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;

  Intake intake;

  public Elevator(Intake intake) {
    rightMotor = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    controller = new XboxController(1);
    eMidPidController = new frc.robot.utils.PIDController(.05, .005, 0, 80, 4, 5,95);
    eGroundPidController = new PIDController(.05, .005, 0, 80, 4, 5,10);

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

  public void manualUp()
  {
    manualMove(MAX_ESPEED);
  }

  public void manualDown(){
    manualMove(-MAX_ESPEED);
  }

  public void manualMove(double speed)
  {
    // Negative speed means down, Positive speed means up.
    // That means that if it's positive, we have to respect isAtMaxUp,
    //  and if negative, we have to respect isAtMaxDown.
    if (speed > 0 && isAtMaxUp()) {
      stop();
    } else if (speed < 0 && isAtMaxDown()) {
      stop();
    } else {
      elevator.set(speed);
    }
  }

  public void e_Mid(){
    manualMove(eMidPidController.getOutput(getRightPosition()));
  }

  public double getRightPosition()
  {
    return -rightMotor.getEncoder().getPosition();
  }

  public void e_GroundPosition(){
    manualMove(eGroundPidController.getOutput(getRightPosition()));
  }

  public boolean isControllerOnTarget(){
    return eMidPidController.isOnTarget();
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  public ElevatorState getState(){
    return elevatorState;
  }

  public boolean isAtMaxUp(){
    if (upperLimitSwitch.get() == false) {
      return true;
    }
    if (getRightPosition() >= ELEVATOR_TOP) {
      return true;
    }
    return false;
  }
  public boolean isAtMaxDown(){
    if (lowerLimitSwitch.get() == false) {
      return true;
    }
    // We intentionally don't check getPosition at ELEVATOR_DOWN 
    //  as we'd have no way to get a reset position if not.
    return false;
  }

  public void teleopPeriodic()
  {
      // "Dead zone" check for Right Joystick
      if(Math.abs(controller.getRightY()) > 0.05) {
        manualMove(-controller.getRightY());
      }else{
        if(controller.getAButton()){
          // TODO: make an e_Home (that handles the wrist)
          manualDown();
        }else if(controller.getXButton()){
          e_Mid();
        }else if(controller.getBButton()){
          e_GroundPosition();
        }else if(controller.getYButton()){
          // TODO: make an e_High (that handles the wrist)
          manualUp();
        }else {
          stop();
        }
      }
  }

  public void oneDriverModeTeleopPeriodic()
  {
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
    }
  }


  @Override
  public void periodic() {
    if (isAtMaxDown()) {
      resetEncoders();
    } else if (isAtMaxUp()) {
      // Note that this motor is "inverted", so setting position here should be negative.
      rightMotor.getEncoder().setPosition(-ELEVATOR_TOP);
    }

    SmartDashboard.putNumber("Left Elevatator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", getRightPosition());
    SmartDashboard.putString("Elevator State", elevatorState+"");
    SmartDashboard.putBoolean("Lower Switch", isAtMaxDown());
    SmartDashboard.putBoolean("Upper Switch", isAtMaxUp());
    SmartDashboard.putNumber("Stick lever", stick.getRawAxis(3));
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
