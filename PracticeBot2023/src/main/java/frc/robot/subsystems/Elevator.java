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
import frc.robot.utils.Metrics;
import frc.robot.utils.PIDController;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax rightMotor, leftmotor;
  MotorControllerGroup elevator;
  Joystick stick;
  XboxController controller;

  frc.robot.utils.PIDController eGroundPidController;
  RobotStates.ElevatorState elevatorState; //todo let the robot know when it's at low medium or high and add it as a state
  DigitalInput lowerLimitSwitch;
  DigitalInput upperLimitSwitch;
  IntakeAlternate m_intake;
  /*
  Intake intake;

  public Elevator(Intake intake) {
    rightMotor = new CANSparkMax(Constants.RobotMap.PORT_ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.PORT_ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    controller = new XboxController(1);
    eMidPidController = new frc.robot.utils.PIDController(.05, .005, 0, 80, 4, 5,Constants.Elevator.ELEVATOR_POS_MID);
    eGroundPidController = new PIDController(.05, .005, 0, 25, .4, .2,Constants.Elevator.ELEVATOR_POS_GROUND_INTAKE);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rightMotor, leftmotor);
    elevator.setInverted(true);

    lowerLimitSwitch = new DigitalInput(1);
    upperLimitSwitch = new DigitalInput(0);

    this.intake = intake;
  }
  */

  public Elevator(IntakeAlternate intake) {
    rightMotor = new CANSparkMax(Constants.RobotMap.PORT_ELEVATOR1, MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.RobotMap.PORT_ELEVATOR2, MotorType.kBrushless);
    stick = new Joystick(0);
    controller = new XboxController(1);
    eGroundPidController = new PIDController(.045, .007, 0, 25, .4, .2,Constants.Elevator.ELEVATOR_POS_GROUND_INTAKE);

    rightMotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setInverted(true);
    elevator = new MotorControllerGroup(rightMotor, leftmotor);
    elevator.setInverted(true);

    lowerLimitSwitch = new DigitalInput(1);
    upperLimitSwitch = new DigitalInput(0);
    //this.intake = null;
    m_intake = intake;
  }

  public double rampUpRate = 0, rampDownRate = 0;

  public void stop() {
    elevator.set(0);
    rampUpRate = 0;
    rampDownRate = 0;
  }

  public void manualUp()
  {
    // The elevator can't go up if the wrist is in the way.
    /* 
    if (!intake.isWristSafeForElevatorUp() && intake != null) {
      stop();
      return;
    }
    */
  
    if (getRightPosition() > Constants.Elevator.ELEVATOR_UP_SLOWDOWN_POINT) {
      manualMove(Constants.Elevator.ELEVATOR_UP_SLOWDOWN_ESPEED);
    } else {
      rampDownRate = 0;
      if(rampUpRate == 0){
        rampUpRate = .1;
      }else if(rampUpRate != 1){
        rampUpRate += .1;
      }
      manualMove(Constants.Elevator.ELEVATOR_UP_ESPEED*rampUpRate);
    }
  }

  public void manualDown(){
    if (getRightPosition() < Constants.Elevator.ELEVATOR_POS_DOWN_SLOWDOWN_POINT) {
      manualMove(Constants.Elevator.ELEVATOR_DOWN_SLOWDOWN_ESPEED);
    } else {
      rampUpRate = 0;
      if(rampDownRate == 0){
        rampDownRate = .1;
      }else if(rampDownRate != 1){
        rampDownRate += .1;
      }
      manualMove(Constants.Elevator.ELEVATOR_DOWN_ESPEED*rampDownRate);
    }
  }

  public double getHeight()
  {
    return getRightPosition();
  }

  public void manualMove(double speed){
    // Negative speed means down, Positive speed means up.
    // That means that if it's positive, we have to respect isAtMaxUp,
    //  and if negative, we have to respect isAtMaxDown.
    if (speed > 0) {
      if (isAtMaxUp()) {
        stop();
        return;
      }

      if (getRightPosition() > Constants.Elevator.ELEVATOR_UP_SLOWDOWN_POINT) {
        // If we're near the upper limit, slow down so we don't smash through it
        //  (i.e. "cap" our speed at the up-slowdown-speed)
        speed = Math.min(speed, Constants.Elevator.ELEVATOR_UP_SLOWDOWN_ESPEED);
      }
      elevator.set(speed);
      return;
    } 
    
    if (speed < 0) {
      if (isAtMaxDown()) {
        stop();
        return;
      }

      // If the wrist is down, it's not gonna be safe for the elevator to go lower than a certain ponit.
      if (m_intake.getRealWristPosition() < Constants.Wrist.WRIST_POS_THRESHOLD_WHERE_ELEVATOR_NEEDS_TO_STOP
          && getRightPosition() < Constants.Elevator.ELEVATOR_POS_LOWER_THRESHOLD_WHERE_WRIST_NEEDS_TO_BE_LIMITED)
      {
          stop();
          return;
      }

      if (getRightPosition() < Constants.Elevator.ELEVATOR_POS_DOWN_SLOWDOWN_POINT) {
        // If we're near the lower limit, slow down so we don't smash through it
        //  (i.e. "cap" our speed at the down-slowdown speed). We use 'max' because
        //  both of the numbers here (speed and the down_slowdown_speed) are negative.
        speed = Math.max(speed, Constants.Elevator.ELEVATOR_DOWN_SLOWDOWN_ESPEED);
      }
      elevator.set(speed);
      return;
    }

    elevator.set(0);
  }

  public void e_GroundPosition(){
    manualMove(eGroundPidController.getOutput(getRightPosition()));
  }

  public double getRightPosition(){
    return -rightMotor.getEncoder().getPosition();
  }

  public boolean isGroungControllerOnTarget(){
    return eGroundPidController.isOnTarget();
  }

  public void resetEncoders() {
    leftmotor.getEncoder().setPosition(Constants.Elevator.ELEVATOR_POS_BOTTOM);
    rightMotor.getEncoder().setPosition(Constants.Elevator.ELEVATOR_POS_BOTTOM);
  }

  public ElevatorState getState(){
    return elevatorState;
  }

  public boolean isAtMaxUp(){
    if (upperLimitSwitch.get() == false) {
      return true;
    }
    /*
    if (getRightPosition() >= Constants.Elevator.ELEVATOR_POS_TOP) {
      return true;
    }
    */
    return false;
  }
  public boolean isAtMaxDown(){
    if (lowerLimitSwitch.get() == false) {
      return true;
    }
    // TODO: We need to check the wrist position and trust our height is correct
    // so that we can actually stop our sevles from destroying ourselves.


    // We intentionally don't check getPosition at ELEVATOR_DOWN 
    //  as we'd have no way to get a reset position if not.
    return false;
  }

  public void teleopPeriodic(){
    final String metric_key = "Elevator::teleopPeriodic";
    Metrics.startTimer(metric_key);
    teleopPeriodic_impl();
    Metrics.stopTimer(metric_key);
  }

  public void teleopPeriodic_impl()
  {
      if(Math.abs(controller.getRightY()) >= 0.05) {  // "Dead zone" check for Right Joystick
        manualMove(-controller.getRightY());
      }else{
        if(controller.getAButton()){
          // TODO: make an e_Home (that handles the wrist)
          manualDown();
          eGroundPidController.pause();
        }else if(controller.getXButton()){
        
        }else if(controller.getBButton()){
          e_GroundPosition();
        }else if(controller.getYButton()){
          // TODO: make an e_High (that handles the wrist)
          manualUp();
          eGroundPidController.pause();
        }else {
          stop();
          eGroundPidController.pause();
        }
      }

  }

  public void oneDriverModeTeleopPeriodic(){
    if(!RobotContainer.TWO_DRIVER_MODE){
      if (stick.getRawButton(4)) {
        manualUp();
      } else if (stick.getRawButton(3)) {
        if(stick.getRawAxis(3) < .6){
          
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
    final String metric_key = "Elevator::periodic";
    Metrics.startTimer(metric_key);
    periodic_impl();
    Metrics.stopTimer(metric_key);
  }

  public void periodic_impl()
  {
    if (isAtMaxDown()) {
      resetEncoders();
    } else if (isAtMaxUp()) {
      // Note that this motor is "inverted", so setting position here should be negative.
      //rightMotor.getEncoder().setPosition(-Constants.Elevator.ELEVATOR_POS_TOP);
    }

    //SmartDashboard.putNumber("Left Elevator Encoder", leftmotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Encoder", getRightPosition());
    //SmartDashboard.putString("Elevator State", elevatorState+"");
    SmartDashboard.putBoolean("Lower Switch", isAtMaxDown());
    SmartDashboard.putBoolean("Upper Switch", isAtMaxUp());
    //SmartDashboard.putNumber("Stick lever", stick.getRawAxis(3));
  }
}
