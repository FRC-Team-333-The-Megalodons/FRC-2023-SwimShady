// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.DutyCycleDataJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ElevatorState;
import frc.robot.utils.RobotMath;

public class Elevator extends SubsystemBase {
  final double HEIGHT_MAX_LIMIT = 70.0;
  final double HEIGHT_HIGH_SHOT = 69.5;
  final double HEIGHT_MID_SHOT = 50.0;
  final double HEIGHT_INTAKE = 5.0;
  final double HEIGHT_MIN_LIMIT = 0.0;

  CANSparkMax m_leftSpark, m_rightSpark;
  Intake m_intake;
  RelativeEncoder m_leftEncoder, m_rightEncoder;
  MotorControllerGroup m_motors;
  Joystick m_stick;
  XboxController m_controller;
  final double ESPEED = 1;

  frc.robot.utils.PIDController m_pidController;
  RobotStates.ElevatorState m_elevatorState; //todo let the robot know when it's at low medium or high and add it as a state


  DigitalInput m_lowerLimitSwitch;
  DigitalInput m_upperLimitSwitch;

  public Elevator(Intake intake) {
    m_intake = intake;

    m_leftSpark = new CANSparkMax(Constants.RobotMap.ELEVATOR2, MotorType.kBrushless);
    m_leftSpark.setInverted(true);
    m_rightSpark = new CANSparkMax(Constants.RobotMap.ELEVATOR1, MotorType.kBrushless);

    setIdleMode(IdleMode.kBrake);
    m_motors = new MotorControllerGroup(m_rightSpark, m_leftSpark);
    m_motors.setInverted(true);

    m_leftEncoder = m_leftSpark.getEncoder();
    m_rightEncoder = m_rightSpark.getEncoder();
    
    m_lowerLimitSwitch = new DigitalInput(0);
    m_upperLimitSwitch = new DigitalInput(1);

    m_stick = new Joystick(0);
    m_controller = new XboxController(1);
    m_pidController = new frc.robot.utils.PIDController(.05, .005, 0, 80, 4, 5,95);
  }

  private ElevatorState calculateStateFromSensors()
  {
    double wristAngle = m_intake.getWristEncoder().get();
    double elevatorHeight = m_rightEncoder.getPosition();
  
    // The "HOME" state is defined by being at the Lower Limit Switch.
    if (m_lowerLimitSwitch.get())
    {
      // If the limit switch is pressed, reset the encoder.
      m_rightEncoder.setPosition(HEIGHT_MIN_LIMIT);
      if (wristAngle <= Intake.ANGLE_WRIST_IN)
      {
        return ElevatorState.ELEVATOR_DOWN_WRIST_IN__HOME;
      }

      if (wristAngle >= Intake.ANGLE_WRIST_INTAKE)
      {
        return ElevatorState.ELEVATOR_DOWN_WRIST_DOWN__INTAKE;
      }

      if (wristAngle >= Intake.ANGLE_WRIST_SAFE)
      {
        return ElevatorState.ELEVATOR_DOWN_WRIST_OUT_SAFE;
      }

      return ElevatorState.ELEVATOR_DOWN_WRIST_TRAVERSING_IN_TO_SAFE;
    }

    if (m_upperLimitSwitch.get())
    {
      m_rightEncoder.setPosition(HEIGHT_MAX_LIMIT);
      if (wristAngle >= Intake.ANGLE_WRIST_PLACE)
      {
        return ElevatorState.ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_PLACE;
      }

      if (wristAngle >= Intake.ANGLE_WRIST_SHOOT)
      {
        return ElevatorState.ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_SHOOT;
      }

      return ElevatorState.ELEVATOR_HIGH_WRIST_IN;
    }

    if (RobotMath.isApprox(elevatorHeight, HEIGHT_MID_SHOT)) {
      if (RobotMath.isApprox(wristAngle, Intake.ANGLE_WRIST_SHOOT)) {
        return ElevatorState.ELEVATOR_MID_WRIST_OUT__LOWGOAL_SHOOT;
      } else if (RobotMath.isApprox(wristAngle, Intake.ANGLE_WRIST_PLACE)) {
        return ElevatorState.ELEVATOR_MID_WRIST_OUT__LOWGOAL_PLACE;
      } else {
        return ElevatorState.ELEVATOR_MID_WRIST_IN;
      }
    }

    if (RobotMath.isApprox(elevatorHeight, HEIGHT_HIGH_SHOT)) {
      if (RobotMath.isApprox(wristAngle, Intake.ANGLE_WRIST_SHOOT)) {
        return ElevatorState.ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_SHOOT;
      } else if (RobotMath.isApprox(wristAngle, Intake.ANGLE_WRIST_PLACE)) {
        return ElevatorState.ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_PLACE;
      } else {
        return ElevatorState.ELEVATOR_HIGH_WRIST_IN;
      }
    }

    // If we've made it this far, it means we're somewhere between the various setpoints.
    if (elevatorHeight > HEIGHT_MIN_LIMIT && elevatorHeight < HEIGHT_MID_SHOT)
    {
      return ElevatorState.ELEVATOR_TRAVERSING_DOWN_TO_MID;
    }
    if (elevatorHeight < HEIGHT_HIGH_SHOT && elevatorHeight > HEIGHT_MID_SHOT)
    {
      return ElevatorState.ELEVATOR_TRAVERSING_MID_TO_HIGH;
    }

    // If we don't recognize anything about where we are, or there's some gap in this logic,
    //  it's better to return a known "unknown" variable. In this case, only manual controls
    //  will work (rather than risk the robot eating itself).
    return ElevatorState.ELEVATOR_UNKNOWN;
  }

  private void setIdleMode(IdleMode idleMode)
  {
    m_leftSpark.setIdleMode(idleMode);
    m_rightSpark.setIdleMode(idleMode);
  }

  // Commenting out an older version of the state-derivation function
  /*
  private void setState(double output){
    if(output > 0){
        if(-m_rightSpark.getEncoder().getPosition() > 95){
          m_elevatorState = ElevatorState.TRAVERSING_HIGH_FROM_MID;
        }else if(-m_rightSpark.getEncoder().getPosition() < 83){
          m_elevatorState = ElevatorState.TRAVERSING_UP_FROM_LOW;
        }
    }else{
      if(-m_rightSpark.getEncoder().getPosition() > 95){
        m_elevatorState = ElevatorState.TRAVERSING_DOWN_FROM_HIGH;
      }else if(-m_rightSpark.getEncoder().getPosition() < 83){
        m_elevatorState = ElevatorState.TRAVERSING_DOWN_FROM_MID;
      }
    }

    if(-m_rightSpark.getEncoder().getPosition() <= 95 || -m_rightSpark.getEncoder().getPosition() >= 90){
      m_elevatorState = ElevatorState.MEDIUM;
    }
    if(m_rightSpark.getEncoder().getPosition() == 0){
      m_elevatorState = ElevatorState.LOW;
    }
  }
  */

  public void stop() {
    m_motors.set(0);
  }

  public void moveElevator(double speed)
  {
    // This function allows us to feed direct input into the elevator, but safely (i.e. respects limit switches)
    if (m_lowerLimitSwitch.get() || m_lowerLimitSwitch.get()) { return; }

    m_motors.set(speed);
  }

  

  public void manualUp(){
    if(m_lowerLimitSwitch.get() == false){
      m_motors.set(0);
      return;
    }

    m_motors.set(ESPEED);
  }

  public void manualDown(){
    if(m_upperLimitSwitch.get() == false){
      m_motors.set(0);
      m_rightSpark.getEncoder().setPosition(0);
      return;
    }else {
      m_motors.set(-(ESPEED-.2));
    }
  }

  public void e_Mid(){
    if(m_lowerLimitSwitch.get() == false ){
      m_motors.set(0);//acts as an emegency stop in case the pid fails
      return;
    }else{
      m_motors.set(m_pidController.getOutput(-m_rightSpark.getEncoder().getPosition()));
    }
  }

  public boolean isControllerOnTarget(){
    return m_pidController.isOnTarget();
  }

  public void resetEncoders() {
    m_leftSpark.getEncoder().setPosition(0);
    m_rightSpark.getEncoder().setPosition(0);
  }

  public double averageEncoderDistance() {
    return (m_leftSpark.getEncoder().getPosition() + m_rightSpark.getEncoder().getPosition()) / 2;
  }

  public ElevatorState getState(){
    return m_elevatorState;
  }

  public boolean isAtMaxUp(){return !m_lowerLimitSwitch.get();}
  public boolean isAtMaxDown(){return !m_upperLimitSwitch.get();}

  public void teleopPeriodic() {
    if(!RobotContainer.TWO_DRIVER_MODE){
      if (m_stick.getRawButton(4)) {
        manualUp();
      } else if (m_stick.getRawButton(3)) {
        if(m_stick.getRawAxis(3) < .6){
          e_Mid();
        }else{
          manualDown();
        }
      }else{
        if(m_stick.getRawAxis(3) < .6){
          manualDown();
        }else{
          m_motors.set(0);
        }
      }

      return;
    }
    
    // Josh & Eman from 2023-02-04 -- The control design here is as follows:
    // A button - hold for home
    // X button & B button - hold for mid-height
    // Y button - hold for hight
    // Down on DPAD - Go to intake from floor ( auto spin the intake? )
    // Left Stick - wrist
    // Right Stick - elevator 
    // Left on DPAD - yellow LED
    // Right on Dpad - purple LED
    // Default start state is claw is closed
    // R2 - Shoot out
    // L2 - Suck in
    // R1 - Open
    // L1 - Close


    /// Automatic buttons (A, X+B, Y, Down on DPAD) override manual equivalents (and must themselves have some heirarchy.)
    /// For now, let's say A > X+B > Y > Down

    final double JOYSTICK_THRESHOLD = 0.05;
    final double DPAD_UP = 0, DPAD_RIGHT = 90, DPAD_DOWN = 180, DPAD_LEFT = 270;
    final double DPAD_DIAGONAL_DIFF = 45;

    if (m_controller.getAButton()) {
      // TODO: Invoke routine to bring the elevator & wrist home
    } else if (m_controller.getXButton() || m_controller.getBButton()) {
      // TODO: Invoke routine to bring the elevator & wrist to mid
    } else if (m_controller.getYButton()) {
      // TODO: Invoke routine to bring the elevator & wrist to high
    } else if (m_controller.getPOV() >= (DPAD_RIGHT+DPAD_DIAGONAL_DIFF) && m_controller.getPOV() <= (DPAD_LEFT-DPAD_DIAGONAL_DIFF)) {
      // TODO: Invoke routine 
    } else {
      // None of the automatic buttons were pressed, so we can respect both manual joysticks.
      if (Math.abs(m_controller.getRightY()) > JOYSTICK_THRESHOLD) {
        moveElevator(m_controller.getRightY());
      }
      if (Math.abs(m_controller.getLeftY()) > JOYSTICK_THRESHOLD) {
        m_intake.moveWrist(m_controller.getLeftY());
      }
    }

    final double TRIGGER_THRESHOLD = 0.05;

    // Movement of the claw & its motors don't rely on anything else
    if (m_controller.getRightBumper()) {
      m_intake.pSqueeze();
    } else if (m_controller.getLeftBumper()) {
      m_intake.pUnsqueeze();
    }
    if (m_controller.getRightTriggerAxis() > TRIGGER_THRESHOLD) {
      m_intake.runIntakeMotors(-1 * m_controller.getRightTriggerAxis());
    } else if (m_controller.getLeftTriggerAxis() > TRIGGER_THRESHOLD) {
      m_intake.runIntakeMotors(m_controller.getLeftTriggerAxis());
    }
    
    /*** ORIGINAL CONTROLS CODE - RE-enable if needed for testing (and comment out the above block)
    if(m_controller.getYButton()){
      manualUp();
    }else if(m_controller.getAButton() && m_controller.getLeftTriggerAxis() < .5){
      if(m_stick.getRawAxis(3) < .6){
        e_Mid();
      }else{
        manualDown();
      }
    }else{
      if(m_stick.getRawAxis(3) < .6 && m_controller.getLeftTriggerAxis() < .5){
        manualDown();
      }else{
        m_motors.set(0);
      }
    }
    */
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevatator Encoder", m_leftSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevatator Encoder", -m_rightSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("Average Encoder Distance", averageEncoderDistance());
    SmartDashboard.putString("Elevator State", m_elevatorState+"");
    SmartDashboard.putBoolean("Lower Switch", m_lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Upper Switch", m_upperLimitSwitch.get());
    SmartDashboard.putNumber("Stick lever", m_stick.getRawAxis(3));
  }
}

/*
 * Elevator Encoder Values
 * 1. 69.5 max extension, high shot
 * 2. 50 mid shot
 * 3. 0 home position low shot
 */
