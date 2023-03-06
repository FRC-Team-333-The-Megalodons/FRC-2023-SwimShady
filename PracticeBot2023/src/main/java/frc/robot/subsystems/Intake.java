// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.WristStates;
import frc.robot.utils.PIDController;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2,intakemotor1, intakemotor2;
  MotorControllerGroup wrist, intake;
  Joystick stick;
  XboxController controller;
  Solenoid solenoid;
  PneumaticHub hub;
  PIDController wristOriginController;
  IntakeStates intakeState;
  WristStates wristState;
  DutyCycleEncoder wristEncoder;

  public double INTAKE_SPEED = .7;
  public double WRIST_SPEED = 0.65;

  double wristValue;
  DecimalFormat df1 = new DecimalFormat("0.##");
  final double wristOrigin = 0;
  final double wrist90 = 0;
  final double wristLimit = 0;

  public Intake(PneumaticHub hub) {
    wristMotor1 = new CANSparkMax(Constants.RobotMap.WRIST1, MotorType.kBrushless);
    wristMotor1.setInverted(false);
    wristMotor2 = new CANSparkMax(Constants.RobotMap.WRIST2, MotorType.kBrushless);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor2.setIdleMode(IdleMode.kBrake);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);

    intakemotor1 = new CANSparkMax(Constants.RobotMap.INTAKE1, MotorType.kBrushless);
    intakemotor2 = new CANSparkMax(Constants.RobotMap.INTAKE2, MotorType.kBrushless);
    
    intakemotor1.setIdleMode(IdleMode.kBrake);
    intakemotor2.setIdleMode(IdleMode.kBrake);

    intake = new MotorControllerGroup(intakemotor1, intakemotor2);

    stick = new Joystick(0);
    controller = new XboxController(1);

    this.hub = hub;

    solenoid = hub.makeSolenoid(Constants.RobotMap.INTAKE_SQUEEZE);

    wristEncoder = new DutyCycleEncoder(9);
    wristEncoder.setConnectedFrequencyThreshold(900);
    wristEncoder.reset();
    wristOriginController = new PIDController(0.04, 0.002, 0, 5, .5, .5,0);
  }

  public void resetEncoder(int val) {
    wristMotor1.getEncoder().setPosition(val);
  }

  public void pSqueeze() {
    solenoid.set(true);
    intakeState = IntakeStates.IN;
  }

  public void pUnsqueeze() {
    solenoid.set(false);
    intakeState = IntakeStates.OUT;
  }
  public void iIn(){
    intakemotor1.set(-INTAKE_SPEED);
    intakemotor2.set(INTAKE_SPEED);
  }
  public void iOut(){
    intakemotor1.set(.2);
    intakemotor2.set(-.2);
  }
  public void iStop(){
    intake.set(0);
  }

  public void resetWrist(){
    wrist.set(wristOriginController.getOutput(wristValue));
  }

  public boolean wristAtOrigin(){
    return wristOriginController.isOnTarget();
  }

  public void teleopPeriodic() {
    hub.enableCompressorDigital();

    if(!RobotContainer.TWO_DRIVER_MODE){
      if (stick.getRawButton(1)) {
        pUnsqueeze();

      } else {
        pSqueeze();
      }

      if (stick.getRawButton(2)){
        //iIn();
        iIn();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_F;
        }else{
          intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      } else if(stick.getRawButton(5)){ 
        iOut();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_R;
        } else{
          intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      } else{
        iStop();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_S;
        }
      }

      if (stick.getPOV() == 0) {
        wrist.set(WRIST_SPEED);
        wristState = WristStates.ROTATING_IN;
      } else if (stick.getPOV() == 180) {
        wrist.set(-WRIST_SPEED);
        wristState = WristStates.ROTATING_OUT;
      } else if(stick.getPOV() == 90){
        resetWrist();
      }else {
        wrist.set(0);
        wristState = WristStates.MOTORS_STOPPED;
      }
    }else{
      if(controller.getRightTriggerAxis() > .05){
        pUnsqueeze();
      }else{
        pSqueeze();
      }

      if(controller.getRightBumper()){
        iIn();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_F;
        }else{
          intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      }else if(controller.getLeftBumper()){
        iOut();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_R;
        } else{
          intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      }else{
        iStop();
        if(intakeState == IntakeStates.OUT){
          intakeState = IntakeStates.OUT_AND_MOTORS_S;
        }
      }
      if (controller.getPOV() == 0) {
        wrist.set(-WRIST_SPEED);
        wristState = WristStates.ROTATING_IN;
      } else if (controller.getPOV() == 180) {
        wrist.set(WRIST_SPEED);
        wristState = WristStates.ROTATING_OUT;
      } else if(stick.getPOV() == 90){
        resetWrist();
      }else {
        wrist.set(0);
        wristState = WristStates.MOTORS_STOPPED;
      }
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristValue = (Double.valueOf(df1.format(wristEncoder.getAbsolutePosition())) * 100)-78;
    SmartDashboard.putString("Wrist State", wristState+"");
    SmartDashboard.putString("Intake State", intakeState+"");
    SmartDashboard.putNumber("WristEncoder", wristValue);
    SmartDashboard.putBoolean("WristAtOrigin", wristAtOrigin());
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}