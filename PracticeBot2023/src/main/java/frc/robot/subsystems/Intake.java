
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.WristStates;
import frc.robot.utils.PIDController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2,intakemotor1, intakemotor2;
  MotorControllerGroup wrist, intake;
  Joystick stick;
  Solenoid solenoid;
  PneumaticHub hub;
  PIDController wristController;
  IntakeStates intakeState;
  WristStates wristState;

  public double INTAKE_SPEED = 0.5;
  public double WRIST_SPEED = 0.15;

  public Intake() {
    wristMotor1 = new CANSparkMax(Constants.RobotMap.WRIST1, MotorType.kBrushless);
    wristMotor1.setInverted(true);
    wristMotor2 = new CANSparkMax(Constants.RobotMap.WRIST2, MotorType.kBrushless);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor2.setIdleMode(IdleMode.kBrake);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);

    intakemotor1 = new CANSparkMax(Constants.RobotMap.INTAKE1, MotorType.kBrushless);
    intakemotor2 = new CANSparkMax(Constants.RobotMap.INTAKE2, MotorType.kBrushless);
    //intakemotor2.setInverted(true);

    intake = new MotorControllerGroup(intakemotor1, intakemotor2);

    stick = new Joystick(0);

    hub = new PneumaticHub(Constants.RobotMap.PCM_ID);

    solenoid = hub.makeSolenoid(Constants.RobotMap.INTAKE_SQUEEZE);

    wristController = new PIDController(0, 0, 0, 0, 0, 0,0);
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
    intake.set(INTAKE_SPEED);
  }
  public void iOut(){
    intake.set(-INTAKE_SPEED);
  }
  public void iStop(){
    intake.set(0);
  }


  public void teleopPeriodic()
  {
    hub.enableCompressorDigital();
    if (stick.getRawButton(1)) {
      pUnsqueeze();

    } else {
      pSqueeze();
    }
    if(stick.getRawButton(2)){
      iIn();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_F;
      }else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    }else if(stick.getRawButton(5)){
      iOut();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_R;
      }else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    }else{
      iStop();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_S;
      }
    }
    if (stick.getPOV() == 0) {
      wrist.set(-WRIST_SPEED);
      wristState = WristStates.ROTATING_IN;
    } else if (stick.getPOV() == 180) {
      wrist.set(WRIST_SPEED);
      wristState = WristStates.ROTATING_OUT;
    } else {
      wrist.set(0);
      wristState = WristStates.MOTORS_STOPPED;
    }

    // TODO: This is a manual encoder reset. During initial development, we're doing this with
    //       a manually pressed button, but in the future this should be done using a Limit Switch
    //       (or, if a potentiometer is used, we don't need to do an encoder reset at all.)
    if (stick.getRawButton(12)) {
      resetEncoder(0);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("wrist", wristMotor1.getEncoder().getPosition());
    SmartDashboard.putString("Wrist State", wristState+"");
    SmartDashboard.putString("Intake State", intakeState+"");
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}
