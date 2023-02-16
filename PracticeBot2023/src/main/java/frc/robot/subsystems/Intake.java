
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.WristStates;
import frc.robot.utils.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2,intakemotor1, intakemotor2;
  MotorControllerGroup wrist, intake;
  Joystick stick;
  DoubleSolenoid solenoid;
  PneumaticHub hub;
  PIDController wristController;
  IntakeStates intakeState;
  WristStates wristState;

  public Intake() {
    wristMotor1 = new CANSparkMax(Constants.RobotMap.WRIST1, MotorType.kBrushless);
    wristMotor1.setInverted(true);
    wristMotor2 = new CANSparkMax(Constants.RobotMap.WRIST2, MotorType.kBrushless);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor2.setIdleMode(IdleMode.kBrake);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);

    intakemotor1 = new CANSparkMax(Constants.RobotMap.INTAKE1, MotorType.kBrushless);
    intakemotor2 = new CANSparkMax(Constants.RobotMap.INTAKE2, MotorType.kBrushless);
    intakemotor2.setInverted(true);

    intake = new MotorControllerGroup(intakemotor1, intakemotor2);

    stick = new Joystick(0);

    hub = new PneumaticHub(Constants.RobotMap.PCM_ID);

    solenoid = hub.makeDoubleSolenoid(4, 6);

    wristController = new PIDController(0, 0, 0, 0, 0, 0,0);
  }

  public void resetEncoder(int val) {
    wristMotor1.getEncoder().setPosition(val);
  }

  public void pSqueeze() {
    solenoid.set(Value.kForward);
    intakeState = IntakeStates.IN;
  }

  public void pUnsqueeze() {
    solenoid.set(Value.kReverse);
    intakeState = IntakeStates.OUT;
  }
  public void iIn(){
    intake.set(.333);
  }
  public void iOut(){
    intake.set(-.333);
  }
  public void iStop(){
    intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hub.enableCompressorDigital();
    if (stick.getRawButton(1)) {
      pSqueeze();
    } else {
      pUnsqueeze();
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
      wrist.set(-.15);
    } else if (stick.getPOV() == 180) {
      wrist.set(.15);
    } else {
      wrist.set(0);
    }

    if (stick.getRawButton(12)) {
      resetEncoder(0);
    }

    SmartDashboard.putNumber("wrist", wristMotor1.getEncoder().getPosition());
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}
