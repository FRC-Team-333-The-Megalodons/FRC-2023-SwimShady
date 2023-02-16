// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2,intakemotor1, intakemotor2;
  MotorControllerGroup wrist, intake;
  Joystick stick;
  DoubleSolenoid solenoid;

  PIDController wristController;

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

    solenoid = new DoubleSolenoid(Constants.RobotMap.PCM_ID, PneumaticsModuleType.CTREPCM, 0, 0);

    wristController = new PIDController(0, 0, 0, 0, 0, 0,0);
  }

  public void resetEncoder(int val) {
    wristMotor1.getEncoder().setPosition(val);
  }

  public void pSqueeze() {
    solenoid.set(Value.kForward);
  }

  public void pUnsqueeze() {
    solenoid.set(Value.kReverse);
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
    if (stick.getRawButton(1)) {
      pSqueeze();
    } else {
      pUnsqueeze();
    }
    if(stick.getRawButton(2)){
      iIn();
    }else if(stick.getRawButton(5)){
      iOut();
    }else{
      iStop();
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
