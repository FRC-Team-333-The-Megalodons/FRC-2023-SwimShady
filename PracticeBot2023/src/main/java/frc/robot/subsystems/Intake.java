// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2;
  Joystick stick;
  double ispeed = 0.;
  DoubleSolenoid solenoid;
  MotorControllerGroup wrist;

  public Intake() {
    wristMotor1 = new CANSparkMax(9, MotorType.kBrushless);
    wristMotor2 = new CANSparkMax(10, MotorType.kBrushless);
    stick = new Joystick(0);
    solenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 0);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor1.setInverted(true);
    wristMotor2.setIdleMode(IdleMode.kBrake);
    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);
  }

  public void wristUp() {
    wrist.set(ispeed);
  }

  public void wristDown() {
    wrist.set(-ispeed);
  }

  public void iStop() {
    wrist.set(0);
  }

  public void pSqueeze() {
    solenoid.set(Value.kForward);
  }

  public void pUnsqueeze() {
    solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (stick.getRawButton(1)) {
      //pSqueeze();
    } else {
      //pUnsqueeze();
    }

    if (stick.getPOV() == 0) {
      System.out.print("***");
      wristDown();
    } else if (stick.getPOV() == 180) {
      wristUp();
    } else {
      iStop();
    }

    SmartDashboard.putNumber("wrist 1",wristMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("wrist 2",wristMotor2.getEncoder().getPosition());
  }
}
