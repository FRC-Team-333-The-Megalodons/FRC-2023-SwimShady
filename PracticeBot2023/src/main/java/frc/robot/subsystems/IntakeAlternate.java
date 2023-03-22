// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAlternate extends SubsystemBase {
  /** Creates a new IntakeAlternate. */
  CANSparkMax intakeMotor;
  XboxController controller = new XboxController(1);

  public IntakeAlternate() {
    intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
  }

  public void teleop(){
    if(controller.getRightBumper()){
      intakeMotor.set(1);
    }else if(controller.getLeftBumper()){
      intakeMotor.set(-1);
    }else {
      intakeMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
