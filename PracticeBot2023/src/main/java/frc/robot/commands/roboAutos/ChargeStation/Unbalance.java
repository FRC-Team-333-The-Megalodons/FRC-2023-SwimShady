// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;

public class Unbalance extends CommandBase {
  /** Creates a new Unbalance. */
  Chassis chassis;
  Gyro gyro;
  public Unbalance(Chassis chassis, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis,gyro);
    this.chassis = chassis;
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.lowGear();
    chassis.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  double output = .2;
  @Override
  public void execute() {
    if(output < .88){
      output += .02;
    }
    chassis.arcadeDrive(0, -output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gyro.getUsableTilt() < -8;
  }
}
