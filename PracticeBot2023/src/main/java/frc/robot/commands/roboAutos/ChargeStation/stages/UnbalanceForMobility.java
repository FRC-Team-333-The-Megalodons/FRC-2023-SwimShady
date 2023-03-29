// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation.stages;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;

public class UnbalanceForMobility extends CommandBase {
  /** Creates a new UnbalanceForMobility. */
  Chassis chassis;
  Gyro gyro;
  double output = Constants.Chassis.AUTO_UNBALANCE_START_SPEED;
  public UnbalanceForMobility(Chassis chassis, Gyro gyro) {
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
    output = Constants.Chassis.AUTO_UNBALANCE_START_SPEED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  int zeroCount = 0;
  @Override
  public void execute() {
    if(output < Constants.Chassis.AUTO_UNBALANCE_MAX_SPEED){
      output += Constants.Chassis.AUTO_UNBALANCE_INCREMENT;
    }
    if(gyro.getUsableTilt() == 0){
      zeroCount++;
    }
    if(gyro.getUsableTilt() >= 12){
      output /= 2.5;
    }
    chassis.arcadeDrive(0, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroCount >= 2;
  }
}
