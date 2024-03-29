// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.PIDController;

public class Turn extends CommandBase {
  /** Creates a new TurnLeftDegrees. */

  Chassis chassis;
  frc.robot.subsystems.Gyro gyro;
  double degrees;
  PIDController controller;
  boolean encodersReset;

  public Turn(Chassis chassis, frc.robot.subsystems.Gyro gyro, double degrees, PIDController controller, boolean encodersReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis,gyro);
    this.chassis = chassis;
    this.gyro = gyro;
    this.degrees = degrees;
    this.controller = controller;
    this.encodersReset = encodersReset;
    controller.setTarget(controller.getMinTolerance(), controller.getMaxTolerance(), degrees);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(encodersReset){
      chassis.resetEncoders();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.arcadeDrive(controller.getOutput(gyro.getAngle()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.isOnTarget();
  }
}
