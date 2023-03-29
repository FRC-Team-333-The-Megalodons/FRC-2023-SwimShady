// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.utils.PIDController;

public class DriveWhileIntaking extends CommandBase {
  /** Creates a new DriveWhileIntaking. */

  Chassis chassis;
  Gyro gyro;
  IntakeAlternate intake;
  frc.robot.utils.PIDController turnController, driveController;
  boolean encodersReset;
  boolean cone;

  public DriveWhileIntaking(Chassis chassis, Gyro gyro, IntakeAlternate intake, PIDController driveController, PIDController straightHeadingController,  boolean encodersReset, boolean cone) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis,gyro);
    this.chassis = chassis;
    this.gyro = gyro;
    this.intake = intake;
    this.driveController = driveController;
    this.turnController = straightHeadingController;
    this.encodersReset = encodersReset;
    this.cone = cone;
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
    chassis.arcadeDrive(turnController != null ? turnController.getOutput(gyro.getAngle()) : 0, driveController.getOutput(chassis.getEncodersAverage()));
    if(cone){
      intake.intakeIn();
    }else{
      intake.eject();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.arcadeDrive(0, 0);
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveController.isOnTarget();
  }
}
