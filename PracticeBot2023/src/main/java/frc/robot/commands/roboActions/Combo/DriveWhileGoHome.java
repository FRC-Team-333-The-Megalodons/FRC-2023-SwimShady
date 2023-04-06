// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.utils.PIDController;

public class DriveWhileGoHome extends CommandBase {
  /** Creates a new DriveWhileGoHome. */
  Chassis chassis;
  Gyro gyro;
  IntakeAlternate intake;
  Elevator elevator;
  PIDController driveController, straightController;

  public DriveWhileGoHome(Chassis chassis, Gyro gyro, IntakeAlternate intake,Elevator elevator, PIDController driveController, PIDController straightHeadingController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis,gyro,intake);
    this.chassis = chassis;
    this.gyro = gyro;
    this.intake = intake;
    this.elevator = elevator;
    this.driveController = driveController;
    this.straightController = straightHeadingController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.arcadeDrive(straightController != null ? straightController.getOutput(gyro.getAngle()) : 0, driveController.getOutput(chassis.getEncodersAverage()));
    intake.wristIn();
    elevator.manualDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWrist();
    elevator.stop();
    chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveController.isOnTarget();
  }
}
