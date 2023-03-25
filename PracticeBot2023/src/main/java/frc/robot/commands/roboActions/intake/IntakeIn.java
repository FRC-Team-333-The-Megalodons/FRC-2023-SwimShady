// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeOld;

public class IntakeIn extends CommandBase {
  /** Creates a new IntakeIn. */
  IntakeOld intake;
  boolean shortDuration;
  public IntakeIn(IntakeOld intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  public IntakeIn(IntakeOld intake, boolean shortDuration) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.shortDuration = shortDuration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetIntakeEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.iIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.iStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shortDuration ? intake.shortIntakeAutoDone() : intake.intakeAutoDone();
  }
}
