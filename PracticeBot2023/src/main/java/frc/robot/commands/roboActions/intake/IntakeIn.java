// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAlternate;

public class IntakeIn extends CommandBase {
  /** Creates a new IntakeIn. */
  IntakeAlternate intake;
  boolean shortDuration;
  boolean cube;
  public IntakeIn(IntakeAlternate intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  public IntakeIn(IntakeAlternate intake, boolean cubeEject) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.cube = cubeEject;
    intake.resetMotorEncoder();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetMotorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cube){
      intake.cubeEject();
    }else{
      intake.intakeIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.intakeAutoDone();
  }
}
