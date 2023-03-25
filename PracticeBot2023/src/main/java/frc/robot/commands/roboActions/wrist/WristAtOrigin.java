// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeOld;

public class WristAtOrigin extends CommandBase {
  /** Creates a new WristAtOrigin. */
  IntakeOld intake;
  public WristAtOrigin(IntakeOld intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.moveWrist(-.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isAtMaxUp();
  }
}
