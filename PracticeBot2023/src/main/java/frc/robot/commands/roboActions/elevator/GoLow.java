// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class GoLow extends CommandBase {
  /** Creates a new GoLow. */

  Elevator elevator;

  public GoLow(Elevator elavator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elavator);
    this.elevator = elavator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.manualDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtMaxDown();
  }
}
