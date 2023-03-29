// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeAlternate;

public class GoMidSmooth extends CommandBase {
  /** Creates a new GoMidSmooth. */
  IntakeAlternate intakeAlternate;
  Elevator elevator;
  public GoMidSmooth(IntakeAlternate intakeAlternate, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeAlternate, elevator);
    this.intakeAlternate = intakeAlternate;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeAlternate.wristToMid();
    elevator.manualUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeAlternate.stopWrist();
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeAlternate.midController.isOnTarget() && elevator.isAtMaxUp();
  }
}
