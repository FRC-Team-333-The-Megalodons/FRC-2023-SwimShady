// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeAlternate;

public class IntakeConePosition extends CommandBase {
  /** Creates a new IntakeConePosition. */
  IntakeAlternate intake;
  Elevator elevator;

  public IntakeConePosition(IntakeAlternate intake, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake,elevator);
    this.intake = intake;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.wristToIntake();
    elevator.e_GroundPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWrist();
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.intakeConeController.isOnTarget() || elevator.isGroungControllerOnTarget();
  }
}
