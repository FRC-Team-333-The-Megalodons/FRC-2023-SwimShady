// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ElevatorGroundWhileWristStraight extends CommandBase {
  /** Creates a new ElevatorGroundWhileWristStraight. */
  Elevator elevator;
  Intake intake;
  public ElevatorGroundWhileWristStraight(Elevator elevator, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator,intake);
    this.elevator = elevator;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!elevator.isAtMaxDown() || !elevator.isGroungControllerOnTarget()){
      elevator.e_GroundPosition();
    }
    intake.setWristStaight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isGroungControllerOnTarget() && intake.isWristStraight();
  }
}
