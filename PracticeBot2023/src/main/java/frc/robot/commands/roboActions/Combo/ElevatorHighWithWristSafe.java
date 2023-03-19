// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.Combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ElevatorHighWithWristSafe extends CommandBase {
  /** Creates a new ElevatorHighWithWristStraightCone. */
  frc.robot.subsystems.Elevator elevator;
  Intake intake;
  public ElevatorHighWithWristSafe(frc.robot.subsystems.Elevator elevator, Intake intake) {
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
    if(!elevator.isAtMaxUp()){
      elevator.manualUp();
    }else{
      elevator.stop();
    }
    if(intake.getRealWristPosition() >= Constants.Wrist.WRIST_LIMIT_FOR_ELEVATOR_UP){//60 for mid
      intake.moveWrist(Constants.Wrist.WRIST_DOWN_SPEED);
    }else{
      intake.stop();
    }
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
    return elevator.isAtMaxUp();
  }
}
