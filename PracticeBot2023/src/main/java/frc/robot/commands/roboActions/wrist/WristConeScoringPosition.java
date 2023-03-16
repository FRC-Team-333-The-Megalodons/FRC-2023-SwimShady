// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboActions.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class WristConeScoringPosition extends CommandBase {
  /** Creates a new WristConeScoringPosition. */
  Intake intake;
  public WristConeScoringPosition(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake= intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getRealWristPosition() >= Constants.Wrist.WRIST_CONE_SCORING_POSITION){
      intake.moveWrist(.4);
    }else{
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getRealWristPosition() <= Constants.Wrist.WRIST_CONE_SCORING_POSITION;
  }
}
