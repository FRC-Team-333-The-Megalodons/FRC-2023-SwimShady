// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.Combo.ElevatorHighWithWristStraight;
import frc.robot.commands.roboActions.elevator.GoGround;
import frc.robot.commands.roboActions.elevator.GoHigh;
import frc.robot.commands.roboActions.intake.Eject;
import frc.robot.commands.roboActions.intake.IntakeIn;
import frc.robot.commands.roboActions.intake.OpenClaw;
import frc.robot.commands.roboActions.wrist.WristAtOrigin;
import frc.robot.commands.roboActions.wrist.WristDownToIntake;
import frc.robot.commands.roboActions.wrist.WristStraight;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCube extends SequentialCommandGroup {
  /** Creates a new ScoreHighCube. */
  frc.robot.subsystems.Elevator elevator;
  Intake intake;
  public ScoreHighCube(frc.robot.subsystems.Elevator elevator, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.intake = intake;

    addCommands(
      new IntakeIn(intake)
      ,new ElevatorHighWithWristStraight(elevator, intake)
      ,new GoHigh(elevator)
      ,new Eject(intake)
      ,new WristAtOrigin(intake)
      ,new GoGround(elevator)
    );
  }
}
