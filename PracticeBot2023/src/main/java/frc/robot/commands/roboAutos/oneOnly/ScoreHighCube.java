// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.oneOnly;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.Combo.ElevatorGroundWhileWristAtOrigin;
import frc.robot.commands.roboActions.Combo.ElevatorHighWithWristSafe;
import frc.robot.commands.roboActions.intake.Eject;
import frc.robot.commands.roboActions.intake.IntakeIn;
import frc.robot.commands.roboActions.intake.OpenClaw;
import frc.robot.commands.roboActions.wrist.WristStraight;
import frc.robot.subsystems.IntakeOld;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCube extends SequentialCommandGroup {
  /** Creates a new ScoreHighCube. */
  frc.robot.subsystems.Elevator elevator;
  IntakeOld intake;
  public ScoreHighCube(frc.robot.subsystems.Elevator elevator, IntakeOld intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.intake = intake;

    addCommands(
      new OpenClaw(intake)
      ,new IntakeIn(intake,true)
      ,new ElevatorHighWithWristSafe(elevator, intake)
      ,new WristStraight(intake)
      ,new Eject(intake)
      ,new ElevatorGroundWhileWristAtOrigin(elevator, intake)
    );
  }
}
