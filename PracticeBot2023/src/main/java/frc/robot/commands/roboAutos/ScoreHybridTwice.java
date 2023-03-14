// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboActions.intake.Eject;
import frc.robot.commands.roboActions.intake.IntakeIn;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHybridTwice extends SequentialCommandGroup {
  /** Creates a new ScoreHybridTwice. */
  frc.robot.utils.PIDController driveBackController,straightHeadingController, turnController, turn180Controller, driveToGridController, alignController;
  public ScoreHybridTwice(Chassis chassis, Gyro gyro, Elevator elevator, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    driveBackController = new PIDController(.013, .01, 0, 35, 5, 1, -Constants.Values.TICKS_PER_METER * 2.4);
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.005, .007, 0, 90, .2, .2, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    turn180Controller = new PIDController(.005, .007, 0, 90, .2, .2, 0);
    driveToGridController = new PIDController(.013, .01, 0, 35, 5, 1, Constants.Values.TICKS_PER_METER * 2.1);
    alignController = new PIDController(.002, .007, 0, 90, .2, .2, 0);
    addCommands(
      new Eject(intake)
      ,new Drive(chassis,gyro,driveBackController,straightHeadingController,true)
      ,new WaitCommand(.1)
      ,new Turn(chassis, gyro, 185, turnController, true)
      ,new IntakeIn(intake)
      ,new WaitCommand(.1)
      ,new Turn(chassis, gyro, 0, turn180Controller, true)
      ,new WaitCommand(.1)
      ,new Drive(chassis,gyro,driveToGridController,straightHeadingController,true)
      ,new WaitCommand(.1)
      ,new Turn(chassis, gyro, 2.5, alignController, true)
      ,new Eject(intake)
    );
  }
}
