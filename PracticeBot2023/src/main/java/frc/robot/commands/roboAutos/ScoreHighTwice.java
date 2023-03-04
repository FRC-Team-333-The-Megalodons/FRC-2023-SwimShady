// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos;

import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboActions.elevator.GoHigh;
import frc.robot.commands.roboActions.elevator.GoLow;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighTwice extends SequentialCommandGroup {
  /** Creates a new ScoreHighTwice. */
  frc.robot.utils.PIDController driveBackController,straightHeadingController, turnController;
  public ScoreHighTwice(Chassis chassis, Gyro gyro, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    driveBackController = new PIDController(.013, .01, 0, 35, 5, 1, -Constants.Values.TICKS_PER_METER*2.8);
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.005, .007, 0, 90, .2, .2, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    addCommands(
      new GoHigh(elevator),
      new WaitCommand(2),
      new GoLow(elevator),
      new Drive(chassis,gyro,driveBackController,straightHeadingController,true),
      new WaitCommand(.2),
      new Turn(chassis, gyro, 165, driveBackController, isFinished())
    );
  }
}
