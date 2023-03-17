// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.two;

import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboAutos.onePlusMobility.ConeHighPlusMobility;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighTwice extends SequentialCommandGroup {
  /** Creates a new ScoreHighTwice. */
  frc.robot.utils.PIDController driveBackController,straightHeadingController, turnController, turn180Controller, driveToGridController, alignController;
  public ScoreHighTwice(Chassis chassis, Gyro gyro, Elevator elevator, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.005, .007, 0, 90, .2, .2, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    turn180Controller = new PIDController(.005, .007, 0, 90, .2, .2, 0);
    driveToGridController = new PIDController(.013, .01, 0, 35, 5, 1, Constants.Values.TICKS_PER_METER * 2.1);
    alignController = new PIDController(.002, .007, 0, 90, .2, .2, 0);
    addCommands(
      new ConeHighPlusMobility(chassis, gyro, elevator, intake)
    );
  }
}