// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.funny;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunMeInFinals3 extends SequentialCommandGroup {
  /** Creates a new RunMeInFinals3. */
  public RunMeInFinals3(Chassis chassis, Gyro gyro, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    frc.robot.utils.PIDController driveBackController,driveForwardController, straightHeadingController, turnController;
    driveBackController = new PIDController(500, 1000, 0, 5, 5, 1, -700);//drives back a bit to score hybrid
    driveForwardController = new PIDController(.015, .005, 0, 25, 5, 1, Constants.Values.TICKS_PER_METER*800);
    straightHeadingController = new PIDController(.05, .005, 0, .15, .2, .2, 800);
    turnController = new PIDController(.05, .005, 0, .15, .2, .2, 0);//setting the target on turn controllers will not matter as they will be reset via constructor

    addCommands(
      new Drive(chassis,gyro,driveForwardController,straightHeadingController,true),
      new WaitCommand(1),
      new Drive(chassis, gyro, driveBackController, straightHeadingController, true),
      new WaitCommand(.5),
      new Turn(chassis, gyro, 180, turnController,true),
      new WaitCommand(.5)
    ); 
  }
}
