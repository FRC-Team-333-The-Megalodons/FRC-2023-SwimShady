// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusMobility;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboAutos.oneOnly.ScoreHighCube;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeHighPlusMobility extends SequentialCommandGroup {
  /** Creates a new CubePlusMobility. */
  frc.robot.utils.PIDController driveForwardController, straightHeadingController, turnToGamePieceController;
  public CubeHighPlusMobility(Chassis chassis, Gyro gyro,Elevator elevator, IntakeAlternate intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveForwardController = new PIDController(.0145, .01, 0, 35, 5, 1, -Constants.Values.TICKS_PER_METER*2.7);
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnToGamePieceController = new PIDController(.0045, .007, 0, 60, .35, .35, 0);
    addCommands(
      new ScoreHighCube(elevator, intake,chassis,gyro)
      ,new Drive(chassis,gyro,driveForwardController,straightHeadingController,true)
      ,new Turn(chassis, gyro, 177, turnToGamePieceController, true)
    );
  }
}
