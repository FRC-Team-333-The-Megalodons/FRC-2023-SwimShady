// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusMobility;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboAutos.oneOnly.ScoreMidCube;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.utils.PIDController;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeMidPlusMobility extends SequentialCommandGroup {
  /** Creates a new CubeMidPlusMobility. */
  frc.robot.utils.PIDController driveForwardController, straightHeadingController, turnToGamePieceController;
  public CubeMidPlusMobility(Chassis chassis, Gyro gyro,frc.robot.subsystems.Elevator elevator, IntakeAlternate intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveForwardController = new PIDController(.014, .01, 0, 35, 3, 1, -Constants.Values.TICKS_PER_METER*2.5);
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnToGamePieceController = new PIDController(.0052, .007, 0, 60, .35, .35, 0);
    addCommands(
      new ScoreMidCube(elevator, intake,chassis,gyro)
      ,new Drive(chassis,gyro,driveForwardController,straightHeadingController,true)
      ,new Turn(chassis, gyro, 180, turnToGamePieceController, true)
    );
  }
}
