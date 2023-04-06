// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusPickUp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboActions.Combo.DriveWhileGoHome;
import frc.robot.commands.roboActions.Combo.DriveWhileIntaking;
import frc.robot.commands.roboActions.Combo.IntakeCubePosition;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboAutos.onePlusMobility.CubeHighPlusMobility;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeHighPickUp extends SequentialCommandGroup {
  /** Creates a new CubeHighPickUp. */
  frc.robot.utils.PIDController straightHeadingController,driveStraightController, driveBackController, turnToGlassController;
  public CubeHighPickUp(frc.robot.subsystems.Elevator elevator, IntakeAlternate intakeAlternate, Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 183);
    driveStraightController = new PIDController(.05, .019, 0, 35, 5, 5, Constants.Values.TICKS_PER_METER*.9);
    driveBackController = new PIDController(.012, .01, 0, 35, 1, 1, -Constants.Values.TICKS_PER_METER*3);
    turnToGlassController = new PIDController(.0038, .0055, 0, 60, .45, .45, 0);

    addCommands(
      new CubeHighPlusMobility(chassis, gyro, elevator, intakeAlternate)
      ,new IntakeCubePosition(intakeAlternate, elevator)
      ,new DriveWhileIntaking(chassis, gyro, intakeAlternate, driveStraightController, straightHeadingController,true,false)
      ,new DriveWhileGoHome(chassis, gyro, intakeAlternate, elevator, driveBackController, straightHeadingController)
      ,new Turn(chassis, gyro, 360, turnToGlassController,true)
    );
  }
}
