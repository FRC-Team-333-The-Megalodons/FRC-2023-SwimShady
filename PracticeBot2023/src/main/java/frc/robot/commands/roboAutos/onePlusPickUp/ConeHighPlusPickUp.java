// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusPickUp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboAutos.onePlusMobility.ConeHighPlusMobility;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeHighPlusPickUp extends SequentialCommandGroup {
  /** Creates a new ConeHighPlusPickUp. */
  frc.robot.utils.PIDController straightHeadingController, turnController, driveStraightController, resetDriveController, turn0Controller;
  public ConeHighPlusPickUp(Chassis chassis, Gyro gyro, Elevator elevator, IntakeAlternate intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.0043, .008, 0, 90, .35, .45, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    driveStraightController = new PIDController(.005, .008, 0, 35, 1, 1, Constants.Values.TICKS_PER_METER*.44);
    resetDriveController = new PIDController(.005, .008, 0, 35, 1, 1, -Constants.Values.TICKS_PER_METER*.44);
    turn0Controller = new PIDController(.0043, .008, 0, 90, .25, .25, 0);

    addCommands(
      new ConeHighPlusMobility(chassis, gyro, elevator, intake)
      ,new Turn(chassis, gyro, 0, driveStraightController, isFinished())
    );
  }
}
