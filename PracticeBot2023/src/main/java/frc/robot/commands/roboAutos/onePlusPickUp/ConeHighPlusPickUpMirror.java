// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusPickUp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeOld;
import frc.robot.Constants;
import frc.robot.commands.roboActions.Combo.DriveWhileIntaking;
import frc.robot.commands.roboActions.Combo.ElevatorGroundWhileWristGround;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboActions.intake.OpenClaw;
import frc.robot.commands.roboActions.wrist.WristAtOrigin;
import frc.robot.commands.roboAutos.onePlusMobility.ConeHighPlusMobility;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeHighPlusPickUpMirror extends SequentialCommandGroup {
  /** Creates a new ConeHighPlusPickUpMirror. */
  frc.robot.utils.PIDController straightHeadingController, turnController, driveStraightController, resetDriveController, turn0Controller;
  public ConeHighPlusPickUpMirror(Chassis chassis, Gyro gyro, Elevator elevator, IntakeOld intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.0043, .008, 0, 90, .25, .25, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    driveStraightController = new PIDController(.006, .008, 0, 35, 1, 1, Constants.Values.TICKS_PER_METER*.41);
    resetDriveController = new PIDController(.006, .008, 0, 35, 1, 1, -Constants.Values.TICKS_PER_METER*.41);
    turn0Controller = new PIDController(.0043, .008, 0, 90, .25, .25, 0);

    addCommands(
      new ConeHighPlusMobility(chassis, gyro, elevator, intake)
      ,new OpenClaw(intake)
      ,new Turn(chassis, gyro, 175 , turnController, true)
      ,new ElevatorGroundWhileWristGround(elevator, intake)
      ,new DriveWhileIntaking(chassis, gyro, intake, driveStraightController, straightHeadingController, true)//if bad delete
      ,new WristAtOrigin(intake)//if bad delete
      /* 
      ,new Drive(chassis, gyro, driveStraightController, true)
      ,new IntakeIn(intake)
      ,new WristAtOrigin(intake)
      */
    );
  }
}
