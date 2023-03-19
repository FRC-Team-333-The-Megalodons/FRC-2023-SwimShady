// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusPickUp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.commands.roboActions.Combo.ElevatorGroundWhileWristGround;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboActions.intake.IntakeIn;
import frc.robot.commands.roboActions.intake.OpenClaw;
import frc.robot.commands.roboActions.wrist.WristAtOrigin;
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
  public ConeHighPlusPickUp(Chassis chassis, Gyro gyro, Elevator elevator, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    turnController = new PIDController(.0043, .008, 0, 90, .25, .25, 0);//setting the target on turn controllers will not matter as they will be reset via constructor
    driveStraightController = new PIDController(.008, .01, 0, 35, 1, 1, Constants.Values.TICKS_PER_METER*.42);
    resetDriveController = new PIDController(.008, .01, 0, 35, 1, 1, -Constants.Values.TICKS_PER_METER*.42);
    turn0Controller = new PIDController(.0043, .008, 0, 90, .25, .25, 0);

    addCommands(
      new ConeHighPlusMobility(chassis, gyro, elevator, intake)
      ,new OpenClaw(intake)
      ,new Turn(chassis, gyro, 185, turnController, true)
      ,new ElevatorGroundWhileWristGround(elevator, intake)
      ,new Drive(chassis, gyro, driveStraightController, true)
      ,new IntakeIn(intake)
      ,new WristAtOrigin(intake)
    );
  }
}
