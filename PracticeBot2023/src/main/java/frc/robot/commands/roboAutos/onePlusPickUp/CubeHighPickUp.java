// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.onePlusPickUp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboActions.Combo.DriveWhileIntaking;
import frc.robot.commands.roboActions.Combo.GoHome;
import frc.robot.commands.roboActions.Combo.IntakeConePosition;
import frc.robot.commands.roboActions.drive.Drive;
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
  frc.robot.utils.PIDController straightHeadingController,driveStraightController, driveBackController;
  public CubeHighPickUp(frc.robot.subsystems.Elevator elevator, IntakeAlternate intakeAlternate, Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 180);
    driveStraightController = new PIDController(.02, .008, 0, 35, 1, 1, Constants.Values.TICKS_PER_METER*.333);
    driveBackController = new PIDController(.02, .008, 0, 35, 1, 1, -Constants.Values.TICKS_PER_METER*2.515);

    addCommands(
      new CubeHighPlusMobility(chassis, gyro, elevator, intakeAlternate)
      ,new IntakeConePosition(intakeAlternate, elevator)
      ,new DriveWhileIntaking(chassis, gyro, intakeAlternate, driveStraightController, straightHeadingController, true, true)
      ,new GoHome(intakeAlternate, elevator)
      ,new Drive(chassis, gyro, driveBackController, straightHeadingController, true)
    );
  }
}
