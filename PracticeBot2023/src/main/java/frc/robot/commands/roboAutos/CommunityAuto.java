// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.drive.DriveForwardMeters;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommunityAuto extends SequentialCommandGroup {
  /** Creates a new CommunityAuto. */
  public CommunityAuto(Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    frc.robot.utils.PIDController driveController, straightHeadingController;
    driveController = new PIDController(.05, .005, 0, 0, .3, .3, 5);
    straightHeadingController = new PIDController(.5, .05, 0, .15, .2, .2, 0);

    addCommands(new DriveForwardMeters(chassis,gyro,0,driveController,straightHeadingController,true)); 
  }
}
