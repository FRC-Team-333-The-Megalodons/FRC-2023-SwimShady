// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.two;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboActions.intake.IntakeIn;
import frc.robot.commands.roboActions.wrist.WristAtOrigin;
import frc.robot.commands.roboActions.wrist.WristDownToIntake;
import frc.robot.commands.roboAutos.mobility.MobilityOnly;
import frc.robot.commands.roboAutos.oneOnly.ScoreHighCone;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmanAuto extends SequentialCommandGroup {
  /** Creates a new EmanAuto. */
  public EmanAuto(frc.robot.subsystems.Chassis chassis, Gyro gyro, Elevator elevator, IntakeAlternate intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MobilityOnly(chassis, gyro)
      ,new WristDownToIntake(null)
      ,new IntakeIn(null)
      ,new WristAtOrigin(null)
      ,new Drive(chassis, gyro, null, true)//drive back
      ,new Turn(chassis, gyro, 360, null, true)
      ,new ScoreHighCone(elevator, null)
    );
  }
}
