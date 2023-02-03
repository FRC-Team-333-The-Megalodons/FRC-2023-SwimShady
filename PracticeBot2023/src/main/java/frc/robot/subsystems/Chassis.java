// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Don't worry about these, just playing around
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class Chassis extends SubsystemBase {

    PneumaticsControlModule pcm;

    PWMVictorSPX motor1;
    PWMVictorSPX motor2;
    PWMVictorSPX motor3;
    PWMVictorSPX motor4;
    PWMVictorSPX motor5;
    PWMVictorSPX motor6;
    //Compressor compressor;
    DoubleSolenoid shift;

    MotorControllerGroup leftDrive;
    MotorControllerGroup rightDrive;
    DifferentialDrive drive;

    Joystick shtiq;

    TalonFX falcon = new TalonFX(-1);

    public Chassis() {

        /*Instantiate drivetraain motors*/

        motor1 = new PWMVictorSPX(Constants.DeviceIDs.LEFT_LEADER_ID);
        motor2 = new PWMVictorSPX(Constants.DeviceIDs.LEFT_FOLLOWER1_ID);
        motor3 = new PWMVictorSPX(Constants.DeviceIDs.LEFT_FOLLOWER2_ID);
        motor4 = new PWMVictorSPX(Constants.DeviceIDs.RIGHT_LEADER_ID);
        motor5 = new PWMVictorSPX(Constants.DeviceIDs.RIGHT_FOLLOWER1_ID);
        motor6 = new PWMVictorSPX(Constants.DeviceIDs.RIGHT_FOLLOWER2_ID);

        //compressor = new Compressor(Constants.DeviceIDs.PNEUMATICS_CONTROL_MODULE,PneumaticsModuleType.CTREPCM);
        shift = new DoubleSolenoid(Constants.DeviceIDs.PNEUMATICS_CONTROL_MODULE,
                                   PneumaticsModuleType.CTREPCM, 
                                   Constants.DeviceIDs.DRIVETRAIN_SOLENOID_LOW, 
                                   Constants.DeviceIDs.DRIVETRAIN_SOLENOID_HIGH);

        leftDrive   = new MotorControllerGroup(motor1, motor2, motor3);
        rightDrive  = new MotorControllerGroup(motor4, motor5, motor6);
        drive       = new DifferentialDrive(leftDrive, rightDrive);

        shtiq = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);
    }

    public void lowTrans() {
        shift.set(Value.kForward);
    }
    public void highTrans() {
        shift.set(Value.kReverse);
    }

    public void stop() {
        drive.arcadeDrive(0, 0);
    }

    public void drive(double x, double y) {
        drive.arcadeDrive(x, y);
    }

    @Override
    public void periodic() {
        //compressor.enableDigital();
        drive.arcadeDrive(-shtiq.getX(), shtiq.getY());

        if (shtiq.getRawButton(2)) {
            lowTrans();
        } else {
            highTrans();
        }
    }
}