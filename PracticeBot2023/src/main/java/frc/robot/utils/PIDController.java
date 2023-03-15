// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
 * Use https://www.desmos.com/calculator/vcdedzbmag to input values and simulate the PID algorythm. This should prevent the need to eyeball values
 * This class uses the basic concept of the PID algorythm to accurately bring the robot to a place where it should be
 */
public class PIDController {

    final double kP;
    final double kI;
    final double kD;

    final double iLimit;
    double maxTolerance;
    double minTolerance;
    double target;
    double maxTarget;
    double minTarget;

    double error = 0;
    double output = 0;
    double errorSum = 0;
    double errorRate = 0;
    double lastError = 0;
    double lastTimeStamp = 0;
    double currentTime = 0;
    double sensorValue;

    boolean greaterThanMax = false;
    boolean lowerThanMin = false;
    boolean isOnTarget = false;

    public PIDController(double kP, double kI, double kD, double iLimit, double maxTolerance, double minTolerance, double target){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iLimit = iLimit;
        this.maxTolerance = maxTolerance;
        this.minTolerance = minTolerance;
        this.target = target;
        minTarget = target - minTolerance;
        maxTarget = target + maxTolerance;
    }

    public double getOutput(double sensorValue){
        SmartDashboard.putNumber("Target", target);
        SmartDashboard.putNumber("error", error);
        currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
        if(sensorValue > maxTarget){
            error = maxTarget - sensorValue;
            greaterThanMax = true;
            lowerThanMin = false;
        }

        if(sensorValue < minTarget){
            error = minTarget - sensorValue;
            greaterThanMax = false;
            lowerThanMin = true;
        }

        if(Math.abs(error) < iLimit && Math.round(error) != 0){
            errorSum += error * currentTime;
        }

        errorRate = (error - lastError)/currentTime;
    
        lastTimeStamp = Timer.getFPGATimestamp();
        lastError = error;

        isOnTarget = maxTarget >= sensorValue && minTarget <= sensorValue;
        
        if(isOnTarget){
            error = 0;
            errorSum = 0;
            errorRate = 0;
            return 0;
        }
        return (kP * error) + (kI * errorSum) + (kD * errorRate);//only PI for now
    }

    public void setTarget(double minTolerance, double maxTolerance, double target){
        this.target = target;
        this.maxTolerance = maxTarget;
        this.minTolerance = minTolerance;
        this.maxTarget = target + maxTolerance;
        this.minTarget = target - minTolerance;
    }

    public double getP(){return kP;}
    public double getI(){return kI;}
    public double getD(){return kD;}

    public double getILimit(){return iLimit;}
  
    public double getMaxTolerance(){return maxTolerance;}
    public double getMinTolerance(){return minTolerance;}

    public double getError(){return error;}
    public double getErrorSum(){return errorSum;}

    public double getLastTimeStamp(){return lastTimeStamp;}
    public double getCurrentTime(){return currentTime;}

    public double getSensorValue(){return sensorValue;}
    
    public boolean isGreaterThanMax(){return greaterThanMax;}
    public boolean isLowerThanMin(){return lowerThanMin;}
    public boolean isOnTarget(){return isOnTarget;}
}