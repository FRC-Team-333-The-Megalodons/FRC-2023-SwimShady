// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class PIDController {

    final double kP;
    final double kI;
    final double kD;

    final double iLimit;
    final double maxTolerance;
    final double minTolerance;

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

    public PIDController(double kP, double kI, double kD, double iLimit, double maxTolerance, double minTolerance){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iLimit = iLimit;
        this.maxTolerance = maxTolerance;
        this.minTolerance = minTolerance;
    }

    public double getOutput(double sensorValue){
        currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
        if(sensorValue > maxTolerance){
            error = maxTolerance - sensorValue;
            greaterThanMax = true;
            lowerThanMin = false;
        }

        if(sensorValue < minTolerance){
            error = minTolerance - sensorValue;
            greaterThanMax = false;
            lowerThanMin = true;
        }

        if(Math.abs(error) < iLimit && Math.round(error) != 0){
            errorSum += error * currentTime;
        }

        errorRate = (error - lastError)/currentTime;
    
        lastTimeStamp = Timer.getFPGATimestamp();
        lastError = error;

        return (kP * error) + (kI * errorSum) + (kD * errorRate);//only PI for now
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
}