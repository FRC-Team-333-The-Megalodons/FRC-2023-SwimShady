// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
 * Use https://www.desmos.com/calculator/vcdedzbmag to input values and simulate the PID algorythm. This should prevent the need to eyeball values
 * This class uses the basic concept of the PID algorythm to accurately bring the robot to a place where it should be
 * 
 * in a nutshell:
 * 
 * The missile knows where it is at all times. It knows this because it knows where it isn't. 
 * By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater), it obtains a difference, or deviation. 
 * The guidance subsystem uses deviations to generate corrective commands to drive the missile from a position where it is to a position where it isn't, and arriving at a position where it wasn't, it now is. 
 * Consequently, the position where it is, is now the position that it wasn't, and it follows that the position that it was, is now the position that it isn't.
 * In the event that the position that it is in is not the position that it wasn't, the system has acquired a variation, the variation being the difference between where the missile is, and where it wasn't. 
 * If variation is considered to be a significant factor, it too may be corrected by the GEA. 
 * However, the missile must also know where it was.
 * The missile guidance computer scenario works as follows. 
 * Because a variation has modified some of the information the missile has obtained, it is not sure just where it is. 
 * However, it is sure where it isn't, within reason, and it knows where it was. 
 * It now subtracts where it should be from where it wasn't, or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be, and where it was, it is able to obtain the deviation and its variation, which is called error.
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
    
    boolean autoKill;

    Timer timer;

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
        timer = new Timer();
    }

    public PIDController(double kP, double kI, double kD, double iLimit, double maxTolerance, double minTolerance, double target, boolean autoKill){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iLimit = iLimit;
        this.maxTolerance = maxTolerance;
        this.minTolerance = minTolerance;
        this.target = target;
        minTarget = target - minTolerance;
        maxTarget = target + maxTolerance;
        this.autoKill = autoKill;
        timer = new Timer();
    }

    public double getOutput(double sensorValue){
        SmartDashboard.putNumber("Target", target);
        SmartDashboard.putNumber("error", error);
        SmartDashboard.putBoolean("Is on target",isOnTarget);
        timer.start();
        currentTime = timer.get() - lastTimeStamp;
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
        lastTimeStamp = timer.get();
        lastError = error;

        isOnTarget = maxTarget >= sensorValue && minTarget <= sensorValue;
        
        if(isOnTarget && autoKill){
            error = 0;
            errorSum = 0;
            errorRate = 0;
            timer.reset();
            timer.stop();
            return 0;
        }
        return (kP * error) + (kI * errorSum) + (kD * errorRate);//only PI for now
    }

    public void pause(){
        error = 0;
        errorSum = 0;
        errorRate = 0;
        timer.reset();
        timer.stop();
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