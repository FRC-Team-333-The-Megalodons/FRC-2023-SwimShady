package frc.robot.utils;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Containers {

public static class Multi_CANSparkMax
{
    private ArrayList<CANSparkMax> m_sparkMaxArray;
    public Multi_CANSparkMax(CANSparkMax ... sparkMaxArray)
    {
        m_sparkMaxArray = new ArrayList<>();
        for (CANSparkMax sparkMax : sparkMaxArray) {
            m_sparkMaxArray.add(sparkMax);
        }
    }

    public void setIdleMode(IdleMode idleMode)
    {
        for (CANSparkMax sparkMax : m_sparkMaxArray) 
        {
            sparkMax.setIdleMode(idleMode);
        }
    }

    public Multi_RelativeEncoder getMultiEncoder()
    {
        ArrayList<RelativeEncoder> encoderArray = new ArrayList<>();
        for (CANSparkMax sparkMax : m_sparkMaxArray)
        {
            encoderArray.add(sparkMax.getEncoder());
        }
        return new Multi_RelativeEncoder(encoderArray);
    }

    public MotorControllerGroup getMotorControllerGroup()
    {
        return new MotorControllerGroup(m_sparkMaxArray.toArray(new CANSparkMax[m_sparkMaxArray.size()]));
    }
}

public static class Multi_RelativeEncoder
{
    private ArrayList<RelativeEncoder> m_encoderArray;
    public Multi_RelativeEncoder(RelativeEncoder ... encoderArray)
    {
        m_encoderArray = new ArrayList<>();
        for (RelativeEncoder encoder : encoderArray)
        {
            m_encoderArray.add(encoder);
        }
    }

    public Multi_RelativeEncoder(ArrayList<RelativeEncoder> encoderArray)
    {
        m_encoderArray = encoderArray;
    }

    public void setPosition(double pos)
    {
        for (RelativeEncoder encoder : m_encoderArray)
        {
            encoder.setPosition(pos);
        }
    }

    public double getAveragePosition()
    {
        double numerator = 0.0;
        double denominator = 0;
        for (RelativeEncoder encoder : m_encoderArray)
        {
            numerator += encoder.getPosition();
            denominator += 1;
        }

        return numerator/denominator;
    }
}

}

