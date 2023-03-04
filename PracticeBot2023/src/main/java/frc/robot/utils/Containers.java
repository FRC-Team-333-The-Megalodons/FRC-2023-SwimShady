package frc.robot.utils;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class Containers {
    
}

class Multi_CANSparkMax
{
    private ArrayList<CANSparkMax> m_sparkMaxArray;
    public Multi_CANSparkMax(CANSparkMax ... sparkMaxArray)
    {
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
}

class Multi_RelativeEncoder
{
    private ArrayList<RelativeEncoder> m_encoderArray;
    public Multi_RelativeEncoder(RelativeEncoder ... encoderArray)
    {
        for (RelativeEncoder encoder : encoderArray)
        {
            m_encoderArray.add(encoder);
        }
    }
}



