package frc.robot.utils;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Containers {

public static class Multi_CANSparkMax
{
    private IdleMode m_lastIdleMode;
    private ArrayList<CANSparkMax> m_sparkMaxArray;
    public Multi_CANSparkMax(CANSparkMax ... sparkMaxArray)
    {
        m_sparkMaxArray = new ArrayList<>();
        for (CANSparkMax sparkMax : sparkMaxArray) {
            m_sparkMaxArray.add(sparkMax);
        }
    }

    public CANSparkMax getLeader(){
        return m_sparkMaxArray.get(0);
    }

    public void setIdleMode(IdleMode idleMode)
    {
        if (idleMode == m_lastIdleMode) { return; }
        for (CANSparkMax sparkMax : m_sparkMaxArray) 
        {
            sparkMax.setIdleMode(idleMode);
        }
        m_lastIdleMode = idleMode;
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
    private double m_lastPos;

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

    public RelativeEncoder getLeader()
    {
        return m_encoderArray.get(0);
    }

    public void setPosition(double pos)
    {
        if (pos == m_lastPos) { return; }

        for (RelativeEncoder encoder : m_encoderArray)
        {
            encoder.setPosition(pos);
        }
        m_lastPos = pos;
    }



    public double getAveragePosition()
    {
        return getLeader().getPosition();

        /* This might be slowing things down?
        double numerator = 0.0;
        double denominator = 0;
        for (RelativeEncoder encoder : m_encoderArray)
        {
            numerator += encoder.getPosition();
            denominator += 1;
        }

        return numerator/denominator;
        */
    }
}

}

