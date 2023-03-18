package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Containers.Metric;

public class Metrics {
    public static final boolean METRICS_ENABLED = true;

    private static Metrics instance = null;

    private HashMap<String, Metric> m_metrics;
    private HashMap<String, Timer> m_timers;
    private long m_count;
    private int m_logFreq;

    private static final int DEFAULT_LOG_FREQ = 500;
    
    private Metrics()
    {
        m_metrics = new HashMap<>();
        m_timers = new HashMap<>();
        m_logFreq = DEFAULT_LOG_FREQ;
    }

    private void startTimer_impl(String name)
    {
        if (!m_timers.containsKey(name)) {
            m_timers.put(name, new Timer());
        }

        Timer timer = m_timers.get(name);
        timer.reset();
        timer.start();
    }

    private void stopTimer_impl(String name)
    {
        if (!m_timers.containsKey(name)) { return; }

        Timer timer = m_timers.get(name);
        timer.stop();

        if (!m_metrics.containsKey(name)) {
            m_metrics.put(name, new Metric(name));
        }
        Metric metric = m_metrics.get(name);
        metric.put(timer.get());
    }

    public void log_impl()
    {
        ++m_count;
        if (m_count < 1 || m_count % m_logFreq != 0) {
            return; // Don't log more than the log frequency
        }

        for(HashMap.Entry<String, Metric> entry : m_metrics.entrySet()) {
            Metric metric = entry.getValue();
            System.out.printf("[METRICS] %s : %.4f \n",metric.getName(),metric.get());
        }
        m_metrics.clear();
        m_timers.clear();
    }

    private static synchronized Metrics getInstance()
    {
        if (instance == null) {
            instance = new Metrics();
        }
        return instance;
    }

    public static void startTimer(String name)
    {
        if (!METRICS_ENABLED) { return; }
        getInstance().startTimer_impl(name);
    }

    public static void stopTimer(String name)
    {
        if (!METRICS_ENABLED) { return; }
        getInstance().stopTimer_impl(name);
    }

    public static void log()
    {
        if (!METRICS_ENABLED) { return; }
        getInstance().log_impl();
    }

}
