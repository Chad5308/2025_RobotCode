package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Util.Constants.constants_Sim;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * OdometryThread combines the functionalities of both the Phoenix and Spark odometry templates.
 * It reads high-frequency measurements from Talon (Phoenix) and Spark motor controllers while ensuring
 * that the drive position timestamp is only logged once per cycle.
 *
 * Fix: Removed duplicate logging of generic signals and timestamps.
 */
public class OdometryThread extends Thread {
  // Lock to prevent conflicts when registering signals
  private final Lock signalsLock = new ReentrantLock();

  // Phoenix (Talon) signals and queues
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();

  // Spark signals and queues
  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();

  // Generic signals and queues (shared between templates)
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();

  // Timestamp queues
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  // Determine if the CAN bus is FD. This can affect how we wait for updates.
  private static boolean isCANFD = new CANBus("rio").isNetworkFD();

  // Singleton instance
  private static OdometryThread instance = null;

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  private OdometryThread() {
    setName("OdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    // Only start if at least one timestamp queue is registered.
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /**
   * Registers a Phoenix (Talon) signal to be logged.
   * @param signal the Phoenix status signal to register.
   * @return a Queue that will receive the logged values.
   */
  public Queue<Double> registerSignal(com.ctre.phoenix6.StatusSignal<?> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }
    return queue;
  }

  /**
   * Registers a Spark signal (from a SparkMax) to be logged.
   * @param spark the Spark motor controller.
   * @param signal a DoubleSupplier that provides the signal value.
   * @return a Queue that will receive the logged values.
   */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * Registers a generic signal to be logged.
   * @param signal a DoubleSupplier that provides the signal value.
   * @return a Queue that will receive the logged values.
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }
    return queue;
  }

  /**
   * Returns a new queue that will receive timestamp values for each sample.
   * @return a Queue for timestamps.
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    // Main logging loop.
    while (true) {
      // Wait for Phoenix (Talon) signal updates if available.
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / constants_Sim.odometryFrequency, phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / constants_Sim.odometryFrequency));
          if (phoenixSignals.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Log sensor data and timestamp.
      Drive.odometryLock.lock();
      try {
        // Compute the sample timestamp (in seconds) adjusted for Phoenix latency.
        double timestamp = RobotController.getFPGATime() / 1e6;
        if (phoenixSignals.length > 0) {
          double totalLatency = 0.0;
          for (BaseStatusSignal signal : phoenixSignals) {
            totalLatency += signal.getTimestamp().getLatency();
          }
          timestamp -= totalLatency / phoenixSignals.length;
        }

        // Log Phoenix (Talon) signals.
        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }

        // Process and log Spark signals (only if all are valid).
        boolean isValid = true;
        double[] sparkValues = new double[sparkSignals.size()];
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkValues[i] = sparkSignals.get(i).getAsDouble();
          if (sparks.get(i).getLastError() != REVLibError.kOk) {
            isValid = false;
          }
        }
        if (isValid) {
          for (int i = 0; i < sparkSignals.size(); i++) {
            sparkQueues.get(i).offer(sparkValues[i]);
          }
        }

        // Log generic signals (logged only once per cycle).
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }

        // Log the timestamp (logged only once per cycle).
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
