// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.odometry;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
// ------------------------------------------------------------[Odometry Thread]-----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues.<p>
 * 
 * @author Cody Washington
 * 
 */
public abstract class OdometryThread<SignalType extends Object> extends Thread implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected static final List<Queue<Double>> QUEUES = new ArrayList<>();
  protected final Lock ODOMETRY_LOCK;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  protected static Double OdometryFrequency = (250d);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Odometry Thread Constructor.
   * @param OdometryLocker A Re-entrance Locker for Odometry
   */
  protected OdometryThread(final Lock OdometryLocker) {
    ODOMETRY_LOCK = OdometryLocker;
  }
  // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * Registers a new signal updated at a frequency with the frequency manager.
   * @param Signal Signal source, which can be queried for new signal values
   * @return The {@link Queue} of signal values
   */
  public abstract Queue<Double> register(final SignalType Signal);

  /**
   * Offers each relevant queue to the relevant signal value
   */
  public abstract void run();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  public abstract void close();

  /**
   * Mutates the current frequency of updating the odometry
   * @param Frequency Frequency of odometry updates in Hertz
   */
  public synchronized void setFrequency(final Double Frequency) {
    OdometryFrequency = Frequency;
  }
}
