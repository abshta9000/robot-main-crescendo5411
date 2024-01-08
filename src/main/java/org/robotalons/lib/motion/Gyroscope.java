// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.Closeable;
import java.io.IOException;
// ------------------------------------------------------------[Drivebase Module]-----------------------------------------------------------//
/**
 *
 *
 * <h1>GyroscopeModule</h1>
 *
 * <p>Singular unit which assists in the sensor feedback of a drivebase by providing heading information of a drivebase.
 * 
 */
public interface Gyroscope extends Closeable {

  /**
   * Updates the underlying signals within this module.
   */
  void update();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  void close() throws IOException;

  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */
  Boolean getConnected();

  /**
   * Provides the most-recent rotational velocity of yaw
   * @return Double representing rotational velocity as radians per second
   */
  Double getYawRotationalVelocity();

  /**
   * Provides the most-recent measurement yaw
   * @return Rotation in radians of yaw position
   */
  Rotation2d getYawRotation();

  /**
   * Provides the most-recent measurements of 
   * @return
   */
  Rotation2d[] getOdometryYawRotations();





}
