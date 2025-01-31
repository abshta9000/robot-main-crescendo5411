// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.robotalons.crescendo.Constants.Simulation;
import org.robotalons.lib.motion.sensors.Gyroscope;
import java.util.function.DoubleSupplier;
// ------------------------------------------------------------[Pigeon Gyroscope]-----------------------------------------------------------//
/**
 *
 *
 * <h1>PigeonGyroscope</h1>
 *
 * <p>Implementation of an auto-logged Gyroscope using a FlywheelSim.<p>
 * 
 * @see Gyroscope
 * @see DrivebaseSubsystem
 */
public class GyroSim extends Gyroscope {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//

  // Since this is a simulation, I am not able to replicate the status signal
  // Substitue would be a double supplier
  private DoubleSupplier YAW_ROTATION; // = GYROSCOPE.getYaw();
  private DoubleSupplier YAW_VELOCITY; // = GYROSCOPE.getAngularVelocityZ();
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Pigeon Gyroscope Constructor.
   * @param PhoenixDrive Whether or not this gyroscope is based on a CTRE or REV drivebase
   */
  public GyroSim(final Boolean PhoenixDrive, final FlywheelSim preferedAzi) {

    // Assigning the double suppliers with the Sim values
    YAW_ROTATION = () -> (preferedAzi.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC);
    YAW_VELOCITY = preferedAzi::getAngularVelocityRadPerSec;

  }

  public synchronized void close() {
    // what else do i got to close? nothing! so this stays blank
    // YAW_ROTATION_QUEUE.clear();
  }

  @Override
  public synchronized void update() {
    Status.Connected = false;
    Status.YawRotation = new Rotation2d(YAW_ROTATION.getAsDouble());
    Status.YawVelocityRadiansSecond = Units.degreesToRadians(YAW_VELOCITY.getAsDouble());
    // Impossible to use PositionDeltas since i cant use the queue
  }
}
