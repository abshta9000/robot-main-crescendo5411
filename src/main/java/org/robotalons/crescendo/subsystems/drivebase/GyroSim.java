// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.ctre.phoenix6.hardware.Pigeon2;

import org.robotalons.crescendo.Constants.Simulation;
import org.robotalons.lib.motion.sensors.Gyroscope;

import java.util.Queue;
import java.util.function.DoubleSupplier;
// ------------------------------------------------------------[Pigeon Gyroscope]-----------------------------------------------------------//
/**
 *
 *
 * <h1>PigeonGyroscope</h1>
 *
 * <p>Implementation of an auto-logged Gyroscope using a Pigeon as hardware.<p>
 * 
 * @see Gyroscope
 * @see DrivebaseSubsystem
 */
public class GyroSim extends Gyroscope {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Pigeon2 GYROSCOPE = new Pigeon2(Constants.Ports.GYROSCOPE_ID);

  // Since this is a simulation, I am not able to replicate the status signal
  // Substitue would be a double supplier
  private DoubleSupplier YAW_ROTATION; // = GYROSCOPE.getYaw();
  private DoubleSupplier YAW_VELOCITY; // = GYROSCOPE.getAngularVelocityZ();
  private Queue<Double> YAW_ROTATION_QUEUE;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Pigeon Gyroscope Constructor.
   * @param PhoenixDrive Whether or not this gyroscope is based on a CTRE or REV drivebase
   */
  public GyroSim(final Boolean PhoenixDrive, final FlywheelSim preferedAzi) {

    YAW_ROTATION = () -> (preferedAzi.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC);
    YAW_VELOCITY = preferedAzi::getAngularVelocityRadPerSec;

    if (!PhoenixDrive) {
      YAW_ROTATION_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD
        .register(() -> YAW_ROTATION.getAsDouble());
    }

  }

  public synchronized void close() {
    // YAW_ROTATION_QUEUE.clear();
  }

  @Override
  public synchronized void update() {
    Status.Connected = false;
    Status.YawRotation = new Rotation2d(YAW_ROTATION.getAsDouble());
    Status.YawVelocityRadiansSecond = Units.degreesToRadians(YAW_VELOCITY.getAsDouble());
    // Status.PositionDeltas =
    //     YAW_ROTATION_QUEUE.stream()
    //         .map((Value) -> Rotation2d.fromDegrees(Value))
    //         .toArray(Rotation2d[]::new);
    // YAW_ROTATION_QUEUE.clear();
  }
}
