// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.io.Closeable;
import java.io.IOException;
// ------------------------------------------------------------[Drivebase Module]-----------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseModule</h1>
 *
 * <p>Singular unit which assists in the control of motion throughout the competition field, which provides implementation for control of both
 * an azimuth (Rotation) motor, which controls the angle or direction of the module, and an linear (translation) motor which controls the velocity
 * or magnitude of the module.
 * 
 */
public interface DrivebaseModule extends Closeable {
  // --------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * Mutates the module controller's current 'set-point' or reference state and mutates the module controller's current mode of operation
   * and how it should identify and calculate reference 'set-points'
   * @param Reference Module's new Goal or 'set-point' reference
   * @param Mode Mode of Module control
   */
  default void set(final SwerveModuleState Reference, final ModuleMode Mode) {
      set(Mode);
      set(Reference);
  }

  /**
   * Mutates the module controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Reference Module's new Goal or 'set-point' reference
   * @return An optimized version of the reference
   */
  SwerveModuleState set(final SwerveModuleState Reference);

  /**
   * Mutates the module controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of Module control
   */
  void set(final ModuleMode Mode);

  /**
   * Updates the underlying signals within this module.
   */
  void update();

  /**
   * Resets the module's azimuth to the absolute position of the encoder
   */
  void reset();

  /**
   * Quickly sets relevant references to zero and stops this module from moving
   */
  void stop();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  void close() throws IOException;

  /**
   * Provides an accurate {@link SwerveModulePosition position} of this module at the current time derived from sensor information.
   * @return Position of system
   */
  SwerveModulePosition getPosition();

  /**
   * Provides accurate deltas of {@link SwerveModulePosition positions} of this module in the most-recent cycle
   * @return Array of positions of system
   */
  SwerveModulePosition[] getPositionDeltas();

  /**
   * Provides an un-optimized {@link SwerveModuleState state} of the device's 'set point', or reference states
   * @return State of system references
   */
  SwerveModuleState getReference();

  /**
   * Provides an optimized {@link SwerveModuleState state} of the device's 'set point', or reference states
   * @return State of system references
   */
  SwerveModuleState getOptimized();

  /**
   * Provides an {@link SwerveModuleState state} of the device's most-recent control cycle outputs
   * @return State of system calculated inputs
   */
  SwerveModuleState getOutput();

  /**
   * Provides an {@link SwerveModuleState state} of the device's most-recent plant cycle outputs
   * @return State of system inputs
   */
  SwerveModuleState getMeasured();

  /**
   * Provides a natural {@link Nat number} representation of how the module is internally denoted (i.e. Front-Right (1), Front-Left (2)...)
   * @return Nat of system denotation
   */
  Nat<Num> getDenotation();
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------//
  /**
   * <p>Describes a given {@link DrivebaseModule}'s current state of control, and how it should operate given the mode.
   */
  public enum ModuleMode {
      CHARACTERIZATION,        
      STATE_CONTROL,
      DISABLED,
      CLOSED,
  }
}
