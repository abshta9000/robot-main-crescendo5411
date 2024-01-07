// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

import java.io.Closeable;
// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
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
     * and how it should identify & calculate reference 'set-points'
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
     */
    void set(final SwerveModuleState Reference);

    /**
     * Mutates the module controller's current mode of operation and how it should identify & calculate reference 'set-points'
     * @param Mode Mode of Module control
     */
    void set(final ModuleMode Mode);

    /**
     * Resets the module's azimuth 
     */
    void reset();

    /**
     * Provides a natural {@link Nat number} representation of how the module is internally denoted (i.e. Front-Right (1), Front-Left (2)...)
     * @return Nat of system denotation
     */
    Nat<Num> getDenotation();

    /**
     * Provides a {@link SwerveModuleState state} of the device's 'set point', or reference states
     * @return State of system references
     */
    SwerveModuleState getReference();

    /**
     * Provides a {@link SwerveModuleState state} of the device's most-recent control cycle outputs
     * @return State of system calculated inputs
     */
    SwerveModuleState getCalculated();

    /**
     * Provides a {@link SwerveModuleState state} of the device's most-recent plant cycle outputs
     * @return State of system inputs
     */
    SwerveModuleState getOutputs();
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
