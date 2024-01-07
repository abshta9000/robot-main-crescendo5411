// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.util.function.Supplier;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseModule</h1>
 *
 * <p>Container of Singular unit which assists in the control of motion throughout the competition field
 * 
 * @see DrivebaseModule
 * 
 */
public final class DrivebaseModuleContainer {
    // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
    public Supplier<Double> LINEAR_ENCODER, ROTATION_ENCODER;
    public MotorController LINEAR_CONTROLLER, ROTATION_CONTROLLER;   
    public double PROPORTIONAL_GAIN;
    public double INTEGRAL_GAIN;
    public double DERIVATIVE_GAIN;
    public Nat<Num> DENOTATION; 
    // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Adds motor encoders to this container object
     * @param LinearEncoder   Supplier of velocity measurement data from the axis of the Linear Controller
     * @param RotationEncoder Supplier of position measurement data from the axis of the Rotation Controller
     * @return This object
     */
    public DrivebaseModuleContainer with(final Supplier<Double> LinearEncoder, final Supplier<Double> RotationEncoder) {
        LINEAR_ENCODER = LinearEncoder;
        ROTATION_ENCODER = RotationEncoder;
        return this;
    }

    /**
     * Adds motors controllers to this container object
     * @param LinearController   Motor controller which controls the linear movement, or translation of the module
     * @param RotationController Motor controller which controls the rotational movement, or direction of the module
     * @return This object
     */    
    public DrivebaseModuleContainer with(final MotorController LinearController, final MotorController RotationController) {
        LINEAR_CONTROLLER = LinearController;
        ROTATION_CONTROLLER = RotationController;
        return this;
    }

    /**
     * Adds gains of a PID controller to this container object
     * @param Proportional Defined gain of proportion relative to the system
     * @param Integral     Defined gain of integral relative to the system
     * @param Derivative   Defined gain of dertivative relative to the system
     * @return This object
     */
    public DrivebaseModuleContainer with(final double Proportional, final double Integral, final double Derivative) {
        PROPORTIONAL_GAIN = Proportional;
        INTEGRAL_GAIN = Integral;
        DERIVATIVE_GAIN = Derivative;
        return this;
    }

    /**
     * Adds a module denotation to this container object
     * @param Denotation Natural number representation of which module the module represents
     * @return This object
     */
    public DrivebaseModuleContainer with(final Nat<Num> Denotation) {
        DENOTATION = Denotation;
        return this;
    }  
}
