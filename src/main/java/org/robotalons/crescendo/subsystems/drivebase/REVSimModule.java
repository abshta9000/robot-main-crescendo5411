// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.drivebase;
// unfortunately most of this code is copied from cody since my understanding of tcb is very little

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import org.robotalons.crescendo.Constants.Simulation;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.lib.motion.actuators.Module;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

// ----------------------------------------------------------[REV Controller Module]--------------------------------------------------------//
/**
 *
 *
 * <h1>REVSimModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes a Flywheel Neo sim.</p>
 * 
 * @see Module
 * @see DrivebaseSubsystem
 */
public class REVSimModule extends Module{
  /** Creates a new DrivebaseModuleCANSparkMax. */

  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final ModuleConstants CONSTANTS;
  private final List<SwerveModulePosition> DELTAS;
  // ds for faster response times (i think)
  private DoubleSupplier LinearAppliedVolts;
  private DoubleSupplier RotationalAppliedVolts;
  private DoubleSupplier CurrentPosition;
  private final Lock ODOMETRY_LOCK;
  private final Queue<Double> LINEAR_QUEUE;
  private final Queue<Double> ROTATIONAL_QUEUE;  
  private ReferenceType ReferenceMode;

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private double RotationalRelativePosition = 0;
  private double RotationalAbsolutePosition = 0;

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Simulator Module Constructor
   * @param Constants Constants of new module instance
   */
  public REVSimModule(final ModuleConstants Constants) {
    super(Constants);
    CONSTANTS = Constants;
    ReferenceMode = ReferenceType.STATE_CONTROL;
    DELTAS = new ArrayList<>();

    LinearAppliedVolts = () -> 0;
    RotationalAppliedVolts = () -> 0;
    CurrentPosition = () -> 0;

    ODOMETRY_LOCK = new ReentrantLock();

    LINEAR_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (Status.LinearPositionRadians));
    ROTATIONAL_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (Status.RotationalAbsolutePosition.getRadians())); 
  }

  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class ModuleConstants extends Constants {
    public FlywheelSim LINEAR_CONTROLLER;
    public FlywheelSim ROTATIONAL_CONTROLLER;
    public WPI_CANCoder ABSOLUTE_ENCODER;
    public PIDController LINEAR_CONTROLLER_PID;
    public PIDController ROTATIONAL_CONTROLLER_PID;
    public SimpleMotorFeedforward LINEAR_CONTROLLER_FEEDFORWARD;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void close() {
    LINEAR_QUEUE.clear();
    ROTATIONAL_QUEUE.clear();
  }

  @Override
  public void cease() {
    CONSTANTS.LINEAR_CONTROLLER.setState((0d));
    CONSTANTS.ROTATIONAL_CONTROLLER.setState((0d));
  }

  @Override
  public void update() {
    CONSTANTS.LINEAR_CONTROLLER.update(Simulation.SIMULATION_LOOPPERIOD_SEC);
    CONSTANTS.ROTATIONAL_CONTROLLER.update(Simulation.SIMULATION_LOOPPERIOD_SEC);


    // code block modified from mechanical advantage
      double angleDiffRad = CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC;
      RotationalRelativePosition += angleDiffRad;
      RotationalAbsolutePosition += angleDiffRad;
      // reverses negative rotation to positive
      // ie -3.14 -> 3.14 (same position)
      while (RotationalAbsolutePosition < 0) {
        RotationalAbsolutePosition += 2.0 * Math.PI;
      }
      // lowers overshoot
      // ie 7.85 - 3.14 = 3.92
      while (RotationalAbsolutePosition > 2.0 * Math.PI) {
        RotationalAbsolutePosition -=  Math.PI;
      }
      // if (RotationalAbsolutePosition > 2.0 * Math.PI){
      //   RotationalAbsolutePosition = 0;
      //   // System.out.println(RotationalAbsolutePosition / Simulation.SIMULATION_LOOPPERIOD_SEC);
      //   CONSTANTS.ROTATIONAL_CONTROLLER.setState(RotationalAbsolutePosition / Simulation.SIMULATION_LOOPPERIOD_SEC);
      //   System.out.println(CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC);
      // }

    Status.RotationalAbsolutePosition = Rotation2d.fromRadians(RotationalAbsolutePosition);
    Status.RotationalRelativePosition = Rotation2d.fromRadians(RotationalRelativePosition);
    Status.RotationalVelocityRadiansSecond = CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec();
    Status.RotationalAppliedVoltage = RotationalAppliedVolts.getAsDouble();


    Status.LinearPositionRadians = (CONSTANTS.LINEAR_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC);
    Status.LinearVelocityRadiansSecond = CONSTANTS.LINEAR_CONTROLLER.getAngularVelocityRadPerSec();
    Status.LinearAppliedVoltage = LinearAppliedVolts.getAsDouble();

    Status.OdometryLinearPositionsRadians =
      LINEAR_QUEUE.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / Measurements.ROTATION_GEAR_RATIO)
        .toArray();
    Status.OdometryAzimuthPositions =
      ROTATIONAL_QUEUE.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / Measurements.LINEAR_GEAR_RATIO))
        .toArray(Rotation2d[]::new);
  }

  @Override
  public void periodic() {
    ODOMETRY_LOCK.lock(); 
    switch(ReferenceMode) {
      case STATE_CONTROL:
        if (Objects.isNull(Azimuth_Offset) && Status.RotationalAbsolutePosition.getRadians() != (0d)) {
          Azimuth_Offset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
        }
        if (!Objects.isNull(Reference.angle)) {
          setRotationVoltage(CONSTANTS.ROTATIONAL_CONTROLLER_PID.calculate(getRelativeRotation().getRadians(),Reference.angle.getRadians()));
          if(!Objects.isNull(Reference.speedMetersPerSecond)) {
            var AdjustReferenceSpeed = Reference.speedMetersPerSecond * Math.cos(CONSTANTS.ROTATIONAL_CONTROLLER_PID.getPositionError()) / CONSTANTS.WHEEL_RADIUS_METERS;
            setLinearVoltage(
              (CONSTANTS.LINEAR_CONTROLLER_PID.calculate(AdjustReferenceSpeed))
                                    +
              (CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD.calculate(Status.LinearVelocityRadiansSecond, AdjustReferenceSpeed)));
          }
        }      
        break;
      case DISABLED:
        cease();
      case CLOSED:
        close();
        break;
    }
    DELTAS.clear();
    IntStream.range((0), Math.min(Status.OdometryLinearPositionsRadians.length, Status.OdometryAzimuthPositions.length)).forEach((Index) -> {
      DELTAS.add(new SwerveModulePosition((getLinearPositionRads() - CurrentPosition.getAsDouble()), getRelativeRotation()));
    });
    ODOMETRY_LOCK.unlock(); 
  }

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  @Override
  public SwerveModuleState set(SwerveModuleState Reference) {
    this.Reference = SwerveModuleState.optimize(Reference, getAbsoluteRotation());
    return this.Reference;
  }

  @Override
  public void set(ReferenceType Mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

/**
   * Mutator for the Rotational Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setRotationVoltage(final Double Demand) {
    RotationalAppliedVolts = () -> Demand;
    CONSTANTS.ROTATIONAL_CONTROLLER.setInputVoltage(Demand);

  }

  /**
   * Mutator for the Linear Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setLinearVoltage(final Double Demand) {
    LinearAppliedVolts = () -> Demand;
    CONSTANTS.LINEAR_CONTROLLER.setInputVoltage(Demand);
  }

  // --------------------------------------------------------------[Acessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    return DELTAS;
  }

  @Override
  public Rotation2d getRelativeRotation() {
    return (Objects.isNull(Azimuth_Offset))? (new Rotation2d()): (Status.RotationalRelativePosition.plus(Azimuth_Offset));
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    // System.out.println(Rotation2d.fromRadians(RotationalAbsolutePosition));
    return Rotation2d.fromRadians(RotationalAbsolutePosition);
  }

  public Double getLinearPositionRads() {
    return Status.LinearPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS;
  }
}
