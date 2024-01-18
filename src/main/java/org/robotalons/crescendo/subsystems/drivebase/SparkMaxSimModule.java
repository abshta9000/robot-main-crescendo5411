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

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import org.robotalons.crescendo.Constants.Simulation;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;


public class SparkMaxSimModule extends Module{
  /** Creates a new DrivebaseModuleCANSparkMax. */

  private final ModuleConstants CONSTANTS;
  private final List<SwerveModulePosition> DELTAS;
 
  // ds for faster response times (i think)
  private DoubleSupplier fwdAppliedVolts;
  private DoubleSupplier aziAppliedVolts;

  private DoubleSupplier CurrentPosition;

  private double aziRelativePositionRad = 0;
  private double aziAbsolutePositionRad = 0;

  private final Lock ODOMETRY_LOCK;
  private final Queue<Double> LINEAR_QUEUE;
  private final Queue<Double> ROTATIONAL_QUEUE;  
  // private final RelativeEncoder LINEAR_ENCODER;
  // private final RelativeEncoder ROTATIONAL_ENCODER;
  private ReferenceType ReferenceMode;


  public SparkMaxSimModule(final ModuleConstants Constants) {
    super(Constants);
    CONSTANTS = Constants;
    ReferenceMode = ReferenceType.STATE_CONTROL;
    DELTAS = new ArrayList<>();

    fwdAppliedVolts = () -> 0;
    aziAppliedVolts = () -> 0;
    CurrentPosition = () -> 0;

    ODOMETRY_LOCK = new ReentrantLock();
    // LINEAR_ENCODER = CONSTANTS.LINEAR_CONTROLLER;
    // ROTATIONAL_ENCODER = CONSTANTS.ROTATIONAL_CONTROLLER.getEncoder();

    // LINEAR_ENCODER.setPosition((0.0));
    // LINEAR_ENCODER.setMeasurementPeriod((10));
    // LINEAR_ENCODER.setAverageDepth((2));

    // ROTATIONAL_ENCODER.setPosition((0.0));
    // ROTATIONAL_ENCODER.setMeasurementPeriod((10));
    // ROTATIONAL_ENCODER.setAverageDepth((2));



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

  @Override
  public void close() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void cease() {
    // TODO Auto-generated method stub
    // // throw new UnsupportedOperationException("Unimplemented method 'cease'");
  }

  @Override
  public void update() {
    CONSTANTS.LINEAR_CONTROLLER.update(Simulation.SIMULATION_LOOPPERIOD_SEC);
    CONSTANTS.ROTATIONAL_CONTROLLER.update(Simulation.SIMULATION_LOOPPERIOD_SEC);


    // code block modified from mechanical advantage
      double angleDiffRad = CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC;
      aziRelativePositionRad += angleDiffRad;
      aziAbsolutePositionRad += angleDiffRad;
      // reverses negative rotation to positive
      // ie -3.14 -> 3.14 (same position)
      while (aziAbsolutePositionRad < 0) {
        aziAbsolutePositionRad += 2.0 * Math.PI;
      }
      // lowers overshoot
      // ie 7.85 - 3.14 = 3.92
      while (aziAbsolutePositionRad > 2.0 * Math.PI) {
        aziAbsolutePositionRad -=  Math.PI;
      }

    Status.RotationalAbsolutePosition = Rotation2d.fromRadians(aziAbsolutePositionRad);
    System.out.println(Status.RotationalAbsolutePosition);
    Status.RotationalRelativePosition = Rotation2d.fromRadians(aziRelativePositionRad);
    Status.RotationalVelocityRadiansSecond = CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec();
    Status.RotationalAppliedVoltage = aziAppliedVolts.getAsDouble();


    Status.LinearPositionRadians = (CONSTANTS.LINEAR_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC);
    Status.LinearVelocityRadiansSecond = CONSTANTS.LINEAR_CONTROLLER.getAngularVelocityRadPerSec();
    Status.LinearAppliedVoltage = fwdAppliedVolts.getAsDouble();

    Status.OdometryLinearPositionsRadians =
      LINEAR_QUEUE.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.ROTATION_GEAR_RATIO)
        .toArray();
    Status.OdometryAzimuthPositions =
      ROTATIONAL_QUEUE.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / CONSTANTS.LINEAR_GEAR_RATIO))
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
    aziAppliedVolts = () -> Demand;
    CONSTANTS.ROTATIONAL_CONTROLLER.setInputVoltage(Demand);

  }

  /**
   * Mutator for the Linear Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setLinearVoltage(final Double Demand) {
    fwdAppliedVolts = () -> Demand;
    CONSTANTS.LINEAR_CONTROLLER.setInputVoltage(Demand);
  }

  // --------------------------------------------------------------[ACESSOR METHODS]--------------------------------------------------------------
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
    return Rotation2d.fromDegrees(CONSTANTS.ABSOLUTE_ENCODER.getAbsolutePosition());
  }

  public Double getLinearPositionRads() {
    return Status.LinearPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS;
  }
}
