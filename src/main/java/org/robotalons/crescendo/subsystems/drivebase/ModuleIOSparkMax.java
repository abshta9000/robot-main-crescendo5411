// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import org.robotalons.crescendo.Constants;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseConstants.Measurements;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.DoubleSupplier;

public class ModuleIOSparkMax implements ModuleIO {
  /** Creates a new DrivebaseModuleCANSparkMax. */
 
  private CANSparkMax motForward;
  private CANSparkMax motAzimuth;

  private RelativeEncoder encFrontLeft; 

  private AHRS gyro;
 
  // ds for faster response times (i think)
  private DoubleSupplier fwdAppliedVolts;
  private DoubleSupplier aziAppliedVolts;

  private double aziRelativePositionRad = 0.0;
  private double aziAbsolutePositionRad = Math.PI;

  public ModuleIOSparkMax() {
    fwdFrontRight = new CANSparkMax(DrivebaseConstants.Ports.FWDMOTOR_FR_ID, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    forwardSim.update(Constants.Simulation.SIMULATION_LOOPPERIOD_SEC);
    azimuthSim.update(Constants.Simulation.SIMULATION_LOOPPERIOD_SEC);


    // code block modified from mechanical advantage
      double angleDiffRad = azimuthSim.getAngularVelocityRadPerSec() * Constants.Simulation.SIMULATION_LOOPPERIOD_SEC;
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

    inputs.aziAbsRotationRad = aziAbsolutePositionRad;
    inputs.aziRotationRad = aziRelativePositionRad;
    inputs.aziAngVelRadperSec = azimuthSim.getAngularVelocityRadPerSec();
    inputs.aziVoltage = aziAppliedVolts.getAsDouble();


    inputs.fwdRotationRad += (forwardSim.getAngularVelocityRadPerSec() * Constants.Simulation.SIMULATION_LOOPPERIOD_SEC);
    inputs.fwdAngVelRadperSec = forwardSim.getAngularVelocityRadPerSec();
    inputs.fwdVoltage = fwdAppliedVolts.getAsDouble();
  
  }

  public void setVoltage(double forward, double azimuth){
    fwdAppliedVolts = () -> MathUtil.clamp(forward,-12,12);
    aziAppliedVolts = () -> MathUtil.clamp(azimuth,-12,12);

    forwardSim.setInputVoltage(fwdAppliedVolts.getAsDouble());
    azimuthSim.setInputVoltage(aziAppliedVolts.getAsDouble());
  }
}
