// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.drivebase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface  DrivebaseModuleIO {
  /** Creates a new DrivebaseIO. */
  @Autolog
  public static class DrivebaseIOInputs() {

    // idc what anshul said about using degrees for rotation telemetry
    // if both angular velocity and rotation use the same units it would make it much easier to understandd

    public double fwdVoltage = 69;
    public double fwdRotationRad = 69;
    public double fwdAngVelRadperSec = 69;
    public double fwdTempF = 69;

    public double fwdVoltage = 69;
    public double fwdRotationRad = 69;
    public double fwdAngVelRadperSec = 69;
    public double fwdTempF = 69;
  }
}
