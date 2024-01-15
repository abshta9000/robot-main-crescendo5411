// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.lib.motion;

import org.littletonrobotics.junction.AutoLog;

public interface  ModuleIO{
  /** Creates a new DrivebaseIO. */
  @AutoLog
  public static class ModuleIOInputs{

    // idc what anshul said about using degrees for rotation telemetry
    // if both angular velocity and rotation use the same units it would make it much easier to understandd

    public double fwdVoltage = 69;
    public double fwdRotationRad = 69;
    public double fwdAngVelRadperSec = 69;
    public double fwdTempF = 69;

    public double aziVoltage = 69;
    public double aziAbsRotationRad = 69;
    public double aziRotationRad = 69;
    public double aziAngVelRadperSec = 69;
    public double aziTempF = -1;
  }

  public default void updateInputs(ModuleIOInputs inputs){} 

  public default void setVoltage(double forward, double azimuth){}
}
