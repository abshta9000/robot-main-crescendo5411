// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;

// -------------------------------------------------------------[Robot Constants]-----------------------------------------------------------//
/**
 * 
 * 
 * <h1>Constants</h1> 
 * 
 * <<p>Contains all robot-wide constants, does not contain subsystem specific constants.</p>>
 * 
 * @see RobotContainer
 */
public final class RobotConstants {
  // -----------------------------------------------------------[Internal Classes]---------------------------------------------------------//
  public static final class Subsystems {
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
  }

  public static final class Logging {
    public static final String LOGGING_DEPOSIT_FOLDER = ("src\\main\\java\\main\\deploy\\logs");
    public static final Logger LOGGING_INSTANCE = Logger.getInstance();
    public static final Boolean LOGGING_TURBO_MODE = (false);    
    public static final Boolean LOGGING_ENABLED = (false);
    public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (0);
    public static final Integer PATHPLANNER_SERVER = (6969);
  }
}