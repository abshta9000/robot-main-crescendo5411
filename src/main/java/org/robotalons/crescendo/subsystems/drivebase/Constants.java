// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.robotalons.crescendo.subsystems.drivebase.REVSimModule.ModuleConstants;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.sensors.Gyroscope;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see DrivebaseSubsystem
 */
public final class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
    public static final class Measurements {
        public static final Double ROBOT_WHEEL_DIAMETER_METERS = Units.inchesToMeters((4));
        public static final Double ROBOT_WHEEL_PERIMETER_METERS = ROBOT_WHEEL_DIAMETER_METERS * Math.PI;
        public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29));        
        public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29));
        public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0));      
        public static final Double ROBOT_MASS_KG = (69d);

        public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((15.4));
        public static final Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

        public static final Double ROBOT_TRANSLATION_KP = (0d);
        public static final Double ROBOT_TRANSLATION_KI = (0d);
        public static final Double ROBOT_TRANSLATION_KD = (0d);    

        public static final Double ROBOT_ROTATIONAL_KP = (0d);
        public static final Double ROBOT_ROTATIONAL_KI = (0d);
        public static final Double ROBOT_ROTATIONAL_KD = (0d);

        public static final Boolean PHOENIX_DRIVE = (false);

        public static final Double ODOMETRY_FREQUENCY = (250d);

        public static final Double LINEAR_GEAR_RATIO = (10d);
        public static final Double ROTATION_GEAR_RATIO = (10d);

        public static final Double ROBOT_MAXIMUM_ANGULAR_MOMENTUM = 
          Math.pow(ROBOT_WHEEL_DIAMETER_METERS/2,2) * // radius
          ROBOT_MAXIMUM_ANGULAR_VELOCITY * // velocity
          ROBOT_MASS_KG; // mass

        public static final class Modules {
          public static final class FL {
            public static final Integer LINEAR_CONTROLLER_ID = (0);
            public static final Integer ROTATIONAL_CONTROLLER_ID = (1);
            public static final Integer ABSOLUTE_ENCODER_ID = (8);
            public static final Double ROTATIONAL_P_GAIN = (.075d);
            public static final Double ROTATIONAL_I_GAIN = (0d);
            public static final Double ROTATIONAL_D_GAIN = (0d);
            public static final Double LINEAR_P_GAIN = (.001d);
            public static final Double LINEAR_I_GAIN = (0d);
            public static final Double LINEAR_D_GAIN = (0d);
            public static final Double LINEAR_KS_GAIN = (0d);
            public static final Double LINEAR_KV_GAIN = (0d);
            public static final Double LINEAR_KA_GAIN = (0d);
            public static final ModuleConstants CONSTANTS = new ModuleConstants();
            static {
              CONSTANTS.LINEAR_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.LINEAR_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              CONSTANTS.ROTATIONAL_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.ROTATION_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              
              CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
              CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
              CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
              CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
              CONSTANTS.NUMBER = 0;
            }
          }
  
          public static final class FR {
            public static final Integer LINEAR_CONTROLLER_ID = (2);
            public static final Integer ROTATIONAL_CONTROLLER_ID = (3);
            public static final Integer ABSOLUTE_ENCODER_ID = (9);
            public static final Double ROTATIONAL_P_GAIN = (.075d);
            public static final Double ROTATIONAL_I_GAIN = (0d);
            public static final Double ROTATIONAL_D_GAIN = (0d);
            public static final Double LINEAR_P_GAIN = (.001d);
            public static final Double LINEAR_I_GAIN = (0d);
            public static final Double LINEAR_D_GAIN = (0d);
            public static final Double LINEAR_KS_GAIN = (0d);
            public static final Double LINEAR_KV_GAIN = (0d);
            public static final Double LINEAR_KA_GAIN = (0d);
            public static final ModuleConstants CONSTANTS = new ModuleConstants();
            static {
              CONSTANTS.LINEAR_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.LINEAR_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              CONSTANTS.ROTATIONAL_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.ROTATION_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              
              CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
              CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
              CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
              CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
              CONSTANTS.NUMBER = 1;
            }
          }
  
          public static final class RL {
            public static final Integer LINEAR_CONTROLLER_ID = (4);
            public static final Integer ROTATIONAL_CONTROLLER_ID = (5);
            public static final Integer ABSOLUTE_ENCODER_ID = (10);
            public static final Double ROTATIONAL_P_GAIN = (.075d);
            public static final Double ROTATIONAL_I_GAIN = (0d);
            public static final Double ROTATIONAL_D_GAIN = (0d);
            public static final Double LINEAR_P_GAIN = (.001d);
            public static final Double LINEAR_I_GAIN = (0d);
            public static final Double LINEAR_D_GAIN = (0d);
            public static final Double LINEAR_KS_GAIN = (0d);
            public static final Double LINEAR_KV_GAIN = (0d);
            public static final Double LINEAR_KA_GAIN = (0d);
            public static final ModuleConstants CONSTANTS = new ModuleConstants();
            static {
              CONSTANTS.LINEAR_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.LINEAR_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              CONSTANTS.ROTATIONAL_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.ROTATION_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              
              CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
              CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
              CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
              CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
              CONSTANTS.NUMBER = 2;
            }
          }
  
          public static final class RR {
            public static final Integer LINEAR_CONTROLLER_ID = (6);
            public static final Integer ROTATIONAL_CONTROLLER_ID = (7);
            public static final Integer ABSOLUTE_ENCODER_ID = (11);
            public static final Double ROTATIONAL_P_GAIN = (.075d);
            public static final Double ROTATIONAL_I_GAIN = (0d);
            public static final Double ROTATIONAL_D_GAIN = (0d);
            public static final Double LINEAR_P_GAIN = (.001d);
            public static final Double LINEAR_I_GAIN = (0d);
            public static final Double LINEAR_D_GAIN = (0d);
            public static final Double LINEAR_KS_GAIN = (0d);
            public static final Double LINEAR_KV_GAIN = (0d);
            public static final Double LINEAR_KA_GAIN = (0d);
            public static final ModuleConstants CONSTANTS = new ModuleConstants();
            static {
              CONSTANTS.LINEAR_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.LINEAR_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              CONSTANTS.ROTATIONAL_CONTROLLER = new FlywheelSim(
                DCMotor.getNEO(1), // motor type
                Measurements.ROTATION_GEAR_RATIO, // gearing
                Measurements.ROBOT_MAXIMUM_ANGULAR_MOMENTUM / Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY // moment of inertia
              );
              
              CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
              CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
              CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
              CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
              CONSTANTS.NUMBER = 3;
            }
          }
  
        }
    }

    public static final class Ports {
      public static final Integer GYROSCOPE_ID = (0);      
    }

    public static final class Objects {
        public static final Lock ODOMETRY_LOCK = new ReentrantLock();
    }

    public static final class Devices {
      public static final Gyroscope GYROSCOPE = 
        new GyroSim(false,Measurements.Modules.FL.CONSTANTS.ROTATIONAL_CONTROLLER);
      public static final Module FRONT_LEFT_MODULE = 
        new REVSimModule(Measurements.Modules.FL.CONSTANTS);     
      public static final Module FRONT_RIGHT_MODULE = 
        new REVSimModule(Measurements.Modules.FR.CONSTANTS);             
      public static final Module REAR_LEFT_MODULE = 
        new REVSimModule(Measurements.Modules.RL.CONSTANTS);        
      public static final Module REAR_RIGHT_MODULE = 
        new REVSimModule(Measurements.Modules.RR.CONSTANTS);    
    }
  }
