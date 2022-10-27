// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Drivetrain {
        //Assignation des moteurs a leur port
        public static final int kMoteurAvantDroitPort = 2; // reseau CAN
        public static final int kMoteurAvantGauchePort = 4; // reseau CAN
        public static final int DEFAULT_SETTING_TIMEOUT_MS = 50;
        public static final double kMaxSpeed = 1.0; // 3.0
        public static final double kMaxAngularSpeed = 1.0;
        public static final double kS = 0.62; // kS
        public static final double kV = 2.42; // kV
        public static final double kA = 0.165; // kA
        public static final double kVAngular = 1.654294727 * 2;
        public static final double kAAngular = kA; // ???????
        public static final double TRACK_WIDTH_METERS =  0.381 * 2;  //0.5757943419;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);
        public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(kS, kV, kA), KINEMATICS, 10);
        public static final LinearSystem<N2, N2, N2> PLANT = LinearSystemId.identifyDrivetrainSystem(kV, kA, kVAngular,
                kAAngular);
        public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13); // 13 ft/s = 3.9624
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2; // 2m/s/s = about 2 seconds to full
                                                                                   // speed
        // RAMSETE constants
        public static final double kRamseteB = 2; // default, should be good
        public static final double kRamseteZeta = 0.7; // default, should be good
        // Encoders (on back)
        // public static final boolean LEFT_BACK_SENSOR_PHASE = true;
        // public static final boolean RIGHT_BACK_SENSOR_PHASE = true;
        // Wheel
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0d; //0.0508;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVE_GEARING = 10.71; // if ratio from motor to wheel is X:1, enter X, todo: ask Ed
                                                          // for real value
        public static final int ENCODER_TICKS_PER_ROTATION = (int) Math.round(2048 * DRIVE_GEARING); // 2048 for
                                                                                                     // TalonFX, 4096
                                                                                                     // for TalonSRX
        public static final int ENCODER_RESOLUTION = -4096; //for the simulated robot only
        // Drive velocity PID (TalonFX)
        public static final int VELOCITY_PID_IDX = 0;
        public static final double VELOCITY_P = 0.000953; // With 10.71 gearing in analysis (In Google Docs Notes as
                                                          // #A1)
        public static final double VELOCITY_I = 0; //
        public static final double VELOCITY_D = 0; //
        public static final int DRIVE_VELOCITY_ERROR_TOLERANCE = (int) (.1d * ENCODER_TICKS_PER_ROTATION); // .1
                                                                                                           // rotation
                                                                                                           // tolerance
        // Drive velocity sampling settings
        // public static final int ROLLING_VELOCITY_SAMPLES = 4; // 1,2,4,8,16,32
        // public static final VelocityMeasPeriod VELOCITY_MEAS_PERIOD =
        // VelocityMeasPeriod.Period_5Ms;
        public static final int STATUS_2_FEEDBACK_MS = 20; // 20ms default
        public static final int STATUS_3_QUADRATURE_MS = 40; // 160ms default
        public static final int MAX_BATTERY_V = 12;
        public static final boolean HAS_ENCODERS = true;
        //Assignation de NavX
        public static final double kNavXOffset = 0.0;
    }

    public static class RobotDimensions {
        public static final double LENGTH = 0.82042;
        public static final double WIDTH = 0.69596;
    }

    public static class Manette {
        public static final int kCoPilot = 0; // port USB
        public static final int kPilotPort = 1; // port USB
    }
}