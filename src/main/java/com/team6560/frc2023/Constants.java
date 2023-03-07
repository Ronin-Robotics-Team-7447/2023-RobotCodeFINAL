// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int GYRO_ID = 13;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47785;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47785;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(163.828 + 90.0 + 45.0 + 45.0 + 180.0 - 90.0);
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(291.09375);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(77.168 + 90.0 + 45.0 - 45.0 + 180.0);
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(245.830078125);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(316.758 + 45.0 + 45.0 + 90.0);
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(294.08203125);
  

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(302.959 + 90.0 + 45.0 - 45.0 + 90.0 - 90.0 - 180.0);
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(234.580078125);

  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      1/7.13 *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double MAX_ACCELERATION = 5.0; // m/s^2
  public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public static final int BREAK_ID = 25; // ARC motor
  public static final int CLAW_MOTOR_LEFT_ID = 26;
  public static final int CLAW_MOTOR_RIGHT_ID = 27;
  public static final int EXTENTION_SOLENOID_ID = 0;

  // public static final double ROTOR_TO_ARM = 38.1;

  public static final double BREAK_TO_ARM = 350;
  public static final double BREAK_MOTOR_MULTIPLIER = 1.0;

  public static final int INTAKE_EXTENSION_MOTOR_LEFT = 17;

  public static final int INTAKE_EXTENSION_MOTOR_RIGHT = 16;

  public static final int INTAKE_ROTATION_MOTOR = 15;

  public static final class TelescopeConstants {
    public static final int logitechID = 1;
    public static final int telescopeID = 16;
    public static final int throttleAxis = 3;
    public static final double telescopeSpeed = 0.3;
    public static final int upperLimit = 10;
    public static final int bottomLimit = 0;
  }

  public static final class ArmConstants {
    public static final int ArmEncoderID = 0;
    public static final int ArmID = 17;
    public static final int gripYAxis = 1;
    public static final double armSpeed = 0.3;
    public static final double upperLimit = 0.0;
    public static final double lowerLimit = 0.0;
  }

  public static final class WristConstants {
    public static final int WristID = 0;
    public static final int ClawID = 0;
    public static final int WristEncoderID = 0;
    public static final double wristSpeed = 0.1;
    public static final double clawSpeed = 0.3;
    public static final double upperLimit = 0.0;
    public static final double lowerLimit = 0.0;
  }

  public static class ArmJoystickConstants {
    public static final int buttonPOV0 = 0;
    public static final int buttonPOV180 = 180;

    public static final int button1 = 1;
    public static final int button2 = 2;
    public static final int button3 = 3;
    public static final int button4 = 4;
    public static final int button5 = 5;
    public static final int button6 = 6;
    public static final int button7 = 7;
    public static final int button8 = 8;
    public static final int button9 = 9;
    public static final int button10 = 10;
    public static final int button11 = 11;
    public static final int button12 = 12;
  }

  public static final class VisionConstants {
    public static final double LIMELIGHT_TO_ROBOT_X = -0.1966595;
    public static final int LIMELIGHT_TO_FLOOR_DISTANCE_METERS = 0;
  }

  public static final class ControllerIds {
    public static final int FIRST_DRIVER_CONTROLLER = 0;
    public static final int SECOND_DRIVER_CONTROL_STATION = 1;
    public static final int SECOND_DRIVER_CONTROLLER = 2;

    public static final int XBOX_L_JOY_X = 0;
    public static final int XBOX_L_JOY_Y = 1;

    public static final int XBOX_R_JOY_X = 4;
    public static final int XBOX_R_JOY_Y = 5;

    public static final int XBOX_L_BUMPER = 5;
    public static final int XBOX_R_BUMPER = 6;

    public static final int XBOX_L_TRIGGER = 2;
    public static final int XBOX_R_TRIGGER = 3;

    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_A_BUTTON = 1;

    

    public static final int DRIVER_STATION_TOGGLE_1 = 2;
    public static final int DRIVER_STATION_TOGGLE_2 = 5;
    public static final int DRIVER_STATION_TOGGLE_3 = 6;
    public static final int DRIVER_STATION_TOGGLE_4 = 9;

    public static final int DRIVER_STATION_BUTTON_1 = 1;
    public static final int DRIVER_STATION_BUTTON_2 = 4;
    public static final int DRIVER_STATION_BUTTON_3 = 3;

    public static final int DRIVER_STATION_X_AXIS = 0;
    public static final int DRIVER_STATION_Y_AXIS = 1;
  }

  // public static class VisionConstants {

  // // Cam mounted facing forward, half a meter forward of center, half a meter
  // up from center.
  // public static final Transform3d robotToCam = new Transform3d(
  // new Translation3d(0.5, 0.0, 0.28),
  // new Rotation3d(0, 0, 0));

  // }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);

    public static final double TOP_CONE_MARKER_TO_EDGE_Z_METERS = 0.98425;
    public static final double TOP_CONE_MARKER_TO_FLOOR_DISTANCE_METERS = 1.0795;
  }

}
