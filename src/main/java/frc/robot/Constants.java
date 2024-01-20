// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.led.LEDStrip;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.PurplePathPose;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

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
  public static class Field {
    public static final double FIELD_WIDTH = 8.21;
    public static final double FIELD_LENGTH = 16.54;

    public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(15.64, 5.55);

    public static final PurplePathPose AMP = new PurplePathPose(
      new Pose2d(Units.Meters.of(1.85), Units.Meters.of(7.77), Rotation2d.fromDegrees(+90.0)),
      new Pose2d(Units.Meters.of(14.66), Units.Meters.of(7.77), Rotation2d.fromDegrees(+90.0)),
      Units.Meters.of(0.7)
    );
    public static final PurplePathPose SOURCE = new PurplePathPose(
      new Pose2d(Units.Meters.of(15.48), Units.Meters.of(0.84), Rotation2d.fromDegrees(-60.00)),
      new Pose2d(Units.Meters.of(1.07), Units.Meters.of(0.82), Rotation2d.fromDegrees(-120.0)),
      Units.Meters.of(0.7)
    );
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.10;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(30.0, 0.0, 0.3, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.08;
    public static final double DRIVE_TURN_SCALAR = 30.0;
    public static final double DRIVE_LOOKAHEAD = 3;

    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.042, 0.168, 0.378, 0.672, 1.050, 1.512, 2.508, 2.688, 3.402, 4.300 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final MAXSwerveModule.GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 5);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 6);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 7);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 8);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 9);
    public static final LEDStrip.ID LED_STRIP_ID = new LEDStrip.ID("DriveHardware/LEDStrip", 0, 200);
  }

  public static class VisionHardware {
    public static final String CAMERA_A_NAME = "cameraA";
    public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
      new Translation3d(0.0, 0.0, 0.5),
      new Rotation3d(0.0, 0.0, 0.0)
    );
    public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_B_NAME = "cameraB";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
      new Translation3d(0.0, 0.0, 0.5),
      new Rotation3d(0.0, 0.0, Math.toRadians(+120.0))
    );
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_C_NAME = "cameraC";
    public static final Transform3d CAMERA_C_LOCATION = new Transform3d(
      new Translation3d(0.0, 0.0, 0.5),
      new Rotation3d(0.0, 0.0, Math.toRadians(-120.0))
    );
    public static final Resolution CAMERA_C_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_C_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_OBJECT_NAME = "cameraC";
    public static final Transform3d CAMERA_OBJECT_LOCATION = new Transform3d(
      new Translation3d(0.0, 0.0, 0.5),
      new Rotation3d(0.0, 0.0, Math.toRadians(-120.0))
    );
    public static final Resolution CAMERA_OBJECT_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_OBJECT_FOV = Rotation2d.fromDegrees(79.7);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }
}