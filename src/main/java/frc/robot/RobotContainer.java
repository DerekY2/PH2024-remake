// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.LeaveAuto;
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {
  
  Pose2d allianceWingTargetPose = new Pose2d(5.62, 6.72, Rotation2d.fromDegrees(0.0));
  Pose2d subwooferPose = new Pose2d(1.27, 5.57, Rotation2d.fromDegrees(0.0));

  // Load the paths we want to follow
  PathPlannerPath allianceWingToSubwoofer = PathPlannerPath.fromPathFile("WingToSubwoofer");
  PathPlannerPath SourceToSubwoofer = PathPlannerPath.fromPathFile("SourceToSubwoofer");
  PathPlannerPath toAllianceSource = PathPlannerPath.fromPathFile("ToSource");

  // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
  PathConstraints constraintsA = new PathConstraints(
  3.0, 2.0,
  2*Math.PI, 2*Math.PI);

  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
    DriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_ROTATE_PID,
    Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD
  );

  private static final VisionSubsystem VISION_SUBSYSTEM = VisionSubsystem.getInstance();

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(
      Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

  private final Field2d field;

  public RobotContainer() {

    field = new Field2d();
                SmartDashboard.putData("Field", field);

                // Logging callback for current robot pose
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.setRobotPose(pose);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.getObject("target pose").setPose(pose);
                });

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        field.getObject("path").setPoses(poses);
                });

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
        DRIVE_SUBSYSTEM.driveCommand(
            () -> PRIMARY_CONTROLLER.getLeftY(),
            () -> PRIMARY_CONTROLLER.getLeftX(),
            () -> PRIMARY_CONTROLLER.getRightX()));

    // Setup AutoBuilder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    VISION_SUBSYSTEM.setPoseSupplier(() -> DRIVE_SUBSYSTEM.getPose());

    // Bind buttons and triggers
    configureBindings();

    // Configure ShuffleBoard
    defaultShuffleboardTab();
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // // Y button - aim at speaker
    // PRIMARY_CONTROLLER.y().whileTrue(
    //   DRIVE_SUBSYSTEM.aimAtPointCommand(
    //     () -> PRIMARY_CONTROLLER.getLeftY(),
    //     () -> PRIMARY_CONTROLLER.getLeftX(),
    //     () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
    //         ? Constants.Field.BLUE_SPEAKER
    //         : Constants.Field.RED_SPEAKER,
    //     true
    //   )
    // );

    // Right bumper button - go to amp
    PRIMARY_CONTROLLER.rightBumper().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.AMP));

    // // A button - go to source
    // PRIMARY_CONTROLLER.a().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));

    // B button - aim at game object
    // PRIMARY_CONTROLLER.a().whileTrue(
    //   AutoBuilder.pathfindToPose(
    //           subwooferPose,
    //           constraintsA,
    //           0.0, // Goal end velocity in meters/sec
    //           0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    //   )
    // );

    PRIMARY_CONTROLLER.b().whileTrue(
      AutoBuilder.pathfindThenFollowPath(
        allianceWingToSubwoofer, constraintsA, 3.4 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    );

    PRIMARY_CONTROLLER.x().whileTrue(
      AutoBuilder.pathfindThenFollowPath(
        SourceToSubwoofer, constraintsA, 3.4 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    );

    PRIMARY_CONTROLLER.y().whileTrue(
      AutoBuilder.pathfindThenFollowPath(
        toAllianceSource, constraintsA, 3.4 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    );

    PRIMARY_CONTROLLER.a().whileTrue(
      new 
    );
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("Simple", new SimpleAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Leave", new LeaveAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("4 Note Auto 1", new PathPlannerAuto("4 Note Auto 1"));
  }

  /**
   * Run simlation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    Logger.recordOutput(DRIVE_SUBSYSTEM.getName() + "/notePose", VISION_SUBSYSTEM.getObjectTranslation());
  }

  /**
   * Get currently selected autonomous command
   *
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
  }
}
