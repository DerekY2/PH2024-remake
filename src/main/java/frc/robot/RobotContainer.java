// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.Leave;
import frc.robot.commands.autonomous.Simple;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
      DriveSubsystem.initializeHardware(),
      Constants.Drive.DRIVE_ROTATE_PID,
      Constants.Drive.DRIVE_CONTROL_CENTRICITY,
      Constants.Drive.DRIVE_TURN_SCALAR,
      Constants.HID.CONTROLLER_DEADBAND,
      Constants.Drive.DRIVE_LOOKAHEAD,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE);

  private static final VisionSubsystem VISION_SUBSYSTEM = VisionSubsystem.getInstance();

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(
      Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static SendableChooser<SequentialCommandGroup> m_automodeChooser = new SendableChooser<>();

  public RobotContainer() {
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
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());
    PRIMARY_CONTROLLER.y().whileTrue(
        DRIVE_SUBSYSTEM.aimAtPointCommand(
            () -> PRIMARY_CONTROLLER.getLeftY(),
            () -> PRIMARY_CONTROLLER.getLeftX(),
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.Field.BLUE_SPEAKER
                : Constants.Field.RED_SPEAKER,
            false));

    PRIMARY_CONTROLLER.rightBumper().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.AMP));
    PRIMARY_CONTROLLER.a().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));
    PRIMARY_CONTROLLER.x().onTrue(DRIVE_SUBSYSTEM.runOnce(() -> DRIVE_SUBSYSTEM.resetPose(new Pose2d())));

    PRIMARY_CONTROLLER.b().whileTrue(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> VISION_SUBSYSTEM.getObjectTranslation() == null ? null : VISION_SUBSYSTEM.getObjectTranslation(),
        false)
      );
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("Mobility", new Leave(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Simple", new Simple(DRIVE_SUBSYSTEM));
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
