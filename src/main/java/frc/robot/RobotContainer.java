package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CMDDriveRobot;
import frc.robot.subsystems.SUBDrive;

public class RobotContainer {

  public static final SUBDrive robotDrive = new SUBDrive();
  public static final CMDDriveRobot driveRobotCommand = new CMDDriveRobot();
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
  public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();
  public CommandXboxController OIdriver1Controller = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    AutoBuilder.configureHolonomic(
        robotDrive::getPose,
        robotDrive::resetOdometry,
        robotDrive::getspeed,
        robotDrive::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.5, 0.0, 0.0),
            new PIDConstants(12, 0.0, 0.0),
            3,
            Units.inchesToMeters(18.2),
            new ReplanningConfig()
        ),
        robotDrive
    );

    fieldOrientedChooser.setDefaultOption("Field Oriented", true);
    fieldOrientedChooser.addOption("Robot Oriented", false);

    rateLimitChooser.setDefaultOption("False", false);
    rateLimitChooser.addOption("True", true);

    SmartDashboard.putData(rateLimitChooser);
    SmartDashboard.putData(fieldOrientedChooser);

    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    robotDrive.setDefaultCommand(driveRobotCommand);
    robotDrive.resetOdometry(PathPlannerPath.fromPathFile("2 meter-straight").getStartingDifferentialPose());
  }

  private void configureButtonBindings() {
    OIdriver1Controller.x().whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}