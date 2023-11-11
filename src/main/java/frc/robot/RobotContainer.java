// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.driverobot;
import frc.robot.subsystems.DriveSubsystem;
import java.io.File;
import java.nio.file.Path;

//import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * 
 * 
 * ## CHECK FOR LOOSE PARTS BEFORE RUNNING ON GROUND
 */
public class RobotContainer {
  // The robot's subsystems

  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final driverobot c_driverobot = new driverobot();
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public static SendableChooser<Boolean> fieldoriented = new SendableChooser<Boolean>();
  public static SendableChooser<Boolean> ratelimitChooser = new SendableChooser<Boolean>();
  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

     AutoBuilder.configureHolonomic(
        m_robotDrive::getPosecorrected, // Robot pose supplier
        m_robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        m_robotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_robotDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(12, 0.0, 0.0), // Rotation PID constants
            3, // Max module speed, in m/s
            Units.inchesToMeters(
                18.2), // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        m_robotDrive // Reference to this subsystem to set requirements
        );
    
    // Configure the button bindings
    fieldoriented.setDefaultOption("field oriented", true);
    fieldoriented.addOption("robot oriented", false);

    ratelimitChooser.setDefaultOption("false", false);
    ratelimitChooser.addOption("true", true);

    configureButtonBindings();
    /*File deploy = Filesystem.getDeployDirectory();
    File pathfolder =
        new File(Path.of(deploy.getAbsolutePath(), "pathplanner", "autos").toString());
    File[] listOfFiles = pathfolder.listFiles();

    for (int i = 0; i < listOfFiles.length; i++) {
      if (listOfFiles[i].isFile()) {
        System.out.println("path:" + listOfFiles[i].getName());
        autoChooser.addOption(
            listOfFiles[i].getName().replace(".auto", ""),
            listOfFiles[i].getName().replace(".auto", ""));
      }
    }*/
    // String pathplannerlocation = ;
SmartDashboard.putData("field oriented", fieldoriented);
SmartDashboard.putData("rate limit", ratelimitChooser);
//SmartDashboard.putData("auto", autoChooser);
autoChooser = AutoBuilder.buildAutoChooser();

// Another option that allows you to specify the default auto by its name
// autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(c_driverobot);
    m_robotDrive.resetOdometry(PathPlannerPath.fromPathFile("2 meter-straight").getStartingDifferentialPose());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 3)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    /*new JoystickButton(m_driverController, 4)
    .whileTrue(new RunCommand(
        () -> m_robotDrive.set0(),
        m_robotDrive));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory

    // An example trajectory to follow.  All units in meters.
    /*PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath(autoChooser.getSelected(),  new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
           exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
            new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.setX());*/
    /*PathPlannerPath path = PathPlannerPath.fromPathFile("test-path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.buildAuto("test-path");*/
    //return new PathPlannerAuto("2 meter straight");
return autoChooser.getSelected();
       
  }
}
