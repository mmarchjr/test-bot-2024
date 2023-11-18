// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import frc.utils.LocalADStarAK;
import frc.utils.SwerveUtils;

public class SUBDrive extends SubsystemBase {
  // Create MAXSwerveModules
  private final SUBMAXSwerveModule m_frontLeft = new SUBMAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);
  /**
   * The front left MAXSwerveModule.
   */
  private final SUBMAXSwerveModule m_frontRight = new SUBMAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);
  /**
   * The front right MAXSwerveModule.
   */
  private final SUBMAXSwerveModule m_rearLeft = new SUBMAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);
  /**
   * The rear left MAXSwerveModule.
   */
  private final SUBMAXSwerveModule m_rearRight = new SUBMAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);
  /**
   * The rear right MAXSwerveModule.
   */

  // The gyro sensor
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  /**
   * The gyro sensor used for navigation.
   */
  private Field2d m_field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation;
  /**
   * The current rotation value.
   */
  private double m_currentTranslationDir = 0.0;
  /**
   * The current translation direction.
   */
  private double m_currentTranslationMag = 0.0;
  /**
   * The current translation magnitude.
   */
  private double gyroOffset = 0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(m_gyro.getAngle()),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });
  /**
   * The odometry class used for tracking the robot's pose.
   */

  /** Creates a new DriveSubsystem. */
  public SUBDrive() {

   /* AutoBuilder.configureHolonomic(
        this::getPosecorrected, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.64, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.85, 0.0, 0.0), // Rotation PID constants
            3, // Max module speed, in m/s
            Units.inchesToMeters(
                18.2), // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        this // Reference to this subsystem to set requirements
        );*/
         //Pathfinding.setPathfinder(new LocalADStarAK());
    /*PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });*/
  }
/**
 * Updates the robot's position and angle in the periodic block.
 */
@Override
public void periodic() {
    // Update the SmartDashboard with the angles of each wheel module
    SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("RL Angle", m_rearLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("RR Angle", m_rearRight.getState().angle.getDegrees());

    // Create an array of SwerveModulePosition objects to store the positions of each wheel module
    SwerveModulePosition[] positions = new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
    };

    // Update the odometry with the current gyro angle and wheel module positions
    m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            positions
    );

    // Update the robot's pose with the current gyro angle offset and wheel module positions
    Pose2d pose = m_odometry.update(
            Rotation2d.fromDegrees(gyroOffset),
            positions
    );

    // Update the field's robot pose with the calculated pose
    m_field.setRobotPose(pose);

    // Update the SmartDashboard with the field object
    SmartDashboard.putData("Field", m_field);
}

  public void resetGyro() {
    m_gyro.reset();
  }

/**
 * Returns the currently estimated pose of the robot.
 *
 * @return The pose.
 */
public Pose2d getPose() {
  // Return the pose in meters
  return m_odometry.getPoseMeters();
}

  public Pose2d getPosecorrected() {
    return m_odometry.getPoseMeters().rotateBy(Rotation2d.fromDegrees(gyroOffset));
  }

/**
 * Returns an array of SwerveModuleState objects representing the states of all four swerve modules.
 *
 * @return an array of SwerveModuleState objects
 */
public SwerveModuleState[] getStates() {
  SwerveModuleState[] moduleStates = new SwerveModuleState[] {
    m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()
  };

  return moduleStates;
}

  public SwerveModulePosition[] getPosition() {
    SwerveModulePosition[] modulepositions =
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        };

    return modulepositions;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }
public void resetPosition(Pose2d pose) {
     m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },pose);
  }
/**
 * Drives the robot with the given speeds and rotation.
 *
 * @param xSpeed         The desired speed in the x direction.
 * @param ySpeed         The desired speed in the y direction.
 * @param rot            The desired rotation.
 * @param fieldRelative  Whether the speeds are field-relative.
 * @param rateLimit      Whether to apply rate limiting to the speeds.
 */
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
  // Calculate the commanded speeds in the x and y directions.
  double xSpeedCommanded;
  double ySpeedCommanded;
  
  if (rateLimit) {
    // Calculate the direction and magnitude of the input translation.
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
    
    // Calculate the direction slew rate based on the current translation magnitude.
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;
    }
    
    // Calculate the elapsed time since the previous drive command.
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    
    // Calculate the difference in angle between the input translation direction and the current translation direction.
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
    
    if (angleDif < 0.45 * Math.PI) {
      // Step towards the input translation direction and limit the magnitude.
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * Math.PI) {
      if (m_currentTranslationMag > 1e-4) {
        // If the current translation magnitude is greater than 1e-4, limit it to 0.0.
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      } else {
        // If the current translation magnitude is less than or equal to 1e-4, wrap the current translation direction by PI and limit the magnitude.
        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
    } else {
      // Step towards the input translation direction and limit the magnitude to 0.0.
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(0.0);
    }
    m_prevTime = currentTime;

    // Calculate the commanded speeds in the x and y directions based on the current translation direction and magnitude.
    xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.calculate(rot);
  } else {
    // Use the input speeds and rotation directly.
    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = rot;
  }

  // Calculate the delivered speeds in the x and y directions and the rotation.
  double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
  double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
}

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void set0() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public ChassisSpeeds getspeed() {
    ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());

    return chassisSpeeds;
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /*boolean isFirstPath;

   public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
     return new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                resetOdometry(traj.getInitialHolonomicPose());
            }
          }),
          new PPSwerveControllerCommand(
              traj,
              this::getPose, // Pose supplier
              Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
              new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
              new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              this::setModuleStates, // Module states consumer
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
              this // Requires this drive subsystem
          )
      );
  }*/

  // Assuming this is a method in your drive subsystem
  // Assuming this is a method in your drive subsystem
  /*public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
    return new FollowPathWithEvents(
        new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this // Reference to this subsystem to set requirements
        ),
        path, // FollowPathWithEvents also requires the path
        this::getPose // FollowPathWithEvents also requires the robot pose supplier
    );
  }*/

}
