// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.utils.RoaringUtils;

public class driveRobot extends Command {
  /** Creates a new driveRobot. */
  double angle = 0;

  double deadzone = 0.3; // variable for amount of deadzone
  double y = 0; // variable for forward/backward movement
  double x = 0; // variable for side to side movement
  double turn = 0; // variable for turning movement
  PIDController turnController;
  double pastTurn = 0;
  
  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system.  Note that the      */
  /* SmartDashboard in Test mode has support for helping you tune    */
  /* controllers by displaying a form where you can enter new P, I,  */
  /* and D constants and test the mechanism.                         */
  
  static final double kP = 0.02;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public driveRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f,  180.0f);
    turnController.setTolerance(kToleranceDegrees);
    angle = RobotContainer.m_robotDrive.getHeading();
    turnController.setSetpoint(RobotContainer.m_robotDrive.getHeading());
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    x = 0;
    y = 0;
    turn = 0;
SmartDashboard.putNumber("gyro", RobotContainer.m_robotDrive.getHeading());


    y=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getRightY(),0.1);
    x=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getRightX(),0.1);
 turn=RoaringUtils.DeadzoneUtils.LinearDeadband(m_driverController.getLeftX(),0.1);
    

    
    
          boolean rotateToAngle = false;
           if (turn != pastTurn) {
              turnController.setSetpoint(RobotContainer.m_robotDrive.getHeading());
          }
          if (m_driverController.getBButton()) {
          turnController.setSetpoint(RobotContainer.m_robotDrive.getHeading());
        }
          if (m_driverController.getPOV()==0) {
              turnController.setSetpoint(0.0f);
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==90) {
              turnController.setSetpoint(-90.0f);//inverted
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==180) {
              turnController.setSetpoint(179.9f);
              rotateToAngle = true;
          } else if (m_driverController.getPOV()==270) {
              turnController.setSetpoint(90.0f);//inverted
              rotateToAngle = true;
          }
          double currentRotationRate;
          if ( rotateToAngle ) {
              currentRotationRate = MathUtil.clamp(turnController.calculate(RobotContainer.m_robotDrive.getHeading()),-1,1); // rotation(divided by 4)

          } else {
              currentRotationRate = -turn;
            } 
          
          if (turn == 0) {
                currentRotationRate = MathUtil.clamp(turnController.calculate(RobotContainer.m_robotDrive.getHeading()),-1,1);
              }
            
          if (y==0 && x==0 && turn == 0 && m_driverController.getPOV() == -1){
            RobotContainer.m_robotDrive.setX();
          } else {
            RobotContainer.m_robotDrive.drive(
          -y / 2, // forwards(divided by 4)
          -x / 2, // sideways(divided by 4)
          currentRotationRate/2,
          RobotContainer.fieldoriented.getSelected(), // field oriented
          RobotContainer.ratelimitChooser.getSelected() // limit max speed
          );
              /* Use the joystick X axis for lateral movement,          */
              /* Y axis for forward movement, and the current           */
              /* calculated rotation rate (or joystick Z axis),         */
              /* depending upon whether "rotate to angle" is active.    */
          }
          pastTurn = turn;
    //^ if (x==0 && y==0 && turn==0){RobotContainer.m_robotDrive.setX()};



    if (m_driverController.getYButton()) {
      RobotContainer.m_robotDrive.resetGyro();
      m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
