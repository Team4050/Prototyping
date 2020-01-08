/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Use this area to initialize different parts of the robot and certain variables.

  // motor init
  private Spark frontLeft;
  private Spark frontRight;
  private Spark rearLeft;
  private Spark rearRight;

  // joystick init
  private Joystick leftJoy;
  private Joystick rightJoy;
  private Joystick xbox;

  // speed controller group init
  private SpeedControllerGroup leftSC;
  private SpeedControllerGroup rightSC;

  // robot drive init
  private DifferentialDrive robotDrive;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Assign joysticks to different usb ports.
    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);
    xbox = new Joystick(2);

    // Assign motor controllers to pwm ports.
    frontLeft = new Spark(0);
    rearLeft = new Spark(1);
    frontRight = new Spark(2);
    rearRight = new Spark(3);

    // Invert left side motors
    frontLeft.set(-1);
    rearLeft.set(-1);

    // Assign motor controllers to speed groups
    leftSC = new SpeedControllerGroup(frontLeft, rearLeft);
    rightSC = new SpeedControllerGroup(frontRight, rearRight);

    //Assign speed groups to drive group
    robotDrive = new DifferentialDrive(leftSC, rightSC);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Joystick buttons
    double xrTrigger = xbox.getRawAxis(3);

    // Limelight code
        //Limelight variables
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
        //Send variables to dashboard
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("TargetValid", tv);

    if(xrTrigger >= 0.5){
      if(tv == 1){
        if(tx >= 9 && tx <= 27){
          robotDrive.tankDrive(0.5, -0.5);
        }
        else if(tx >= -27 && tx <= -9){
          robotDrive.tankDrive(-0.5, 0.5);
        }
        else if(tx >= -8 && tx <= 8){
          robotDrive.tankDrive(0, 0);
        }
      }
      else{
        robotDrive.tankDrive(0.5, -0.5);
      }
    }

    // Once again, sad and lonely tank drive code.
    robotDrive.tankDrive(leftJoy.getRawAxis(1), rightJoy.getRawAxis(1));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
