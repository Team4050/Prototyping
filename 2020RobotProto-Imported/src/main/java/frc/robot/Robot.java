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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Use this area to initialize different parts of the robot and certain variables.

  //Ultasonic Sensor
  private static final double kHoldDistance = 12.0;
  private static final double kValueToInches = 0.125;
  private static final double kp = 0.05;
  private static final int kUltrasonicPort = 1;
  private final AnalogInput sonic = new AnalogInput(kUltrasonicPort);

  //Limit Switches
  DigitalInput input = new DigitalInput(0);

  //I2C
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  //Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // motor init
  private Spark frontLeft;
  private Spark frontRight;
  private Spark rearLeft;
  private Spark rearRight;
  private double kP;
  private double kI;
  private double kD;
  TalonSRX _tal = new TalonSRX(2);

  //limelight
  PIDController lime = new PIDController(kP, kI, kD);

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
  //pid
  int integral, previous_error, setpoint = 0;
  private double rcw;

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
    frontRight.set(1);
    rearRight.set(1);
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
    //Color Sensor
    final Color detectedColor = m_colorSensor.getColor();
    final double IR = m_colorSensor.getIR();

    final double velocity = _tal.getSelectedSensorVelocity(0);    
    SmartDashboard.putNumber("vel", velocity);
    final double position = _tal.getSelectedSensorPosition(0);    
    SmartDashboard.putNumber("position", position);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    final int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    final boolean limitpressed = input.get();
    SmartDashboard.putBoolean("Testlimit", limitpressed);

    // Ultrasonic
    final double currentDistance = sonic.getValue() * 0.0508474576;
    SmartDashboard.putNumber("Distance", currentDistance);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
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
    if (_tal.getSelectedSensorPosition() < 16384 && _tal.getSelectedSensorPosition() > 0){
      _tal.set(ControlMode.PercentOutput, 0);
    }
    else if (_tal.getSelectedSensorPosition() > 16384){
      _tal.set(ControlMode.PercentOutput, -0.30);
    }
    else if (_tal.getSelectedSensorPosition() < 0){
      _tal.set(ControlMode.PercentOutput, 0.30);
    }
    Timer.delay(1);
    if (_tal.getSelectedSensorPosition() < -2048 && _tal.getSelectedSensorPosition() > -16384){
      _tal.set(ControlMode.PercentOutput, 0);
    }
    else if (_tal.getSelectedSensorPosition() > -2048){
      _tal.set(ControlMode.PercentOutput, -0.30);
    }
    else if (_tal.getSelectedSensorPosition() < -16384){
      _tal.set(ControlMode.PercentOutput, 0.30);
    }
    Timer.delay(1);
    if (_tal.getSelectedSensorPosition() < 16384 && _tal.getSelectedSensorPosition() > 0){
      _tal.set(ControlMode.PercentOutput, 0);
    }
    else if (_tal.getSelectedSensorPosition() > 16384){
      _tal.set(ControlMode.PercentOutput, -0.30);
    }
    else if (_tal.getSelectedSensorPosition() < 0){
      _tal.set(ControlMode.PercentOutput, 0.30);
    }
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopInit() {
    /* Factory default hardware to prevent unexpected behavior */

    _tal.configFactoryDefault();

    /* Victor will follow Talon */
    /*
     * New frame every 1ms, since this is a test project use up as much bandwidth as
     * possible for the purpose of this test.
     */

    _tal.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    _tal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    /* Fire the plotter */
  }

  @Override
  public void teleopPeriodic() {

    // Encoder
    if (xbox.getRawButton(1)) {
      if (_tal.getSelectedSensorPosition() < 16384 && _tal.getSelectedSensorPosition() > 0){
        _tal.set(ControlMode.PercentOutput, 0);
      }
      else if (_tal.getSelectedSensorPosition() > 16384){
        _tal.set(ControlMode.PercentOutput, -0.30);
      }
      else if (_tal.getSelectedSensorPosition() < 0){
        _tal.set(ControlMode.PercentOutput, 0.30);
      }
    }
		else 
      _tal.set(ControlMode.PercentOutput, 0.0);

    // PID
    kP = 0.04;
    kI = 0.07;
    kD = 0;

    // Limelight
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);
    final double v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightValid", v);

    // pid
    //final double lspeed = MathUtil.clamp(rcw, -0.4, 0.4);
    //this.x = x;
    //this.setpoint = setpoint;
    //final double error = setpoint - x;
    //this.integral += (error * .02);
    //final double derivative = (error - this.previous_error) / .02;
    //this.rcw = kP*error + kI*this.integral + kD*derivative;

    // Joystick buttons
    final double xrTrigger = xbox.getRawAxis(3);

    // Limelight code
    // Limelight variables
    if (xrTrigger >= 0.5) {
      if (v == 1){
      robotDrive.arcadeDrive(0, rcw * -1);
      }
      else{
        robotDrive.arcadeDrive(0, 0.7);
      }
    }
    else{
      // Once again, sad and lonely tank drive code.
      robotDrive.tankDrive(leftJoy.getRawAxis(1) * -1, rightJoy.getRawAxis(1) * -1);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
