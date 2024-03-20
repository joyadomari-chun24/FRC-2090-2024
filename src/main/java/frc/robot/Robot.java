// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Color Sensor Code 
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private SendableChooser<Command> auto;

  //Apriltag variables
  private double tagCenterX;
  private double tagCenterY;
  private int tagId;

  // Color Sensor Code
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  //Calibrating sensor and defining colors
  private final Color kOrangeTarget = new Color(0.573, 0.354, 0.078);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture(0); //Starts capture for camera 0 (this one is purely video, no processing)
    var visionThread = new Thread(this::apriltagVisionThreadProcessor); //Creates thread
    visionThread.setDaemon(true); //Specifies that this thread is a daemon
    visionThread.start(); //Starts the thread

    auto = new SendableChooser<>();

    auto.setDefaultOption("Test Rotate", m_robotContainer.getAutoRotateCommand());
    auto.addOption("Test Drive & Rotate", m_robotContainer.getAutoDriveCommand());
    auto.addOption("New Auto", m_robotContainer.getAutoTestCommand());
    auto.addOption("autoC", m_robotContainer.autoCommandC());

    SmartDashboard.putData("Auto Mode", auto);

    m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  /**)
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Color Sensor Code

    // double IR = m_colorSensor.getIR();
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);

     // if (match.color == kOrangeTarget) {
    //   colorString = "Orange";
    // } else {
    //   colorString = "Not Orange";
    // }

    // boolean ringProximity = false; 


    // if (proximity>250){
    //   ringProximity = true;
     // } else {
    //   colorString = "Unknown";
    //   ringProximity = false;
    // }

    // SmartDashboard.putBoolean("Ring Proximity", ringProximity);
    // SmartDashboard.putString("Orange", colorString);

    Color detectedColor = m_colorSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    boolean ringPresent = false;

    if (match.color == kOrangeTarget && proximity>200)
      ringPresent = true;
    else 
      ringPresent = false;

    SmartDashboard.putBoolean("Ring Present", ringPresent);
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = auto.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Shooter shooter = new Shooter();
    shooter.runShooter();
  }

  public double isTagInRange()
  {
    //Check if the tag is a speaker tag, then if it is an amp tag, then if it is a trap tag
    if (tagId == 8 || tagId == 4)
    {
      if (tagCenterX < xSpeakerFarLimit && tagCenterX > xSpeakerCloseLimit && tagCenterY < ySpeakerUpperLimit && tagCenterX > ySpeakerLowerLimit)
        return true;
    }
    else if (tagId == 5 || tagId == 6)
    {
      if (tagCenterX < xAmpFarLimit && tagCenterX > xAmpCloseLimit && tagCenterY < yAmpUpperLimit && tagCenterX > yAmpLowerLimit)
        return true;
    }
    else if (tagId >= 11 && tagId <= 16)
    {
      if (tagCenterX < xTrapFarLimit && tagCenterX > xTrapCloseLimit && tagCenterY < yTrapUpperLimit && tagCenterX > yTrapLowerLimit)
        return true;
    }
    return false;
  }

  /** 
   * Processor code for april tags
  */
  private void apriltagVisionThreadProcessor() {

    AprilTagDetector detector = new AprilTagDetector();

    detector.addFamily("tag16h5", 0); // Specifies the type of april tags to look for

    // Get the UsbCamera from CameraServer
    UsbCamera camera1 = CameraServer.startAutomaticCapture(1);

    // Set the resolution
    camera1.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat ().
    Mat mat = new Mat();
    Mat grayMat = new Mat();

    // Instantiate colors and list of tags
    ArrayList<Integer> tags = new ArrayList<>();
    Scalar outlineColor = new Scalar(0, 255, 0);
    Scalar xColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      // Convert video from rgb to gray
      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY); 

      boolean importantTags = false; // boolean that's used to check if any important IDs were found
      AprilTagDetection[] detections = detector.detect(grayMat);
      tags.clear(); //Clear list of detected tags from the last loop of code
      // Loop through all the camera's detections and get their IDs and draw a rectangle around them on the camera
      for (AprilTagDetection detection : detections) {

        tags.add(detection.getId());

        if(detection.getId() == 8 || (detection.getId() => 4 && detection.getID() <= 6) || (detection.getId() >= 11 && detection.getId() <= 16)) {
          hasImportantTags = true;
          tagCenterX = detection.getCenterX;
          tagGenterY = detection.getCenterY;
          tagId = detection.getId();
          importantTags = true;
        }

        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        cx = detection.getCenterX();
        cy = detection.getCenterY();
        var ll = 10;

        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
      }

      if (!importantTags) {
        tagCenterX = 0;
        tagCenterY = 0;
      }

      SmartDashboard.putString("tag", tags.toString());
      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }
    detector.close();
  }
}
