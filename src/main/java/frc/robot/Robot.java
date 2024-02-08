// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;   
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.apriltag.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Thread m_visionThread;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  AprilTagDetector tagDetector = new AprilTagDetector();
  

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    System.out.println("1");
    m_visionThread =
    new Thread(
        () -> {
          System.out.println("Starting Vision Thread");

          // Get the UsbCamera from CameraServer and set up
          UsbCamera camera = CameraServer.startAutomaticCapture();
          camera.setResolution(400, 400);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Detection", 640, 480);

          //Set up AprilTagDetector
          AprilTagDetector.Config apeConfig = new AprilTagDetector.Config();
          //apeConfig.debug = true;
          //apeConfig.
          //tagDetector.setConfig(apeConfig);
          //tagDetector.
          tagDetector.addFamily("tag16h5");

          //tagDetector.
          // Mats are very memory expensive. Lets reuse this Mat.
          Mat frame = new Mat();
          Mat g_frame = new Mat();
          
          // Also make an array of detected tags
          AprilTagDetection tags[];

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          //Image Proccesing Loop
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(frame) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }

            Imgproc.cvtColor(frame, g_frame, Imgproc.COLOR_RGB2GRAY);

            //Detect tags and load into 'tags'
            tags = tagDetector.detect(g_frame);
            //System.out.println("April Tags Detected:" + tags.length);


            for (AprilTagDetection tag : tags) {
              //Put a quad around the april tag.
              for (int i=0;i<4;i++) {
                var j = (i + 1) % 4;
                var pt1 = new Point(tag.getCornerX(i),tag.getCornerY(i));
                var pt2 = new Point(tag.getCornerX(j),tag.getCornerY(j));
                Imgproc.line(frame, pt1, pt2, new Scalar(0, 255, 0), 2);
              }
              
              System.out.println("April Tag Detected:" + tag.getId());

            }
            

           // if (tags.length > 0){
              //firstTag = tags[0];
              //System.out.println("April Tag Detected");

              

            //}

            //Edit!
            
            // Give the output stream a new image to display
            outputStream.putFrame(frame);
          }
        });

    //Start Vision Thread
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /** This function is called periodically during autonomous. */
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

}
