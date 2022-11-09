// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.playback.Reader;
import frc.robot.playback.Recorder;
// import frc.robot.subsystems.DriveTrain;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Joystick;
// import java.io.File;
// import org.json.simple.JSONObject;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  double robotSpeedHigh = -1.0;
  double robotSpeedLow = -0.85;

  // private Convoyeur m_convoyor;

  // UsbCamera camera1 = CameraServer.startAutomaticCapture(0);
  // UsbCamera camera2 = CameraServer.startAutomaticCapture(1);
  // // UsbCamera camera2;
  // // camera2 = CameraServer.startAutomaticCapture(1);
  // NetworkTableEntry cameraSelection;

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
 // Joystick joy1 = new Joystick(0);
  NetworkTableEntry cameraSelection;


  // An example trajectory to follow during the autonomous period.
  // private Trajectory m_trajectory;
  // The Ramsete Controller to follow the trajectory.
  // private final RamseteController m_ramseteController = new
  // RamseteController();
  // The timer to use during the autonomous period.
  // private Timer m_timer;
  // Create Field2d for robot and trajectory visualizations.
  private Field2d m_field;
  // private File deployDir = Filesystem.getDeployDirectory();
  // private File speedFile ;
  // private File rotateFile ;
  // private Scanner inputSpeed;
  // private Scanner inputRotate;
  // private int counter;
  // private final Drivetrain m_drive = new Drivetrain();
  // private final Joystick m_joystick = new Joystick(0);
  // Here are the options for using recordable autonomous mode.
  private boolean record = false, play = true;
  // To which location the recordings should be stored (if a file of the same
  // name already exists (such as foobar.json), a new name will be chosen
  // (foobar(1).json, etc.))
  String recordingURL = "/home/lvuser/AutonomeAvance3.json";
  // An array of which files should be played back during autonomous
  // private File deployDir = Filesystem.getDeployDirectory();
  // private File speedFile = new File(deployDir, "foobar.json");
  String[] playbackURLs = { "/home/lvuser/AutonomeAvance3.json"};
  // These variables are necessary, but need not be initialized
  long initialTime;
  Reader reader;
  Recorder recorder;
  int currentRecordingIndex;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    // m_trajectory =
    // TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    // List.of(new Translation2d(0.25, 0), new Translation2d(0.5, 0)),
    // new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
    // new TrajectoryConfig(Units.feetToMeters(2.0), Units.feetToMeters(2.0)));
    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    // Push the trajectory to Field2d.
    // m_field.getObject("traj").setTrajectory(m_trajectory);
    // counter = 0;

    // camera1 = CameraServer.startAutomaticCapture(0);
    // camera2 = CameraServer.startAutomaticCapture(1);

    // cameraSelection =
    // NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    camera1.setResolution(120, 160);
    camera2.setResolution(120, 160);

    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Closes all readers and recorder (allows files to close and/or save)
    if (recorder != null) {
      recorder.close();
      recorder = null;
    }
    if (reader != null) {
      reader.close();
      reader = null;
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
    // Chooses the first recording
    currentRecordingIndex = 0;
    // Recordable autonomous
    if (play) {
      reader = initializeReader(playbackURLs[currentRecordingIndex]);
    }
  }

  private Reader initializeReader(String playbackURL) {
    Reader reader;
    try {
      // System.out.println(playbackURL); //for testing purpose only
      reader = new Reader(playbackURL);
      initialTime = System.currentTimeMillis();
    } catch (Exception e) {
      // This segment will execute if the file is missing or has the wrong
      // permissions
      reader = null;
      e.printStackTrace();
    }
    return reader;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if (reader != null) {
    // if (reader.isDone(System.currentTimeMillis() - initialTime)
    // && currentRecordingIndex < playbackURLs.length - 1) {
    // reader.close();
    // This will choose the next recording
    // reader = initializeReader(playbackURLs[++currentRecordingIndex]);
    // }
    // JSONObject current = reader.getDataAtTime(System.currentTimeMillis() -
    // initialTime);
    // m_robotContainer.m_drive.driveFalcon((Double) current.get("v"), (Double)
    // current.get("omega"));
    // arm.setRawSpeed((Double) current.get("arm"));
    // conveyor.setSpeed((Double) current.get("intake"));
    // }
    m_autonomousCommand = m_robotContainer.autoChooser.getSelected();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    // System.out.println(record);
    if (record) {
      // This initializes the recorder. The former parameter is the keys,
      // and the latter is the defaults to use.
      recorder = new Recorder(new String[] { "v", "omega", "arm", "intake" }, new Object[] { 0.0, 0.0, 0.0, 0.0 },
          recordingURL);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Pour enregistrement
    // ------------------------------------------------------------------------------------------------------------
    if (recorder != null) {
     
     Object[] input = new Object[] { (-0.7) * m_robotContainer.getJoystick().getRawAxis(1),
         (0.7) * m_robotContainer.getJoystick().getRawAxis(4), 0.0, 0.0 };
      // Do stuff to drive with the inputs.
     m_robotContainer.m_driveTrain.curvatureDriveV2((Double) input[0], (Double) input[1], true);

     recorder.appendData(input);
   }
    //------------------------------------------------------------------------------------------------------------
//Pour le boost
// if (m_robotContainer.getJoystick().getRightTriggerAxis() > 0.5) {
//   m_robotContainer.getDriveTrain().curvatureDrive((Double) (robotSpeedHigh) * m_robotContainer.getJoystick().getRawAxis(1), (Double) (0.7) * m_robotContainer.getJoystick().getRawAxis(4), true);
//   }
//   else{
//     m_robotContainer.getDriveTrain().curvatureDrive((Double) (robotSpeedLow) * m_robotContainer.getJoystick().getRawAxis(1), (Double) (0.7) * m_robotContainer.getJoystick().getRawAxis(4), true);
// }


//Led pour Convoyeur
    // if (m_robotContainer.detectBallC() == true) {
    //   m_robotContainer.drawLedPattern(0.77);
    // } else if (m_robotContainer.detectBallC() == false) {
    //   m_robotContainer.drawLedPattern(0.91);
    // }
//Led pour ramasseur
    // if (m_robotContainer.detectBallR() == true) {
    //   m_robotContainer.drawLedPattern2(0.77); 
    // } else if (m_robotContainer.detectBallR() == false) {
    //   m_robotContainer.drawLedPattern2(0.91);
    // }
   
  

    if (m_robotContainer.getCoJoystick().getLeftTriggerAxis() > 0.5) {
      // System.out.println("Setting camera 2");
      server.setSource(camera2);
    } else if (m_robotContainer.getCoJoystick().getRightTriggerAxis() > 0.5) {
      // System.out.println("Setting camera 1");
      server.setSource(camera1);
    }

    // if (m_robotContainer.getStick().getRawButton(12) == true) {
    // toggleCamera1();
    // } else if (m_robotContainer.getStick().getRawButton(11) == true) {
    // toggleCamera2();
    // }
  }

  public void toggleCamera1() {
    // camera1 = CameraServer.startAutomaticCapture(0);
    // if (m_robotContainer.getStick().getRawButton(12) == true) {
    // // do stuff
    // System.out.println("Setting camera 0");
    // camera1 = CameraServer.startAutomaticCapture(0);
    // }
  }

  public void toggleCamera2() {
    // camera2 = CameraServer.startAutomaticCapture(1);
    // if (m_robotContainer.getStick().getRawButton(11) == true) {
    // // do stuff
    // System.out.println("Setting camera 0");
    // camera1 = CameraServer.startAutomaticCapture(1);
    // }
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}