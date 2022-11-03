// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.playback.Reader;
import frc.robot.playback.Recorder;
import frc.robot.subsystems.DriveTrain;

import org.json.simple.JSONObject;
import edu.wpi.first.wpilibj2.command.CommandBase;
/** An example command that uses an example subsystem. */
public class RunPathCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  //SuperSubsystem m_ramasseur = new SuperSubsystem();
  // Here are the options for using recordable autonomous mode.
 boolean record = false, play = true;
 // To which location the recordings should be stored (if a file of the same
 // name already exists (such as foobar.json), a new name will be chosen
 // (foobar(1).json, etc.))
 //String recordingURL = "/home/lvuser/MonAuto.json";
 // An array of which files should be played back during autonomous
 //private File deployDir = Filesystem.getDeployDirectory();
 //private File speedFile = new File(deployDir, "foobar.json");
 //String[] playbackURLs = {"/home/lvuser/deploy/autoangelik(5).json"} ;
 String[] playbackURLs ;
 // These variables are necessary, but need not be initialized
 long initialTime;
 Reader reader;
 Recorder recorder;
 int currentRecordingIndex;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * path_to_run = {"/home/lvuser/deploy/autoangelik(5).json"}
   */
  public RunPathCommand(DriveTrain drivetrain, String[] path_to_run) {
    m_drivetrain = drivetrain;
    //m_ramasseur = ramasseur;
    playbackURLs = path_to_run;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentRecordingIndex = 0;
    // Recordable autonomous
    if (play) {
        reader = initializeReader(playbackURLs[currentRecordingIndex]);
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (reader != null) {
      if (reader.isDone(System.currentTimeMillis() - initialTime)
              && currentRecordingIndex < playbackURLs.length - 1) {
          reader.close();
          // This will choose the next recording
          reader = initializeReader(playbackURLs[++currentRecordingIndex]);
      }
      JSONObject current = reader.getDataAtTime(System.currentTimeMillis() - initialTime);
      m_drivetrain.driveFalcon((Double) current.get("v"), (Double) current.get("omega"));
      //m_ramasseur.ramasser();
     // arm.setRawSpeed((Double) current.get("arm"));
     // conveyor.setSpeed((Double) current.get("intake"));
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       // Closes all readers and recorder (allows files to close and/or save)
       if (recorder != null) {
        recorder.close();
        recorder = null;
    }
    if (reader != null) {
        reader.close();
        reader = null;
    }
    m_drivetrain.driveFalcon(0,0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
}