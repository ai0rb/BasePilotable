// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.io.BufferedOutputStream;
// import java.io.BufferedWriter;
// import java.io.FileOutputStream;
// import java.io.FileWriter;
// import java.io.FilterOutputStream;
// import java.io.IOException;
// import java.io.PrintStream;
// import java.io.PrintWriter;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.DoubleSupplier;


/** An example command that uses an example subsystem. */
public class DriveRienCommand extends CommandBase {
  private double x;
  private double y;

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_DriveTrain;

  //======================Handling time and joystick data============================

  //======================Handling time and joystick data============================
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveRienCommand(DriveTrain driveTrain) {
    m_DriveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.drive2(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.drive2(0, 0); //end value
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
