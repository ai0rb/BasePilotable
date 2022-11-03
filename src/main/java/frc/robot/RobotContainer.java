// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveRienCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunPathCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.subsystems.Grimpeur;
// import frc.robot.subsystems.PhotoSwitches;
// import frc.robot.subsystems.SuperSubsystem;
// import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final DriveTrain m_driveTrain = new DriveTrain();


  private final Joystick m_pilot = new Joystick(Constants.Manette.kPilotPort);
  // private final XboxController m_pilot = new XboxController(Constants.Manette.kPilotPort);
  private final XboxController m_coPilot = new XboxController(Constants.Manette.kCoPilot);

  private final ExampleCommand m_autoCommand = null;

  public final SendableChooser<Command> autoChooser = new SendableChooser<>();

  String[] oneball = {"/home/lvuser/deploy/kenzy.json"};
  String[] avance = {"/home/lvuser/deploy/AutonomeAvance.json"};
  String[] recule = {"/home/lvuser/deploy/AutonomeRecule.json"};
  // String[] avance = {"/home/lvuser/deploy/FirstAutonomous.json"};
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // autoChooser.addObject(name, object);
    autoChooser.setDefaultOption("Ne Fait Rien", new DriveRienCommand(m_driveTrain));
    autoChooser.addOption("AutonomousAvance", new RunPathCommand(m_driveTrain, avance));
    autoChooser.addOption("AutonomousRecule", new RunPathCommand(m_driveTrain, recule));
    // autoChooser.addOption("AutonomousAvance", new RunPathCommand(m_driveTrain, avance));
    Shuffleboard.getTab("Autonomous").add(autoChooser);

    // m_compressor.enableDigital();
    //m_driveTrain.setDefaultCommand(new DriveCommand(()->(-1)*m_joystick.getY(), ()->m_joystick.getX(), m_driveTrain));
    m_driveTrain.setDefaultCommand(new DriveCommand(()->(-1)*m_pilot.getY(), ()->m_pilot.getX(), m_driveTrain));

    // commenter lorsquon enrengistre
   // m_driveTrain.setDefaultCommand(new DriveCommand(()->(-0.9)*m_pilot.getRawAxis(1), ()->(0.9)*m_pilot.getRawAxis(4), m_driveTrain));
  // m_driveTrain.setDefaultCommand(new RunCommand(()->m_driveTrain.boostDrive( m_pilot.getRightTriggerAxis(), m_pilot.getRawAxis(1),m_pilot.getRawAxis(4)), m_driveTrain));

    //m_driveTrain.setDefaultCommand(new TankDriveCommand(()->(-1)*m_joystick.getY(), ()->m_joystick.getX(), m_driveTrain));
    // m_grimpeur.setDefaultCommand(new GrimpeurSTOPCommand(m_grimpeur));
    // m_ramasseur.setDefaultCommand(new RamasseurStopCommand(m_ramasseur));
    // m_convoyeur.setDefaultCommand(new ConvoyorSTOPCommand(m_convoyeur));
    // m_shooter.setDefaultCommand(new ShooterSTOPCommand(m_shooter));

    //m_driveTrain.setDefaultCommand(new BoostDrive(()->(-1)*m_pilot.getY(), ()->m_joystick.getX(), m_driveTrain));

    // m_convoyeur.setDefaultCommand(new ConvoyorSTOPCommand(m_convoyeur));
    // m_ramasseur.setDefaultCommand(new RamasseurStopCommand(m_ramasseur));
    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // OLD JoyStick Buttons
    // new JoystickButton(m_coPilot, 1).whileHeld(new ConvoyorDOWNCommand(m_convoyeur));
    // new JoystickButton(m_coPilot, 4).whenPressed(new PickUpGroupCommand(m_convoyeur, m_shooter, m_ramasseur, m_photoSwitches));
    // new JoystickButton(m_coPilot, 3).whileHeld(new ConvoyorUPIndividualCommand(m_convoyeur, m_photoSwitches));
    // new JoystickButton(m_pilot, 5).whenPressed(new RamasseurRamasseCommand(m_ramasseur, m_photoSwitches));
    // new JoystickButton(m_pilot, 6).whileHeld(new RamasseurRejetteCommand(m_ramasseur));
    // //new JoystickButton(m_joystick, 6).whileHeld(new PatternCommand(m_driveTrain, 0.45).withTimeout(4));
    // new JoystickButton(m_coPilot, 6).whileHeld(new GrimpeurDOWNCommand(m_grimpeur));
    // new JoystickButton(m_coPilot, 5).whileHeld(new GrimpeurUPCommand(m_grimpeur));
    // new JoystickButton(m_pilot, 1).whileHeld(new RamasseurExtendCommand(m_ramasseur));
    // new JoystickButton(m_pilot, 3).whileHeld(new RamasseurRetractCommand(m_ramasseur));

    // new JoystickButton(m_coPilot, 2).whenPressed(new AimButtonCommand(m_shooter)).whenReleased(
    //                                              new ShootBottom(m_convoyeur, m_ramasseur, m_shooter, m_photoSwitches));

    
    // NEW JoyStick Buttons

    // Pilot
    // new JoystickButton(m_pilot, 1).whenPressed(new AimButtonCommand(m_shooter,m_pilot)).whenReleased(new ShootBottom(m_convoyeur, m_ramasseur, m_shooter, m_photoSwitches, 0.75)); //  make new command for Shooter En Bas
    // new JoystickButton(m_pilot, 5).whenPressed(new AimUpCommand(m_shooter, m_pilot)).whenReleased(new ShootUpCommand(m_convoyeur, m_ramasseur, m_shooter, m_photoSwitches, 0.75)); //  make new command for Shooter en Haut
    // new JoystickButton(m_pilot, 3).whileHeld(new GrimpeurDOWNCommand(m_grimpeur));
    // new JoystickButton(m_pilot, 4).whileHeld(new GrimpeurUPCommand(m_grimpeur));
    // new JoystickButton(m_pilot, 6).whenPressed(new PickUpGroupCommand(m_convoyeur, m_shooter, m_ramasseur, m_photoSwitches));
    // new JoystickButton(m_pilot, 9).whileHeld(new RamasseurRamasseFSMCommand(m_ramasseur, m_photoSwitches));
    
    // Co Pilot
    // new JoystickButton(m_coPilot, 1).whileHeld(new ConvoyorDOWNCommand(m_convoyeur));
    // new JoystickButton(m_coPilot, 4).whileHeld(new ConvoyorUPIndividualCommand(m_convoyeur, m_photoSwitches));
    // new JoystickButton(m_coPilot, 5).whileHeld(new RamasseurRetractCommand(m_ramasseur));
    // new JoystickButton(m_coPilot, 6).whileHeld(new RamasseurExtendCommand(m_ramasseur));
    // new JoystickButton(m_coPilot, 3).whileHeld(new RamasseurRamasseIndividualCommand(m_ramasseur, 0.75));
    // new JoystickButton(m_coPilot, 2).whileHeld(new RamasseurRejetteCommand(m_ramasseur));
    // new JoystickButton(m_coPilot, 9).whileHeld(new ShooterONcommand(m_shooter));
    // new JoystickButton(m_coPilot, 9).whileHeld(new ShooterONPracticecommand(m_shooter));

    //new JoystickButton(m_coPilot, 10).whileHeld(new RamasseurAutoCommand2(m_ramasseur, m_photoSwitches));
   // new JoystickButton(m_coPilot, 8).whileHeld(new AutoShooterONcommand(m_shooter));
    // new JoystickButton(m_coPilot, 10).whileHeld(new ShooterReverseCommand(m_shooter));
  }
  
  // public boolean getC() {
  //   return m_photoSwitches.getPhotoSwitchHigh();
  // }

  // public boolean detectBallC(){
  //   return m_photoSwitches.detectBallC();
  // }

  // public void drawLedPattern(double patterns){
  //   m_convoyeur.drawLedPatterns(patterns);
  // }

  // public boolean getR() {
  //   return m_photoSwitches.getPhotoSwitchLow();
  // }

  // public boolean detectBallR(){
  //   return m_photoSwitches.detectBallR();
  // }
  // public void drawLedPattern2(double patterns){
  //   m_convoyeur.drawLedPatterns2(patterns);
  // }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }


  public Joystick getStick() {
    return m_pilot;
  }

  // public void drawLedPatterns2(double patterns) {
  //   m_driveTrain.drawLedPatterns(patterns); 
  // }

  public double getJoystickX(){
    return m_pilot.getRawAxis(4);
  }

  public double getJoystickY(){
    return m_pilot.getRawAxis(1);
  }

  public void displayAxis(){
    m_driveTrain.displayAxis();
  }

  public void resetMyEncoder(){
    m_driveTrain.resetMyEncoder();
  }

  public void resetMyRightEncoder(){
    m_driveTrain.resetMyRightEncoder();
  }

  public double getRightDistanceM(){
    return m_driveTrain.getRightDistanceM();
  }

  public double getLeftDistanceM(){
    return m_driveTrain.getLeftDistanceM();
  }

  public double getAxis0(){
    return m_pilot.getRawAxis(0);
  }
  public double getAxis1(){
    return m_pilot.getRawAxis(1);
  }
  public Joystick getJoystick(){
    return m_pilot;
  }

  public XboxController getCoJoystick(){
    return m_coPilot;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
    // An ExampleCommand will run in autonomous
    //drawLedPatterns2(0.77);

    //return new DriveCommand(()-> 0.5, ()-> 0, m_driveTrain).withTimeout(2).andThen(
     //      new DriveCommand(()-> -0.5, ()-> 0, m_driveTrain).withTimeout(2)
    //);
  }
}