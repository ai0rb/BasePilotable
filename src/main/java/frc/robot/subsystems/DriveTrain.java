// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.PWM;
//import java.util.regex.Pattern;

/***********************************************************************************************************************
   * Definition de la classe DriveTrain
   *********************************************************************************************************************/
public class DriveTrain extends SubsystemBase {
  /***************************************************
   * Definition des proprietes de la classe DriveTrain
   ***************************************************/
  private double x;
  private double y;

  private double currentSpeed = 0.0;
  private double deltaSpeed = 0.01;

  private double currentRotation = 0.0;
  private double deltaRotation = 0.05;
  
  double robotSpeedHigh = -1.0;
  double robotSpeedLow = -0.85;

  /**Definition du gyroscope*/
  private AHRS m_Navx = new AHRS();
  /**Definition des moteurs du DriveTrain*/
  private WPI_TalonFX m_moteurAvantGauche = new WPI_TalonFX(Constants.Drivetrain.kMoteurAvantGauchePort);
  private WPI_TalonFX m_moteurAvantDroit = new WPI_TalonFX(Constants.Drivetrain.kMoteurAvantDroitPort);

  private final MotorControllerGroup m_groupeGauche = new MotorControllerGroup(m_moteurAvantGauche);
  private final MotorControllerGroup m_groupeDroit = new MotorControllerGroup(m_moteurAvantDroit);
  
  /**Definition de la classe DifferentialDrive pour la gestion des moteurs de deplacement*/
  //private DifferentialDrive m_Drive = new DifferentialDrive(m_moteurAvantGauche, m_moteurAvantDroit);
  private DifferentialDrive m_Drive;
  private double m_yawOffset = Constants.Drivetrain.kNavXOffset;
  private final Encoder m_leftEncoder = new Encoder(6, 7);
  private final Encoder m_rightEncoder = new Encoder(8, 9);
  private final AnalogGyro m_gyro = new AnalogGyro(0);
   // Simulation drive
    private DifferentialDrivetrainSim m_drivetrainSim;
    private WPI_TalonFX frontLeftSimMotor; // TalonFX
    private WPI_TalonFX frontRightSimMotor;
    private WPI_TalonFX backLeftSimMotor;
    private WPI_TalonFX backRightSimMotor;

    private TalonFXSimCollection frontLeftSimSensors;
    private TalonFXSimCollection frontRightSimSensors;
    private TalonFXSimCollection backLeftSimSensors;
    private TalonFXSimCollection backRightSimSensors;
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final Field2d m_fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
        0.3);

    
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Drivetrain.TRACK_WIDTH_METERS);
  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private DriveTrain instence;
  

  /****************************************************
   * Definition du constructeur de la classe DriveTrain
   ****************************************************/
  /** Constructeur  du Subsystem */
  public DriveTrain() {
    /**Reinitialisation de tous les moteurs de deplacement a leurs valeurs par defaut*/
     m_moteurAvantGauche.configFactoryDefault();  
     m_moteurAvantDroit.configFactoryDefault();  

     m_moteurAvantGauche.setNeutralMode(NeutralMode.Brake);
     m_moteurAvantDroit.setNeutralMode(NeutralMode.Brake);

    //  m_moteurAvantGauche.configOpenloopRamp(0.5);
    //  m_moteurArriereGauche.configOpenloopRamp(0.5);
    //  m_moteurAvantDroit.configOpenloopRamp(0.5);
    //  m_moteurArriereDroit.configOpenloopRamp(0.5);
     
     m_groupeDroit.setInverted(false);
     m_moteurAvantGauche.setInverted(false);
     /**Configuration des moteurs a l'arriere comme suiveurs des moteurs a l'avant*/

     if (RobotBase.isReal()) {
      m_Drive = new DifferentialDrive(m_groupeGauche, m_groupeDroit);
      // Don't let WPI invert, we already did through TalonFX APIs. We want forward to
      // be a green
      // LED on the controllers.
      //m_drive.setRightSideInverted(false);
    }
    if (RobotBase.isSimulation()) {
      frontLeftSimMotor = new WPI_TalonFX(4); // TalonSRX here
      frontRightSimMotor = new WPI_TalonFX(3);
      backLeftSimMotor = new WPI_TalonFX(2);
      backRightSimMotor = new WPI_TalonFX(1);
      frontLeftSimSensors = frontLeftSimMotor.getSimCollection();
      frontRightSimSensors = frontRightSimMotor.getSimCollection();
      backLeftSimSensors = backLeftSimMotor.getSimCollection();
      backRightSimSensors = backRightSimMotor.getSimCollection();
      //m_drive = new DifferentialDrive(new MotorControllerGroup(m_groupeGauche),
          //new MotorControllerGroup(m_groupeDroit));
     // m_drive.setRightSideInverted(false);
      m_drivetrainSim = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getFalcon500(2),
          Drivetrain.DRIVE_GEARING, Drivetrain.TRACK_WIDTH_METERS, Drivetrain.WHEEL_RADIUS_METERS, null // VecBuilder.fill(0,
                                                                                                        // 0, 0.0001,
                                                                                                        // 0.1, 0.1,
                                                                                                        // 0.005, 0.005)
      );
    }
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Drivetrain.WHEEL_RADIUS_METERS / Drivetrain.ENCODER_RESOLUTION);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Drivetrain.WHEEL_RADIUS_METERS / Drivetrain.ENCODER_RESOLUTION);
   // m_leftEncoder.reset();
    //m_rightEncoder.reset();
    resetEncoders(); // reset simulated encoders
    m_groupeDroit.setInverted(true);
    // SmartDashboard.putData("Field", m_fieldSim);
    // SmartDashboard.putData("m_leftPIDController", m_leftPIDController);
    // SmartDashboard.putData("m_rightPIDController", m_rightPIDController);
    // SmartDashboard.putNumber("m_rightPID_Kp", m_rightPIDController.getP());
  }
   /************************************************
   * Definition des methodes de la classe DriveTrain
   *************************************************/
  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // m_drivetrainSim.setInputs(leftInputVoltage, rightInputVoltage);
    //m_drivetrainSim.setInputs(m_moteurAvantGauche.get() * RobotController.getInputVoltage(),
        //-m_moteurAvantDroit.get() * RobotController.getInputVoltage());
    m_drivetrainSim.update(0.02);

    // From NavX example
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-m_drivetrainSim.getHeading().getDegrees(), 360));
    // navxSimAngle = -drivetrainSim.getHeading().getDegrees();
    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
    // Encoders
    frontLeftSimSensors
        .setIntegratedSensorVelocity((int) (metersToTicks(m_drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d));
    backLeftSimSensors
        .setIntegratedSensorVelocity((int) (metersToTicks(m_drivetrainSim.getLeftVelocityMetersPerSecond()) / 10d));
    frontLeftSimSensors.setIntegratedSensorRawPosition((int) metersToTicks(m_drivetrainSim.getLeftPositionMeters()));
    backLeftSimSensors.setIntegratedSensorRawPosition((int) metersToTicks(m_drivetrainSim.getLeftPositionMeters()));
    frontRightSimSensors
        .setIntegratedSensorVelocity((int) (metersToTicks(m_drivetrainSim.getRightVelocityMetersPerSecond()) / 10d));
    backRightSimSensors
        .setIntegratedSensorVelocity((int) (metersToTicks(m_drivetrainSim.getRightVelocityMetersPerSecond()) / 10d));
    frontRightSimSensors.setIntegratedSensorRawPosition((int) metersToTicks(m_drivetrainSim.getRightPositionMeters()));
    backRightSimSensors.setIntegratedSensorRawPosition((int) metersToTicks(m_drivetrainSim.getRightPositionMeters()));
    frontLeftSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    backLeftSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    frontRightSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
    backRightSimSensors.setBusVoltage(RobotController.getBatteryVoltage());
  }
  public static double metersToTicks(double meters) {
    double rotations = meters / Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
    return rotationsToTicks(rotations);
  }
  public static double rotationsToTicks(double rotations) {
    return rotations * Drivetrain.ENCODER_TICKS_PER_ROTATION;
  }
  public static double ticksToMeters(double ticks) {
    return ticksToRotations(ticks) * Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
  }
  // Rotations of the wheel
  public static double ticksToRotations(double ticks) {
    return ticks / (double) Drivetrain.ENCODER_TICKS_PER_ROTATION;
  }
  private double getLeftPosition() {
    if (RobotBase.isSimulation()) {
      return (backLeftSimMotor.getSelectedSensorPosition() + frontLeftSimMotor.getSelectedSensorPosition()) / 2.0d;
    }
    return (m_moteurAvantGauche.getSelectedSensorPosition()) / 1.0d;
  }
  private double getRightPosition() {
    if (RobotBase.isSimulation()) {
      return (backRightSimMotor.getSelectedSensorPosition() + frontRightSimMotor.getSelectedSensorPosition()) / 2.0d;
    }
    return (m_moteurAvantDroit.getSelectedSensorPosition()) / 1.0d;
  }

/**
   * Get distance travelled by the right side
   *
   * @return Distance in meters
   */
  public double getRightDistanceM() {
    double rightTicks = getRightPosition();
    return ticksToMeters(rightTicks);
  }
  /**
   * Get distance travelled by the left side
   *
   * @return Distance in meters
   */
  public double getLeftDistanceM() {
    double leftTicks = getLeftPosition();
    return (-1) * ticksToMeters(leftTicks);
  }
  public double getLeftVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return (frontLeftSimMotor.getSelectedSensorVelocity() + backLeftSimMotor.getSelectedSensorVelocity()) / 2.0d;
    }
    return (m_moteurAvantGauche.getSelectedSensorVelocity()) / 1.0d;
  }
  public double getRightVelocityTicksPerDs() {
    if (RobotBase.isSimulation()) {
      return (frontRightSimMotor.getSelectedSensorVelocity() + backRightSimMotor.getSelectedSensorVelocity()) / 2.0d;
    }
    return (m_moteurAvantDroit.getSelectedSensorVelocity()) / 1.0d;
  }
  /**
   * Get current pose from odometry
   *
   * @return Pose representing position and rotation on field
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public Field2d getFieldSim() {
    return m_fieldSim;
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    resetEncoders();
    if (RobotBase.isSimulation()) {
      m_drivetrainSim.setPose(pose);
    }
    if (RobotBase.isReal()) {
      // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
      m_odometry.resetPosition(pose, m_Navx.getRotation2d());
    }
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    if (RobotBase.isSimulation()) {
      m_odometry.update(
          // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
          // m_rightEncoder.getDistance());
          Rotation2d.fromDegrees(Math.IEEEremainder(m_drivetrainSim.getHeading().getDegrees(), 360)),
          // m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
          // m_drivetrainSim.getLeftPositionMeters(),
          // m_drivetrainSim.getRightPositionMeters());
          m_drivetrainSim.getLeftPositionMeters(), m_drivetrainSim.getRightPositionMeters()
      );
    }
    if (RobotBase.isReal()) {
      m_odometry.update(
          // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
          // m_rightEncoder.getDistance());
          Rotation2d.fromDegrees(getHeading()),
          // m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
          // m_drivetrainSim.getLeftPositionMeters(),
          // m_drivetrainSim.getRightPositionMeters());
          m_moteurAvantGauche.getSelectedSensorPosition(), m_moteurAvantDroit.getSelectedSensorPosition()
      );
    }
  }

 /** Update odometry - this should be run every robot loop. */
 public void periodic() {
  updateOdometry();
  m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
}
  /** Zero all encoders */
  public void resetEncoders() {
    if (RobotBase.isSimulation()) {
      frontLeftSimMotor.setSelectedSensorPosition(0);
      frontRightSimMotor.setSelectedSensorPosition(0);
      backLeftSimMotor.setSelectedSensorPosition(0);
      backRightSimMotor.setSelectedSensorPosition(0);
    }
    m_moteurAvantGauche.setSelectedSensorPosition(0);
    m_moteurAvantDroit.setSelectedSensorPosition(0);
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_groupeGauche.setVoltage(leftOutput + leftFeedforward);
    m_groupeDroit.setVoltage(rightOutput + rightFeedforward);
  }
  
  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the linear velocity speed for the x axis in m/s
   * @param rot    the angular velocity in rad/s
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    // Get the x speed. We are inverting this because Xbox controllers return    // negative values when we push forward. 
    //var xSpeed1 = -m_speedLimiter.calculate(xSpeed) * Drivetrain.kMaxSpeed;
    //final var rot2 = -m_rotLimiter.calculate(xSpeed) * Drivetrain.kMaxAngularSpeed;
    x = xSpeed;
    y = rot;
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(inputsFilter(xSpeed), 0, inputsFilter(rot))));
  }

  public double inputsFilter(double in){
    if (Math.abs(in) < 0.08 ){
      in = 0;
    }
    return in;
  }

  public void displayAxis(){
    // SmartDashboard.putNumber("X Speed", x);
    // SmartDashboard.putNumber("X Speed", y);
  }

  public void resetMyEncoder(){
    m_moteurAvantGauche.setSelectedSensorPosition(0);
  }

  public void resetMyRightEncoder(){
    m_moteurAvantDroit.setSelectedSensorPosition(0);
  }

  public void drive2(double xSpeed, double rot) {
    if(Math.abs(xSpeed) < 0.4){
      xSpeed = 0;
    }

    if(Math.abs(rot) < 0.4){
      rot = 0;
    }

    m_Drive.arcadeDrive(xSpeed, rot);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    double sxSpeed = Math.signum(xSpeed);
    double szRotation = Math.signum(zRotation);

    if(Math.abs(xSpeed) < 0.15){
      xSpeed = 0;
    }

    if(Math.abs(zRotation) < 0.1){
      zRotation = 0;
    }
    
    if (xSpeed < 0) { // recul
      xSpeed = 0.9 * xSpeed;
    }

    if (zRotation > 0) {
      zRotation = 0.9 * zRotation;
    }

    m_Drive.curvatureDrive(xSpeed * sxSpeed * xSpeed, zRotation * szRotation * zRotation, allowTurnInPlace);
  }

  public void curvatureDriveV2(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    m_Drive.arcadeDrive(xSpeed, zRotation, true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    x = leftSpeed;
    y = rightSpeed;
    m_Drive.tankDrive(leftSpeed, rightSpeed);
  }
//Pour le boost
public void boostDrive(double getTrigger, double getRawAxisVertical, double getRawAxisHorizontal) {
  if (getTrigger > 0.5) {
      curvatureDrive((Double) (robotSpeedHigh) * Math.copySign(getRawAxisVertical * getRawAxisVertical, getRawAxisVertical), (Double) (0.7) * Math.copySign(getRawAxisHorizontal * getRawAxisHorizontal * 0.8, getRawAxisHorizontal), true);
  } else {
      curvatureDrive((Double) (robotSpeedLow) * Math.copySign(getRawAxisVertical * getRawAxisVertical, getRawAxisVertical), (Double) (0.7) * Math.copySign(getRawAxisHorizontal * getRawAxisHorizontal * 0.8, getRawAxisHorizontal), true);
  }
}
  public double getLeftSideEncoderPosition(){
    return m_moteurAvantGauche.getSelectedSensorPosition();
  }

  public double getLeftSideEncoderPositionReading(){
    double result = getLeftSideEncoderPosition() * Math.PI * Units.inchesToMeters(6) * (1 / 21596);   
    return result;
  }

  public double getLeftSideEncoderVelocity(){
    return m_moteurAvantGauche.getSelectedSensorVelocity();
  }


  public double getRightSideEncoderPosition(){
    return m_moteurAvantDroit.getSelectedSensorPosition(); // * Math.PI * Units.inchesToMeters(6) * (1 / 21596)
  }

  public double getRightSideEncoderVelocity(){
    return m_moteurAvantDroit.getSelectedSensorVelocity();
  }

  /**Definition de la methode de lecture de la vitesse des roues gauches de la base pilotable
   * donnee par les encodeurs des roues gauches
  */
  public double getLeftEncodeurVelocity() {
    return m_moteurAvantGauche.getSelectedSensorVelocity();
  }
  /**Definition de la methode de lecture de la vitesse des roues droites de la base pilotable
   * donnee par les encodeurs des roues droites
  */
  public double getRightEncodeurVelocity() {
    return m_moteurAvantDroit.getSelectedSensorVelocity();
  }
  /**Definition de la methode de lecture de la distance parcourue des roues gauches de la base pilotable
   * donnee par les encodeurs des roues gauches
  */
  public double getLeftEncodeurDistance() {
    return m_moteurAvantGauche.getSelectedSensorPosition();
  }
  /**Definition de la methode de lecture de la distance parcourue des roues droites de la base pilotable
   * donnee par les encodeurs des roues droites
  */
  public double getRightEncodeurDistance() {
    return m_moteurAvantDroit.getSelectedSensorPosition();
  }

  public void driveFalcon(double speed, double rot){
    if(Math.abs(speed) < 0.1){
      speed = 0;
    }
    if(Math.abs(rot) < 0.1){
      rot = 0;
    }
    m_Drive.arcadeDrive(speed, rot);
  }

  /**Definition de la methode de lecture de l'angle d'orientation du robot donne par le gyroscope*/
  public double getHeading() {
    double heading = -m_Navx.getYaw() + m_yawOffset;
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
  }
  public void OperatorControl() {
    SmartDashboard.putNumber("IMU_TotalYaw", m_Navx.getAngle());
  }
}