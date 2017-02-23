

package org.usfirst.frc.team5816.robot;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import poseHandler.PoseContainer;
import poseHandler.PoseHandler;
import poseHandler.PoseHandlerInterface;
/**
 * @author Gregory Tracy
 * @author Adam Naylor
 */
public class Robot extends IterativeRobot implements PoseHandlerInterface {
	
	private CANTalon leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3, winchMotor, shooterMotor;
	private DriveState driveState;
	private Compressor compressor; 
	private Joystick driverController, manipulatorController;
	private Solenoid transmissionSolenoid, gearDropSolenoid;
	private CameraServer cameraServer;
	private UsbCamera camera1, camera2;
	private Gyro gyro;
	private PIDController gyroPID;
	private PIDSource gyroPIDSource;
	private PIDOutput gyroPIDOutput;
	private Double gyroPIDOutputValue;
	private PoseHandler poseHandler;
	private PoseContainer<Double> gyroPoseContainer;
	private TegraHandler tegraHandler;
	private AnalogInput ultrasonicSensor;
	
	private final Double TRIGGER_THRESHOLD = 0.5d;	
	private final Double GYRO_PID_P = 0.05d;
	private final Double GYRO_PID_I = 0.0d;
	private final Double GYRO_PID_D = 0.01d;	
	private final Double GYRO_PID_SCALE = 0.75d;
	
	@Override
	public void robotInit() {
		this.leftMotor1 = new CANTalon(1);
		this.leftMotor2 = new CANTalon(2);
		this.leftMotor3 = new CANTalon(3);
		this.rightMotor1 = new CANTalon(6);
		this.rightMotor2 = new CANTalon(7);
		this.rightMotor3 = new CANTalon(8);
		
		this.shooterMotor = new CANTalon(9);//TODO Fix
		this.winchMotor = new CANTalon(4);//TODO Fix
		
		this.ultrasonicSensor = new AnalogInput(0);
		
		this.setDriveBrakes(false);
		
		this.shooterMotor.enableBrakeMode(false);
		this.winchMotor.enableBrakeMode(true);
		
		this.compressor = new Compressor(11);
		this.driverController = new Joystick(0);
		this.manipulatorController = new Joystick(1);
		this.transmissionSolenoid = new Solenoid(11, 0);
		this.gearDropSolenoid = new Solenoid(11, 1);
		
		this.gyro = new ADXRS450_Gyro();
		
		this.gyroPIDSource = new PIDSource() {
			
			public void setPIDSourceType(PIDSourceType pidSource) {
				// TODO Auto-generated method stub
				
			}
			
			public double pidGet() {
				return getAngle();
			}
			
			public PIDSourceType getPIDSourceType() {
				// TODO Auto-generated method stub
				return PIDSourceType.kDisplacement;
			}
		};
		this.gyroPIDOutput = new PIDOutput() {
			
			public void pidWrite(double output) {
				setGyroPIDOutputValue(output);
				
			}
		};
		this.gyroPID = new PIDController(this.GYRO_PID_P, this.GYRO_PID_I, this.GYRO_PID_D, this.gyroPIDSource, this.gyroPIDOutput);
		this.gyroPID.setContinuous();
		this.gyroPIDOutputValue = 0.0d;
		this.tegraHandler = new TegraHandler();
		
		this.poseHandler = new PoseHandler(this, 5000, 50);
		this.gyroPoseContainer = new PoseContainer<>(this.poseHandler);
		this.poseHandler.start();
		this.compressor.start();
		
	
	}
	
	@Override
	public void autonomousInit() {
		this.setDriveBrakes(false);
	}
	
	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void teleopInit() {
		this.setDriveBrakes(false);
		this.driveState = DriveState.DriverControlledDefault;
//		this.gyroPID.enable();
	}
	
	@Override
	public void teleopPeriodic() {
		
		this.driveState = this.driverController.getRawButton(7) ? DriveState.DriverControlledDefault : this.driveState;
		this.driveState = this.driverController.getRawButton(8) ? DriveState.DriverControlledArcade : this.driveState;
		
		switch(this.driveState){
		case DriverControlledStraight:{
			this.setDriveBrakes(false);
			this.drive(this.getLeftDriveInput(), this.getLeftDriveInput());
			this.transmissionSolenoid.set(this.driverController.getRawAxis(3)>this.TRIGGER_THRESHOLD);
			break;
		}case DriverControlledTurn:{
			this.setDriveBrakes(false);
			this.drive(-this.getRightHorizonalDriveInput(), this.getRightHorizonalDriveInput());
			this.transmissionSolenoid.set(this.driverController.getRawAxis(3)>this.TRIGGER_THRESHOLD);
			break;
		}
		case GyroPIDControlled:{
			this.setDriveBrakes(true);
			this.transmissionSolenoid.set(false);
			this.gyroDrive();
			break;
		}
		case DriverGyroMixControlled:{
			this.setDriveBrakes(false);
			break;
		}
		case DriverControlledArcade:{
			this.setDriveBrakes(false);
			this.drive(this.getLeftDriveInput() + this.getRightHorizonalDriveInput(), this.getLeftDriveInput() - this.getRightHorizonalDriveInput());
			this.transmissionSolenoid.set(this.driverController.getRawAxis(3)>this.TRIGGER_THRESHOLD);
			break;
		}
		case DriverControlledDefault:
		default:{
			this.setDriveBrakes(false);
			this.drive(this.getLeftDriveInput(), this.getRightDriveInput());
			this.transmissionSolenoid.set(this.driverController.getRawAxis(3)>this.TRIGGER_THRESHOLD);
		}
		}
		this.gearDropSolenoid.set(this.manipulatorController.getRawButton(1));
		this.winchMotor.set(this.manipulatorController.getRawButton(4) ? 1.0d : 0.0d);
		this.shooterMotor.set(this.manipulatorController.getRawAxis(2));
		
		SmartDashboard.putData("Gyro PID", this.gyroPID);
		SmartDashboard.putNumber("Gyro", this.getAngle());
		SmartDashboard.putNumber("Get Distance", this.getUltrasonicDistance());

	}
	
	@Override
	public void testInit() {
	}
	
	@Override
	public void testPeriodic() {
		
	}
	
	public void sleep(long time){
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void drive(double left, double right){
		this.leftMotor1.set(left);
		this.leftMotor2.set(left);
		this.leftMotor3.set(left);
		this.rightMotor1.set(-right);
		this.rightMotor2.set(-right);
		this.rightMotor3.set(-right);
	}


	public long getTime() {
		return System.currentTimeMillis();
	}
	
	public double getLeftDriveInput(){
		return -this.driverController.getRawAxis(1);
	}
	
	public double getRightDriveInput(){
		return -this.driverController.getRawAxis(5);
	}
	
	public double getLeftHorizontalDriveInput(){
		return this.driverController.getRawAxis(0);
	}
	
	public double getRightHorizonalDriveInput(){
		return this.driverController.getRawAxis(4);
	}

	public void updatePoseValues() {
		this.gyroPoseContainer.setCurrentValue(this.getAngle());
	}

	public double getAngle(){
		return this.gyro.getAngle();
	}
	
	public void setDriveBrakes(Boolean state){
		this.leftMotor1.enableBrakeMode(state);
		this.leftMotor2.enableBrakeMode(state);
		this.leftMotor3.enableBrakeMode(state);
		this.rightMotor1.enableBrakeMode(state);
		this.rightMotor2.enableBrakeMode(state);
		this.rightMotor3.enableBrakeMode(state);
	}
	
	public void gyroDrive(){
		this.drive((this.gyroPIDOutputValue * this.GYRO_PID_SCALE), -(this.gyroPIDOutputValue * this.GYRO_PID_SCALE));
	}
	
	public void setGyroPIDOutputValue(Double value){
		this.gyroPIDOutputValue = value;
	}
	
	public double getUltrasonicDistance(){
		return this.ultrasonicSensor.getVoltage()/0.000977d;
	}
	
	public enum DriveState{
		DriverControlledDefault, 
		DriverControlledArcade, 
		DriverControlledStraight, 
		DriverControlledTurn, 
		GyroPIDControlled, 
		DriverGyroMixControlled;
	}
	
}
