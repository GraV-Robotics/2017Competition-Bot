package org.usfirst.frc.team5816.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import fsm.AllStatesSelector;
import fsm.SingleStateSelector;
import fsm.State;
import fsm.StateMachine;
import fsm.TimeAccessor;
import fsm.TimeTransition;
import fsm.Transition;


/**
 * @author Gregory Tracy 
 * @author Adam Naylor
 */
public class Robot extends IterativeRobot implements TimeAccessor{


	protected CANTalon leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3, winchMotor1, winchMotor2;
	protected Compressor compressor; 
	protected Solenoid gearSolenoid;
	protected Joystick driverController, manipulatorController;
	protected Gyro gyro;
	protected PIDController gyroPID;
	protected PIDSource gyroPIDSource;
	protected PIDOutput gyroPIDOutput;
	protected Double gyroPIDOutputValue;
	protected PIDController encoderPID;
	protected PIDSource encoderPIDSource;
	protected PIDOutput encoderPIDOutput;
	protected Double encoderPIDOutputValue;
	protected AnalogInput ultrasonicSensor;
	protected Encoder leftEncoder, rightEncoder;
	protected Runnable pixyHandler;
	protected Thread pixyThread;
	protected SerialPort pixySerialPort;
	protected String pixyBuffer;
	protected Double pixyOutputAngle;
	protected Long pixySightWatchdogCounter;
	protected StateMachine autoStateMachine;
	protected Long autoStartTime;
	protected State autoCenter1, autoNoPressure, autoPlaceGear1, autoPlaceGear2, autoPlaceGear3, noState;
	protected Transition autoHitWall;
	protected BuiltInAccelerometer accelerometer;
	
	protected Boolean autoDriveEnabled;
	protected Boolean autoGearKickEnabled;
	protected Boolean autoHasPressure;
	
	protected Long teleopCount;

	protected final Double TRIGGER_THRESHOLD = 0.5d;
	protected final Double AUTO_CURRENT_THRESHOLD = 10.0d;
	protected final Double GYRO_PID_P = 0.1d;
	protected final Double GYRO_PID_I = 0.0d;
	protected final Double GYRO_PID_D = 0.1d;
	protected final Double GYRO_PID_THRESHOLD = 1.0d;
	protected final Double GYRO_PID_SCALE = 0.50d;
	protected final Double ENCODER_PID_P = 0.0d;
	protected final Double ENCODER_PID_I = 0.0d;
	protected final Double ENCODER_PID_D = 0.0d;
	protected final Double ENCODER_PID_THRESHOLD = 1.0d;
	protected final Double ENCODER_PID_SCALE = 0.35d;
	protected final Double TURNING_CONST = 0.70d;
	protected final String AUTO_DRIVE_KEY = "Auto Drive";
	protected final String AUTO_GEAR_KICK_KEY = "Auto Gear Kick";
	protected final String AUTO_HAS_PRESSURE = "Has Pressure";
	protected final Boolean SINGLE_CONTROLLER_MODE = true;



	//	##### Robot Main #####

	@Override
	public void robotInit() {
		this.leftMotor1 = new CANTalon(1);
		this.leftMotor2 = new CANTalon(2);
		this.leftMotor3 = new CANTalon(3);
		this.rightMotor1 = new CANTalon(6);
		this.rightMotor2 = new CANTalon(7);
		this.rightMotor3 = new CANTalon(10);

		this.winchMotor1 = new CANTalon(4);
		this.winchMotor2 = new CANTalon(9);

		this.compressor = new Compressor(11);
		this.gearSolenoid = new Solenoid(11, 1);

		this.driverController = new Joystick(0);
		this.manipulatorController = new Joystick(1);

		this.gyro = new ADXRS450_Gyro();

		this.leftEncoder = new Encoder(0, 1, true);
		this.rightEncoder = new Encoder(2, 3, true);
		
		this.setDriveCurrentLimit(50);
		this.enableDriveCurrentLimit(false);
		
		this.pidInit();
		
		this.autoStateMachineInit();

//		this.pixyInit();

		this.compressor.start();

		CameraServer.getInstance().startAutomaticCapture();
		
		this.accelerometer = new BuiltInAccelerometer();
		
		this.autoDriveEnabled = true;
		this.autoGearKickEnabled = true;
		this.autoHasPressure = true;
		
		SmartDashboard.putBoolean(this.AUTO_DRIVE_KEY, true);
		SmartDashboard.putBoolean(this.AUTO_GEAR_KICK_KEY, true);
		SmartDashboard.putBoolean(this.AUTO_HAS_PRESSURE, true);

	}

	@Override
	public void robotPeriodic() {
		this.setWinchBreaks(true);
		
		this.autoDriveEnabled = SmartDashboard.getBoolean(this.AUTO_DRIVE_KEY, true);
		this.autoGearKickEnabled = SmartDashboard.getBoolean(this.AUTO_GEAR_KICK_KEY, true);
		this.autoHasPressure = SmartDashboard.getBoolean(this.AUTO_HAS_PRESSURE, true);
	}


	//	##### Autonomous Main #####	

	@Override
	public void autonomousInit() {
		this.autoStartTime = System.currentTimeMillis();
		this.autoHitWall.setFutureState(this.autoGearKickEnabled ? (this.autoHasPressure ? this.autoPlaceGear1 : this.autoNoPressure) : this.noState);
		this.autoStateMachine.setState(this.autoDriveEnabled ? this.autoCenter1 : this.noState);
	}

	@Override
	public void autonomousPeriodic() {
		this.autoStateMachine.update();
	}


	//	##### Teleop Main #####	

	@Override
	public void teleopInit() {
		this.teleopCount = 0l;
	}

	@Override
	public void teleopPeriodic() {
		this.teleopCount++;
		this.setDriveBreaks(true);
		this.setWinchBreaks(true);
		this.drive((this.getVerticleDriveInput() + this.getHorizontalDriveInput()) * (this.getSpeedModifierInput() ? 0.25d : 1.0d), (this.getVerticleDriveInput() - this.getHorizontalDriveInput()) * (this.getSpeedModifierInput() ? 0.25d : 1.0d));
		if(this.getPulseGearPlaceInput()){
			this.gearSolenoid.set(((long)(this.teleopCount/2) % 4)!=0);
		}else{
			this.gearSolenoid.set(this.getGearPlaceInput());
		}
		
		if(this.getFastWinchInput()){
			this.winchDrive(1.0d);
		}else if(this.getSlowWinchInput()){
			this.winchDrive(0.25d);
		}else{
			this.winchDrive(0.0d);
		}
	}
	
	
	//	##### Test Main #####
	@Override
	public void testInit() {
		
	}
	
	@Override
	public void testPeriodic() {
		this.setDriveBreaks(true);
		double speed = this.getVerticleDriveInput();
		this.leftMotor1.set(this.driverController.getRawButton(1) ? speed : 0d);
		this.leftMotor2.set(this.driverController.getRawButton(2) ? speed : 0d);
		this.leftMotor3.set(this.driverController.getRawButton(3) ? speed : 0d);
		this.rightMotor1.set(this.driverController.getRawButton(4) ? speed : 0d);
		this.rightMotor2.set(this.driverController.getRawButton(5) ? speed : 0d);
		this.rightMotor3.set(this.driverController.getRawButton(6) ? speed : 0d);
	}

	//	##### Standard #####

	protected void drive(double left, double right){
		this.leftMotor1.set(left);
		this.leftMotor2.set(left);
		this.leftMotor3.set(left);
		this.rightMotor1.set(-right);
		this.rightMotor2.set(-right);
		this.rightMotor3.set(-right);
	}

	protected void setDriveBreaks(boolean state) {
		this.leftMotor1.enableBrakeMode(state);
		this.leftMotor2.enableBrakeMode(state);
		this.leftMotor3.enableBrakeMode(state);
		this.rightMotor1.enableBrakeMode(state);
		this.rightMotor2.enableBrakeMode(state);
		this.rightMotor3.enableBrakeMode(state);
	}

	protected void winchDrive(double speed){
		this.winchMotor1.set(speed);
		this.winchMotor2.set(-speed);
	}

	protected void setWinchBreaks(boolean state) {
		this.winchMotor1.enableBrakeMode(state);
		this.winchMotor2.enableBrakeMode(state);
	}
	
	protected void setDriveCurrentLimit(int amps){
		this.leftMotor1.setCurrentLimit(amps);
		this.leftMotor2.setCurrentLimit(amps);
		this.leftMotor3.setCurrentLimit(amps);
		this.rightMotor1.setCurrentLimit(amps);
		this.rightMotor2.setCurrentLimit(amps);
		this.rightMotor3.setCurrentLimit(amps);
	}
	
	protected void enableDriveCurrentLimit(boolean state){
		this.leftMotor1.EnableCurrentLimit(state);
		this.leftMotor2.EnableCurrentLimit(state);
		this.leftMotor3.EnableCurrentLimit(state);
		this.rightMotor1.EnableCurrentLimit(state);
		this.rightMotor2.EnableCurrentLimit(state);
		this.rightMotor3.EnableCurrentLimit(state);
	}

	//	##### Input #####

	protected double getVerticleDriveInput(){
		return -this.driverController.getRawAxis(1);
	}

	protected double getHorizontalDriveInput(){
		return this.driverController.getRawAxis(4) * this.TURNING_CONST;
	}

	protected boolean getSpeedModifierInput() {
		return this.driverController.getRawAxis(2) > this.TRIGGER_THRESHOLD;
	}
	
	protected boolean getGearPlaceInput(){
		return this.SINGLE_CONTROLLER_MODE ? this.driverController.getRawButton(1) : this.manipulatorController.getRawButton(1);
	}
	
	protected boolean getPulseGearPlaceInput(){
		return this.SINGLE_CONTROLLER_MODE ? this.driverController.getRawButton(2) : this.manipulatorController.getRawButton(2);
	}

	protected boolean getSlowWinchInput() {
		return this.SINGLE_CONTROLLER_MODE ? this.driverController.getRawButton(6) : this.manipulatorController.getRawButton(6);
	}
	
	protected boolean getFastWinchInput() {
		return this.SINGLE_CONTROLLER_MODE ? this.driverController.getRawAxis(3) > this.TRIGGER_THRESHOLD : this.manipulatorController.getRawAxis(3) > this.TRIGGER_THRESHOLD;
	}
	
	@Override
	public Long getTime() {
		
		return System.currentTimeMillis();
	}


	//	##### Computer Vision #####

	protected void pixyInit() {
		this.pixySerialPort = new SerialPort(9600, edu.wpi.first.wpilibj.SerialPort.Port.kUSB1);
		this.pixyBuffer = "";
		this.pixyOutputAngle = null;
		this.pixySightWatchdogCounter = 0l;
		this.pixyHandler = new Runnable() {

			@Override
			public void run() {
				while(true){
					try {
						Thread.sleep(19);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					pixyReadSerialPort();
					pixySightWatchdogCounter++;
					while(pixyIsLineAvalible()){
						String read = pixyNextLine();    		
						try{
							pixyOutputAngle = Double.parseDouble(read);
							pixySightWatchdogCounter = 0l;
						}catch (Exception e){
							e.printStackTrace();
						}
					}
					if(pixySightWatchdogCounter > 2){
						pixyOutputAngle = null;
					}

				}
			}
		};

		this.pixyThread = new Thread(this.pixyHandler);
		this.pixyThread.start();
	}

	protected void pixyReadSerialPort(){
		this.pixyBuffer += this.pixySerialPort.readString();
	}

	protected String pixyNextLine(){
		if(this.pixyBuffer.indexOf("\n") == -1){
			return null;
		}
		String out = this.pixyBuffer.substring(0, this.pixyBuffer.indexOf("\n"));
		this.pixyBuffer = (this.pixyBuffer.indexOf("\n") + 1 == this.pixyBuffer.length()) ? "" : this.pixyBuffer.substring(this.pixyBuffer.indexOf("\n") + 1, this.pixyBuffer.length());
		return out;
	}

	protected boolean pixyIsLineAvalible(){
		return this.pixyBuffer.contains("\n");
	}


	//	##### PID #####

	private void pidInit(){
		this.gyroPIDSource = new PIDSource() {

			public void setPIDSourceType(PIDSourceType pidSource) {

			}

			public double pidGet() {
				return gyro.getAngle();
			}

			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		this.gyroPIDOutput = new PIDOutput() {

			public void pidWrite(double output) {
				gyroPIDOutputValue = output;

			}
		};
		this.gyroPID = new PIDController(this.GYRO_PID_P, this.GYRO_PID_I, this.GYRO_PID_D, this.gyroPIDSource, this.gyroPIDOutput);
		this.gyroPID.setAbsoluteTolerance(this.GYRO_PID_THRESHOLD);
		this.gyroPID.setContinuous();
		this.gyroPIDOutputValue = 0.0d;

		this.encoderPIDSource = new PIDSource() {

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {

			}

			@Override
			public double pidGet() {

				return getEncoderValue();
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		this.encoderPIDOutput = new PIDOutput() {

			@Override
			public void pidWrite(double output) {
				encoderPIDOutputValue = -output;

			}
		};
		this.encoderPID = new PIDController(this.ENCODER_PID_P, this.ENCODER_PID_I, this.ENCODER_PID_D, this.encoderPIDSource, this.encoderPIDOutput);
		this.encoderPID.setAbsoluteTolerance(this.ENCODER_PID_THRESHOLD);
		this.encoderPID.setContinuous();
		this.encoderPIDOutputValue = 0.0d;
	}

	protected double getEncoderValue(){
		return this.leftEncoder.getDistance();
	}

	protected void driveAtAngle(double speed) {
		this.drive((this.gyroPIDOutputValue * this.GYRO_PID_SCALE) + speed, -(this.gyroPIDOutputValue * this.GYRO_PID_SCALE) + speed);
	}

	
//	##### Auto State Machine #####
	
	private void autoStateMachineInit(){
		this.autoStateMachine = new StateMachine(this);
		
		autoCenter1 = new State() {
			
			@Override
			protected void periodic() {
				driveAtAngle(0.30d);
				
			}
			
			@Override
			protected void exit() {
				drive(0.0d, 0.0d);
				gyroPID.disable();
				
			}
			
			@Override
			protected void enter() {
				setDriveBreaks(true);
				gyroPID.enable();
				gyroPID.setSetpoint(gyro.getAngle());
			}
		};
		
		this.autoNoPressure = new State() {
			
			@Override
			protected void periodic() {
				
				
			}
			
			@Override
			protected void exit() {
				
				
			}
			
			@Override
			protected void enter() {
				
				
			}
		};
		
		autoPlaceGear1 = new State() {
			
			@Override
			protected void periodic() {
				drive(-0.30d, -0.30d);
				
			}
			
			@Override
			protected void exit() {
				drive(0.0d, 0.0d);
				
			}
			
			@Override
			protected void enter() {
				
			}
		};
		
		autoPlaceGear2 = new State() {
			
			@Override
			protected void periodic() {
				gearSolenoid.set(((long)(this.getStateMachine().getCurrentStateLog().getUpdateCount()/2) % 4)!=0);
				
			}
			
			@Override
			protected void exit() {
				gearSolenoid.set(false);
				
			}
			
			@Override
			protected void enter() {
				
			}
		};
		
		autoPlaceGear3 = new State() {
			
			@Override
			protected void periodic() {
				drive(-0.30d, -0.30d);
				
			}
			
			@Override
			protected void exit() {
				drive(0.0d, 0.0d);
				
			}
			
			@Override
			protected void enter() {
				
			}
		};
		
		noState = new State() {
			
			@Override
			protected void periodic(){
				setDriveBreaks(false);
				drive(0.0d, 0.0d);
				gyroPID.disable();
			}
			
			@Override
			protected void exit() {
				
			}
			
			@Override
			protected void enter() {
				
			}
		};
		
		this.autoStateMachine.addState(autoCenter1);
		this.autoStateMachine.addState(this.autoNoPressure);
		this.autoStateMachine.addState(autoPlaceGear1);
		this.autoStateMachine.addState(autoPlaceGear2);
		this.autoStateMachine.addState(autoPlaceGear3);
		this.autoStateMachine.addState(noState);
		
		this.autoHitWall = new Transition(new SingleStateSelector(autoCenter1), autoPlaceGear1) {
			
			@Override
			public Boolean conditionMet() {
				
				return (leftEncoder.getRate() == 0.0d) && (this.getStateMachine().getCurrentStateLog().getEntranceTime() + 1000 < getTime());
			}
		};
		this.autoStateMachine.addTransition(this.autoHitWall);
		this.autoStateMachine.addTransition(new Transition(new SingleStateSelector(this.autoNoPressure), this.autoPlaceGear1) {
			
			@Override
			public Boolean conditionMet() {
				return autoStartTime + 8000 < getTime();
			}
		});
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear1), autoPlaceGear2, 100l));
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear2), autoPlaceGear3, 1500l));
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear3), noState, 300l));
		this.autoStateMachine.addTransition(new Transition(new AllStatesSelector(this.autoStateMachine), noState) {
			
			@Override
			public Boolean conditionMet() {
				return autoStartTime + 14000 < getTime();
			}
		});
	}


}