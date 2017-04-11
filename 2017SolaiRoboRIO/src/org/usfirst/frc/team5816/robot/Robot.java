package org.usfirst.frc.team5816.robot;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import fsm.AllStatesSelector;
import fsm.SingleStateSelector;
import fsm.State;
/**
 * @author Gregory Tracy 
 * @author Adam Naylor
 */
import fsm.StateMachine;
import fsm.TimeAccessor;
import fsm.TimeTransition;
import fsm.Transition;
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
	protected State autoCenter1, autoPlaceGear1, autoPlaceGear2, autoPlaceGear3, noState;
	protected BuiltInAccelerometer accelerometer;

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
	protected final Double TURNING_CONST = 0.65d;



	//	##### Robot Main #####

	@Override
	public void robotInit() {
		this.leftMotor1 = new CANTalon(1);
		this.leftMotor2 = new CANTalon(2);
		this.leftMotor3 = new CANTalon(3);
		this.rightMotor1 = new CANTalon(6);
		this.rightMotor2 = new CANTalon(7);
		this.rightMotor3 = new CANTalon(8);

		this.winchMotor1 = new CANTalon(4);
		this.winchMotor2 = new CANTalon(9);

		this.compressor = new Compressor(11);
		this.gearSolenoid = new Solenoid(11, 1);

		this.driverController = new Joystick(0);
		this.manipulatorController = new Joystick(1);

		this.gyro = new ADXRS450_Gyro();

		this.leftEncoder = new Encoder(0, 1, true);
		this.rightEncoder = new Encoder(2, 3, true);

		this.pidInit();
		
		this.autoStateMachineInit();

//		this.pixyInit();

		this.compressor.start();

		CameraServer.getInstance().startAutomaticCapture();
		
		this.accelerometer = new BuiltInAccelerometer();

	}

	@Override
	public void robotPeriodic() {
		this.setWinchBreaks(true);
//		System.out.println(this.rightEncoder.getRate());
	}


	//	##### Autonomous Main #####	

	@Override
	public void autonomousInit() {
		this.autoStartTime = System.currentTimeMillis();
		this.autoStateMachine.setState(this.autoCenter1);
	}

	@Override
	public void autonomousPeriodic() {
		this.autoStateMachine.update();
	}


	//	##### Teleop Main #####	

	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {
		this.setDriveBreaks(true);
		this.setWinchBreaks(true);
		this.drive((this.getVerticleDriveInput() + this.getHorizontalDriveInput()) * (this.getSpeedModifierInput() ? 0.25d : 1.0d), (this.getVerticleDriveInput() - this.getHorizontalDriveInput()) * (this.getSpeedModifierInput() ? 0.25d : 1.0d));
		this.gearSolenoid.set(this.getGearPlaceInput());
		if(this.getFastWinchInput()){
			this.winchDrive(1.0d);
		}else if(this.getSlowWinchInput()){
			this.winchDrive(0.25d);
		}else{
			this.winchDrive(0.0d);
		}
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
		return this.manipulatorController.getRawButton(1);
	}

	protected boolean getSlowWinchInput() {
		return this.manipulatorController.getRawButton(6);
	}
	
	protected boolean getFastWinchInput() {
		return this.manipulatorController.getRawAxis(3) > this.TRIGGER_THRESHOLD;
	}
	
	@Override
	public Long getTime() {
		
		return System.currentTimeMillis();
	}


	//	##### Computer Vision #####

	private void pixyInit() {
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

	public void pixyReadSerialPort(){
		this.pixyBuffer += this.pixySerialPort.readString();
	}

	public String pixyNextLine(){
		if(this.pixyBuffer.indexOf("\n") == -1){
			return null;
		}
		String out = this.pixyBuffer.substring(0, this.pixyBuffer.indexOf("\n"));
		this.pixyBuffer = (this.pixyBuffer.indexOf("\n") + 1 == this.pixyBuffer.length()) ? "" : this.pixyBuffer.substring(this.pixyBuffer.indexOf("\n") + 1, this.pixyBuffer.length());
		return out;
	}

	public boolean pixyIsLineAvalible(){
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
				// TODO Auto-generated method stub
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
				// TODO Auto-generated method stub
				
			}
		};
		
		autoPlaceGear2 = new State() {
			
			@Override
			protected void periodic() {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			protected void exit() {
				gearSolenoid.set(false);
				
			}
			
			@Override
			protected void enter() {
				gearSolenoid.set(true);
				
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
				// TODO Auto-generated method stub
				
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
				// TODO Auto-generated method stub
				
			}
			
			@Override
			protected void enter() {
				// TODO Auto-generated method stub
				
			}
		};
		
		this.autoStateMachine.addState(autoCenter1);
		this.autoStateMachine.addState(autoPlaceGear1);
		this.autoStateMachine.addState(autoPlaceGear2);
		this.autoStateMachine.addState(autoPlaceGear3);
		this.autoStateMachine.addState(noState);
		
		this.autoStateMachine.addTransition(new Transition(new SingleStateSelector(autoCenter1), autoPlaceGear1) {
			
			@Override
			public Boolean conditionMet() {
				
				return (leftEncoder.getRate() == 0.0d) && (this.getStateMachine().getCurrentStateLog().getEntranceTime() + 1000 < getTime());
			}
		});
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear1), autoPlaceGear2, 100l));
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear2), autoPlaceGear3, 1000l));
		this.autoStateMachine.addTransition(new TimeTransition(new SingleStateSelector(autoPlaceGear3), noState, 300l));
		this.autoStateMachine.addTransition(new Transition(new AllStatesSelector(this.autoStateMachine), noState) {
			
			@Override
			public Boolean conditionMet() {
				return autoStartTime + 6000 < getTime();
			}
		});
	}
	
	


}