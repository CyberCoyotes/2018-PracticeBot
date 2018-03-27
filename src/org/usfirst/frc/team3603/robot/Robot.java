package org.usfirst.frc.team3603.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
   
    final static DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward; //Piston out value
    final static DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse; //Piston in value

    //All of these are individual speed controllers
    WPI_TalonSRX leftFront = new WPI_TalonSRX(10);//TODO
    WPI_TalonSRX leftMiddle = new WPI_TalonSRX(11);//TODO
    WPI_TalonSRX rightFront = new WPI_TalonSRX(4);//TODO
    WPI_TalonSRX rightMiddle = new WPI_TalonSRX(5);//TODO
    //This groups the speed controllers into left and right
    SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftMiddle);
    SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightMiddle);
    //This groups them into the new type of RobotDrive
    DifferentialDrive mainDrive = new DifferentialDrive(left, right);
   
    WPI_TalonSRX leftHolder = new WPI_TalonSRX(9);//TODO Leftholder speedcontroller
    WPI_TalonSRX rightHolder = new WPI_TalonSRX(8);//TODO Rightholder speedcontroller
    WPI_TalonSRX cubeLift = new WPI_TalonSRX(3); //TODO Cube lift speed controller
    WPI_TalonSRX arm = new WPI_TalonSRX(7); //TODO Arm speed controller
   
    //Compressor compressor = new Compressor(); //Air compressor
    DoubleSolenoid shift = new DoubleSolenoid(2, 3);//TODO Transmission solenoid
    Solenoid grabber = new Solenoid(4);//TODO
   
    Joystick joy1 = new Joystick(0); //Large twist-axis joystick
    Joystick joy2 = new Joystick(1); //Xbox controller
    MyEncoder driveEnc = new MyEncoder(leftMiddle, false, highGear);//TODO find motor is is plugged in to
    Encoder armEnc = new Encoder(0, 1, false, EncodingType.k2X); //Arm angle encoder
    WPI_TalonSRX armStore = new WPI_TalonSRX(2); //TODO Speed controller for setting up armPID
    PIDController armPID = new PIDController(0.05, 0, 0, armEnc, armStore); //TODO PID controller for arm
    AHRS gyro = new AHRS(Port.kMXP); //NavX
    PIDController strPID = new PIDController(0.15, 0, 0, gyro, armStore); //TODO PID controller for driving straight
   
    DriverStation matchInfo = DriverStation.getInstance(); //Object to get switch/scale colors
    boolean doOnce = true; //TODO Boolean to only enable the liftPID once at a time
    final static double lowGear = 1/9.07;//TODO check to see if these are correct
    final static double highGear = 1/19.61;
   
    @Override
    public void robotInit() {
        armStore.disable(); //Disable the PID store//TODO remove
        cubeLift.getSensorCollection();//TODO ?
        //compressor.start(); //Start compressor
        mainDrive.setSafetyEnabled(false); //Disable safety
       
        armPID.setOutputRange(-0.5, 0.5); //Set the range of speeds for the arm PID
        new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            // camera.setResolution(320, 480);
            // camera.setFPS(15); //
            camera.setVideoMode(PixelFormat.kMJPEG, 4000, 3000, 15);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            Mat source = new Mat();
            Mat output = new Mat();
            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();
    }
    @Override
    public void autonomousInit() {
        strPID.setSetpoint(0); //Set the setpoint of the drive straight PID to 0 degrees
        driveEnc.reset(); //Set the drive encoder to 0
        gyro.reset(); //Set the gyro angle to 0
        armPID.enable();//Enable the armPID
        armEnc.reset();//Reset the arm encoder
    }
    @Override
    public void autonomousPeriodic() {
    }
   
    @Override
    public void teleopPeriodic() {
       
        /**********
         * DRIVER *
         **********/
       
        double sense = -0.5 * joy1.getRawAxis(3) + 0.5;//Sensitivity coefficient determined by the little axis on the big joystick
        double y = Math.pow(joy1.getRawAxis(1), 1); //Double to store the joystick's y axis
        double rot = -Math.pow(joy1.getRawAxis(2), 1)/1.25; //Double to store the joystick's x axis
        if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05 && !joy1.getRawButton(1)) { //Thresholding function
            mainDrive.arcadeDrive(y * sense, rot * sense); //Arcade drive with the joystick's axis
        } else {
            mainDrive.arcadeDrive(0, 0); //Stop if value doesn't meet threshhold
        }
       
        if(joy1.getRawButton(3)) { //Press and hold button 3 for transmission
            shift.set(out);//Set the transmission piston to out (high gear)
        } else {
            shift.set(in); //Set the transmission piston to in (low gear)
        }
       
        if(joy1.getRawButton(12)) {
            driveEnc.reset();
        }
       
       
        /***************
         * MANIPULATOR *
         ***************/
       
        if(Math.abs(joy2.getRawAxis(5)) >= 0.1) { //If axis 5 is off-center...
            arm.set(joy2.getRawAxis(5));//Set the arm motor to the axis reading
            armPID.setSetpoint(armEnc.get());//Set the armPID setpoint to the current encoder reading, so that it locks in to place
            armPID.enable();//Enable the PID
        } else {//If the joystick isn't being touched...
            arm.set(armPID.get());//Set the arm motor to the armPID
        }
       
      //TODO how should the piston work
        if(Math.abs(joy2.getRawAxis(2)) >= 0.25) { //If the left trigger is pulled...
            leftHolder.set(0.85); //Input cube TODO change these numbers if the intake speed is too fast/slow
            rightHolder.set(0.85);
        } else if(Math.abs(joy2.getRawAxis(3)) >= 0.25) { //If right trigger is pulled...
            leftHolder.set(-0.45);//Soft spit TODO change these numbers if the grabber motors are too fast/slow
            rightHolder.set(-0.45);
        } else if(joy2.getRawButton(5)) { //If left bumper is pressed...
            leftHolder.set(-0.75); // Rotate cube
            rightHolder.set(0.75);
        } else if(joy2.getRawButton(6)) { //If right bumper is pressed...
            leftHolder.set(0.75); // Rotate cube
            rightHolder.set(-0.75);
        } else if(joy2.getRawButton(4)){ //If the Y button is pressed...
            leftHolder.set(-0.75);//Hard spit
            rightHolder.set(-0.75);
        } else {
            leftHolder.set(0);
            rightHolder.set(0);
        }
           
        read();//Read from sensors
    }
   
    void read() {//This puts data onto the smart dashboard
        SmartDashboard.putNumber("Lift speed", cubeLift.get());
        SmartDashboard.putNumber("Arm Encoder", armEnc.get());
        SmartDashboard.putNumber("Arm PID", armPID.get());
        SmartDashboard.putNumber("Arm speed", arm.get());
        SmartDashboard.putNumber("STRAIGHT PID", strPID.get());
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("Drive distance", driveEnc.get());
    }
   
    @Override
    public void testPeriodic() {
    }
   
    enum AutonType {//A list of different auton types
        rightScale, leftScale, rightSwitch, leftSwitch, straight, rightMiddle, leftMiddle
    }
    {
    /*
    void leftMiddle() {
        switch(step) {
        case 1://Step one of the left auton
            liftPID.setSetpoint(12000);//Set the liftPID to 4000
            armPID.setSetpoint(75);//Set the armPID to 75
            armPID.enable();//Enable the armPID
            liftPID.enable();//Enable the liftPID
            strPID.enable();//Enable the drive straight PID
            if(driveEnc.get() < 10) {//If the drive distance is less than 10 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive at -3/4 speed and use the PID
            } else {//If the drive distance greater than 10 inches...
                mainDrive.arcadeDrive(0, 0);///Stop
                step = 2;//Set the step to 2
            }
            break;
        case 2://Step 2 of the left middle auton
            if(gyro.getAngle() > -60) {//If the current angle is greater than -60
                mainDrive.arcadeDrive(0, 0.4);//Slowly turn left
            } else { //If it has reached the -60 degree mark...
                mainDrive.arcadeDrive(0, 0);//Stop
                strPID.setSetpoint(-60);//Set the straightPID setpoint to -60
                strPID.enable();//Enable the straight PID
                driveEnc.reset();//Reset the touchless encoder
                step = 3;//Set the step to 3
            }
            break;
        case 3://Step 3 of the left middle auton
            cubeLift.set(-liftPID.get());//Activate the lift with the PID
            arm.set(armPID.get());//Activate the arm with the PID
            if(driveEnc.get() < 144) {//If the robot has driven less than 144 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive at -3/4 and drive straight with PID
            } else {//If the robot has driven more than 144 inches...
                mainDrive.arcadeDrive(0, 0);//Stop
                step = 4;//Set the step to 4
            }
            break;
        case 4://Step 4
            cubeLift.set(-liftPID.get());//Keep the lift in position
            arm.set(armPID.get());//Keep the arm in position
            if(gyro.getAngle() < 0) {//If the current gyro angle is less than 0...
                mainDrive.arcadeDrive(0, -0.4);//Slowly turn right
            } else {//If it has completely turned 60 degrees...
                mainDrive.arcadeDrive(0, 0);//Stop
                step = 5;//Set the step to 5
            }
            break;
        case 5://Step 5
            cubeLift.set(-liftPID.get());//Keep the cube lift in place
            arm.set(armPID.get());//Keep the arm in place
            leftHolder.set(-0.5);//Output the cube into the switch
            rightHolder.set(-0.5);
            break;
        }
    }
   
    void rightMiddle() {//Method to do the right side of the switch when in position 2
        strPID.enable();//Enable the drive straight PID
        shift.set(out);//Go into high gear
        switch(step) {
        case 1:
            liftPID.setSetpoint(12000);//set the lift PID setpoint to 10000 TODO change this if it is too high/low
            armPID.setSetpoint(75);//Set the armPID setpoint to 75
            armPID.enable();//Enable the armPID
            liftPID.enable();//Enable the liftPID
            cubeLift.set(-liftPID.get());//Activate the lift
            arm.set(armPID.get());//Activate the arm
            if(driveEnc.get() < 83) {//If the robot has driven less than 83 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive straight at -3/4 speed
            } else {//If it has driven 83 inches...
                mainDrive.arcadeDrive(0, 0);//Stop
                step = 2;//Set the step to 2
            }
            break;
        case 2:
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//Keep the arm in place
            leftHolder.set(-0.35);//Output the cube
            rightHolder.set(-0.35);
            break;
        }
    }
    void straight() {//Auton the drive straight
       
        strPID.enable();//Enable the drive straight PID
        if(timer.get() < 3) {//If the robot has driven less than 83 inches...
            mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive straight at -3/4 speed
        } else {//Else
            mainDrive.arcadeDrive(0, 0);//Stop
        }
    }
   
    void rightScale() {//Auton for the right side of the scale
        strPID.enable();//Enable the drive straight PID
        shift.set(out);//Go into high gear
        switch(step) {
        case 1:
            liftPID.setSetpoint(15000);//Set the lift setpoint to 15000
            armPID.setSetpoint(75);//Set the arm setpoint to 75
            armPID.enable();//Enable the armPID
            liftPID.enable();//Enable the liftPID
            cubeLift.set(-liftPID.get());//Activate the lift
            arm.set(armPID.get());//Activate the arm
            if(driveEnc.get() < 252) {//If the robot has driven less than 252 inches TODO change this if the robot drives too short/far
                mainDrive.arcadeDrive(-0.75, -strPID.get()); //Drive forwards at 3/4 speed and drive straight
            } else {//If the robot has driven further than 252 inches
                mainDrive.arcadeDrive(0, 0);//Stop
                step = 2;//Set the step to 2
                liftPID.setSetpoint(24000);//Set the lift setpoint to max height TODO change this if the lift goes too high/low
            }
            break;
        case 2://Step 2
            cubeLift.set(-liftPID.get());//Activate the lift
            arm.set(armPID.get());//Activate the arm
            if(gyro.getAngle() > -45) {//If the current gyro angle is greater than -35 degrees
                mainDrive.arcadeDrive(0, 0.4);//Turn left
            } else {//Else
                step = 3;//Set the step to 3
                driveEnc.reset();//Reset the touchless encoder
                mainDrive.arcadeDrive(0, 0);//Stop
            }
            break;
        case 3://Step 3
            cubeLift.set(-liftPID.get());//Hold the lift in place
            arm.set(armPID.get());//keep the arm in place
            if(liftEnc.get() < 23000) {//Don't activate the cube spitter until the lift has reached a certain height
            } else {
                leftHolder.set(-0.75);
                rightHolder.set(-0.75);
            }
            break;
        }
    }
   
    void leftScale() {//Auton for the left side of the scale
        strPID.enable();//Enable the drive straight PID
        shift.set(out);//Go into the high
        switch(step) {
        case 1:
            liftPID.setSetpoint(15000);//Set the lift setpoint to a little over half way
            armPID.setSetpoint(75);//Set the arm PID setpoint
            armPID.enable();//Enable the arm PID
            liftPID.enable();//Enable the lift PID
            cubeLift.set(-liftPID.get());//Activate the lift
            if(driveEnc.get() < 252) {//If the robot has driven less than 252 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive straight
            } else {//Else
                mainDrive.arcadeDrive(0, 0);//Stop
                step = 2;//go to step 2
                strPID.disable();
                liftPID.setSetpoint(24000);//Set the lift pid setpoint to max
            }
            break;
        case 2://Step 2
            cubeLift.set(-liftPID.get());//Activate the lift
            arm.set(armPID.get());//Activate the arm
            if(gyro.getAngle() < 45) {//If the current gyro angle is less than 45 degrees
                mainDrive.arcadeDrive(0, -0.4);//Turn right
            } else {
                step = 3;//go to step 3
                driveEnc.reset();//reset the touchless encoder
                mainDrive.arcadeDrive(0, 0);//stop
            }
            break;
        case 3://step 3
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//keep the arms in place
            if(liftEnc.get() < 23000) {//dont output the cube until the lift is up
            } else {
                leftHolder.set(-0.75);
                rightHolder.set(-0.75);
            }
            break;
        }
    }
   
    void leftSwitch() {//auton for the left side of the switch
        strPID.enable();//enable the drive straight auton
        shift.set(out);//go in to high gear
        switch(step) {
        case 1:
            liftPID.setSetpoint(12000);//St the lift to 10000
            armPID.setSetpoint(75);//Set the armm to 75
            armPID.enable();//enable the arm pid
            liftPID.enable();//enable the lift pid
            cubeLift.set(-liftPID.get());//activate the lift
            if(driveEnc.get() < 149) {//if the robot has driven less than 149 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//drive straight
            } else {
                mainDrive.arcadeDrive(0, 0);//stop
                strPID.disable();
                step = 2;//go to step 2
            }
            break;
        case 2://step 2
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//activate the arm
            if(gyro.getAngle() < 90) {//if the current angle is less than 90 degrees
                mainDrive.arcadeDrive(0, -0.6);//turn right
            } else {//else
                step = 3;//go to step 3
                driveEnc.reset();//reset the touchless encoder
                mainDrive.arcadeDrive(0, 0);//stop
            }
            break;
        case 3:
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//keep the arm in place
            if(driveEnc.get() < 9) {//if the robot has driven less than 9 inches
                mainDrive.arcadeDrive(-0.5, 0);//drive forwards
            } else {//else
                step = 4;//go to step 4
                mainDrive.arcadeDrive(0, 0);//stop
            }
            break;
        case 4:
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//keep the arm in place
            leftHolder.set(-0.5);//shoot the cube
            rightHolder.set(-0.5);
            break;
        }
    }
   
   
    void rightSwitch() {//auton for the right side of the switch
        strPID.enable();//enable the straight PID
        shift.set(out);//go into high gear
        switch(step) {
        case 1://step 1
            liftPID.setSetpoint(12000);//set the lift setpoint TODO change if too high/low
            armPID.setSetpoint(75);//Set the arm setpoint
            armPID.enable();//enable arm PID
            liftPID.enable();//enable lift PID
            cubeLift.set(-liftPID.get());//activate the lift
            if(driveEnc.get() < 149) {//if the robot has driven less tham 149 inches...
                mainDrive.arcadeDrive(-0.75, -strPID.get());//drive straight
            } else {//else
                mainDrive.arcadeDrive(0, 0);//stop
                strPID.disable();
                step = 2;//go to step 2
            }
            break;
        case 2://step 2
            cubeLift.set(-liftPID.get());//keep the lift in place
            arm.set(armPID.get());//activate the arm
            if(gyro.getAngle() > -90) {//turn left until -90 degrees
                mainDrive.arcadeDrive(0, 0.6);
            } else {
                step = 3;//go to step 3
                driveEnc.reset();//reset the encoder
                mainDrive.arcadeDrive(0, 0);//stop
            }
            break;
        case 3:
            cubeLift.set(-liftPID.get());//keep lift in place
            arm.set(armPID.get());//keep arm in place
            if(driveEnc.get() < 9) {//drive forward 9 inches
                mainDrive.arcadeDrive(-0.5, 0);
            } else {
                step = 4;//go to step 4
                mainDrive.arcadeDrive(0, 0);//stop
            }
            break;
        case 4:
            cubeLift.set(-liftPID.get());//keep lift in place
            arm.set(armPID.get());//keep arm in place
            leftHolder.set(-0.5);//output the cube
            rightHolder.set(-0.5);
            break;
        }
    }
    */}
}