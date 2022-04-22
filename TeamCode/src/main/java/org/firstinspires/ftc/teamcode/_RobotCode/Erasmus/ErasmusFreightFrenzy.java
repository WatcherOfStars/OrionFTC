package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel ;
import com.qualcomm.robotcore.hardware.TouchSensor ;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.FieldState.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Hashtable;


/*
This class will be the thread runnable class. One instance of this. Many threads (if necessary).
It will instantiate a chassis object and a payload object.

*/

@Config
public class ErasmusFreightFrenzy extends BaseRobot // implements Runnable
{
    ////Dependencies////
    OpMode opMode; // ** Do I need this ?? For encoders?? **
    //Mechanical Components
    public MecanumChassis chassis;
    //public ErasmusPayload payload;
    //Misc
    FtcDashboard dashboard;
    public static double turnCoefficient = 0.02;
    //@Config
    public static double armTop ;
    public DigitalChannel ledRed ;
    public DigitalChannel ledGreen ;

    public static double backupDistance = 1.5 ;
    public static double strafeDistance = 1.3 ;
    //public static int colorWhiteThreshold = 90 ;  // for V3 sensor
    public static int colorWhiteThreshold = 800 ;  // for V2 sensor

    public static double colorBackupSpeed = 0.7 ;


    // NEW Servo Turret =====================================
    //public ServoThroughBoreTurret newTurret ;
    // NEW Servo Turret =====================================
    public DcEncoderActuator newTurret ;
    public static double turretResolution = 2.35 ;
    public static double turretSpeed = 0.7 ;
    public static double turretPointP  = 10 ;
    // NEW DC Motor Arm======================================
    public DcEncoderActuator newArm ;
    public double armPower = 0.3 ;
    public static double armPointP  = 10 ;
    // NEW Servo Intake =====================================
    public IntakeServoController newIntake ;
    // NEW State Process ====================================
    public double targetArmPosition = 0 ;
    public double targetTurretPosition = 0 ;
    public double targetIntakeState = 0 ;
    public double targetRobotAngle  = 0;
    // NEW Sensors ==========================================
    ColorSensor colorSensor ;
    TouchSensor intakeTouchSensor ;
    // For combining controller input with auto driving =====
    public double autoDriveHeading ;
    public double autoDriveSpeed ;
    public double autoTurnOffset ;
    public double manualSpeedMultiplier ;

    // ======================================================

    public ErasmusFreightFrenzy(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.RED, new Pose(0,0,0), usePayload, useChassis, useNavigator);
        opMode = setOpMode;
        dashboard = FtcDashboard.getInstance();
        if(USE_CHASSIS) {
            //initialize the chassis
            chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Erasmus", 200, setOpMode), this);
        }
        opMode.telemetry.update();
        if(USE_PAYLOAD){
            //motors
            //DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Sometimes need this after changing gears.
            //Servo intakeMotor = opMode.hardwareMap.servo.get("Intake");
            //ColorSensor colorSensor =  opMode.hardwareMap.colorSensor.get("colorSensor") ;
            colorSensor =  opMode.hardwareMap.get(ColorSensor.class, "colorSensor") ;
            intakeTouchSensor = opMode.hardwareMap.touchSensor.get("touchSensor") ;
            // NEW turret ======================================================
            newTurret = new DcEncoderActuator(opMode, "Turret", turretResolution);
            newTurret.setSpeed( turretSpeed );
            //newTurret = new ServoThroughBoreTurret( opMode, "Turret", "Turret" ) ;
            // NEW Arm =======================================================
            newArm = new DcEncoderActuator(opMode, "Arm", 24*(537.7/360));
            // NEW Intake =======================================================
            newIntake = new IntakeServoController(opMode, "Intake") ;

            //sensors
            //DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            //payload = new ErasmusPayload(opMode, new _ArmProfile(armMotor), intakeMotor, intakeDist, false);
            //payload.arm.ResetToZero();

            // Lights
            ledGreen = opMode.hardwareMap.get(DigitalChannel.class, "green") ;
            ledRed = opMode.hardwareMap.get(DigitalChannel.class, "red") ;
            ledRed.setMode(DigitalChannel.Mode.INPUT) ;
            ledGreen.setMode(DigitalChannel.Mode.INPUT) ;
        }

        // Ignore USE_NAVIGATOR. We are going to attempt to combine it within this very class.
    }

    public void run(){ }  // For multithreading

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        chassis.InitCoreRobotModules();
    }

    public void Start(){
        chassis.StartCoreRobotModules();  // Starting the mechanum drive
    }

    // FLAG for REMOVAL
    public void Update(){
        if(USE_PAYLOAD){
        }
    }

    public void Stop(){
        //navigation.StopNavigator();
        if(USE_PAYLOAD) {
            //payload.StopArmThread();
            //payload.armThread.StopAutoIntake();
            //payload.StopIntake();
            //payload.arm.SetPowerRaw(0);
        }
    }

    public void updateState(ControllerInput controllerInput1, double driveSpeed, double turnSpeed, double speedMultiplier) {
        // Move arm to position
        newArm.pointP = armPointP ;
        newArm.setTarget(targetArmPosition) ;
        // Move turret to position
        newTurret.pointP = turretPointP ;
        newTurret.setTarget(targetTurretPosition) ;
        // Driving based on controller and automation
        chassis.combinedDrive( controllerInput1, driveSpeed, turnSpeed, speedMultiplier, autoDriveSpeed, autoDriveHeading, targetRobotAngle ) ;
        // Manage intake state
        newIntake.runIntake(targetIntakeState);
    }

    public void setInterlock() {
        double currentTurretPosition = newTurret.getPosition() ;
        double currentArmPosition = newArm.getPosition() ;
        if (currentArmPosition<-30 ) {
            newTurret.setTarget(targetTurretPosition) ;
        }
        if (currentTurretPosition<4 & currentTurretPosition>-4 ) {
            newArm.setTarget(targetArmPosition) ;
        }
        else if ( targetArmPosition<currentArmPosition ) {
            newArm.setTarget(targetArmPosition) ;
        }
        newArm.setTarget(targetArmPosition) ;
    }

    public void testSequence1() {
        targetArmPosition = -30 ;
        DriveForTime(0, -0.8, 0, 1 ) ;
        targetTurretPosition = 90 ;
        targetArmPosition = -80 ;
        DriveForTime(0, 0.5, 0, 1) ;
        DriveForTime(90, 0.5, 0, 0.8) ;
        targetIntakeState = -0.5 ;
        Wait(1) ;
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        Wait(2) ;
        targetArmPosition = 0 ;
    }

    public void testSequence2() {
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        targetArmPosition = 0 ;
        Wait(2) ;
        targetArmPosition = -40 ;
        Wait(5) ;
        targetTurretPosition = 90 ;
        Wait(5) ;
        targetTurretPosition  = 0 ;
        Wait(5) ;
        targetArmPosition = 0 ;
    }

    public void fatherIsAFatWormThatWasLeftInTheSunForTooLong(){
        //DriveForTime(0,-0.3,0,1);
        targetIntakeState = 1;
        autoDriveSpeed= -0.3;
        autoDriveHeading= 0;
        int countDown = 700;
        while (!intakeTouchSensor.isPressed() & countDown>0) { countDown -= 1 ; }
        autoDriveSpeed = 0;
        targetArmPosition = -30;
        targetIntakeState = 0;
        DriveForTime(0,0.8,0,2);
        targetTurretPosition = 90;
        targetArmPosition = -80;
        DriveForTime(90,0.5,0,1.5);
        targetIntakeState = -1;
        Wait(1);
        targetIntakeState = 0;
    }

    public void letoPotato(){
        targetIntakeState = 1;
        autoDriveSpeed= -0.3;
        autoDriveHeading= 0;
        int countDown = 700;
        while (!intakeTouchSensor.isPressed() & countDown>0) { countDown -= 1 ; }
        autoDriveSpeed = 0;
        targetArmPosition = -30;
        targetIntakeState = 0;
        DriveForTime(0,0.8,0,backupDistance);
        targetTurretPosition = 90;
        targetArmPosition = -70;
        DriveForTime(90,0.5,0,strafeDistance);
        targetIntakeState = -0.5;
        Wait(1);
        targetIntakeState = 0;
    }

    public void blueHubDeliver() {
        // =============  Intake until sensed ==============
        int loopCounter = 4 ;
        while (loopCounter > 0) {
            targetIntakeState = 1;
            autoDriveSpeed = 0.4;
            autoDriveHeading = 180;
            int countDown = 700;
            while (!intakeTouchSensor.isPressed() & countDown > 0) {
                countDown -= 1;
                //autoDriveHeading -= 0.1 ;
            }
            autoDriveSpeed = 0;
            autoDriveHeading = 0;
            targetArmPosition = -30;
            targetIntakeState = 0;
            // >>>>>>>>>>>>>> Intake Complete <<<<<<<<<<<<<<<<
            // Drive to white line
            DriveForTime(-10, 0.8, 0, 0.05) ;
            DriveForTime(-40, 1, 0, 0.2) ;
            WallFollowToWhite(colorBackupSpeed, -20, 1.5);
            // At the line Get past pipes and strafe
            targetTurretPosition = 90;
            targetArmPosition = -75;
            DriveForTime(0, 1, 0, 0.3);
            DriveForTime(80, 1, 0, 0.8);
            targetIntakeState = -0.4;
            Wait(0.3);
            // >>>>>>>>>>>>>>> Deliver Complete <<<<<<<<<<<<<<<<<<<<
            targetIntakeState = 0;
            // Drop freight
            targetTurretPosition = 0;
            targetArmPosition = -35;
            Wait(0.5) ;
            DriveForTime(-105, 1, 0, 0.9);
            targetArmPosition = 0;
            WallFollowToWhite(1, -160, 1);
            DriveForTime(170, 0.8, 0, 0.3);
            loopCounter -= 1 ;
        }
    }

    public void lineTest() {
        WallFollowToWhite(colorBackupSpeed, -20, 2) ;
        DriveForTime(0, 1, 0, 0.3) ;
        DriveForTime(80, 1, 0, 0.8) ;
    }

    public void ledSequence() {
        ledRed.setMode(DigitalChannel.Mode.OUTPUT) ;
        ledGreen.setMode(DigitalChannel.Mode.OUTPUT) ;
        ledGreen.setState(true) ;
        Wait(2) ;
        ledGreen.setState(false) ;
        ledRed.setState(true) ;
        Wait(2 ) ;
        ledRed.setState(false) ;
        ledRed.setMode(DigitalChannel.Mode.INPUT) ;
        ledGreen.setMode(DigitalChannel.Mode.INPUT) ;
    }

    public double markTop() {
        //return payload.armGetPosition() ;
        return 10 ;
    }

    //For the moment, I am going to cherry pick a few methods out of Owen's ChassisFunctions class.
    // Ultimately, I expect to either use that class or something like it.
    // But, for now, I want to keep them here.
    // !!! I modified a few of these because I don't have a distance sensor yet !!!

    ////UTILITY / GENERAL FUNCTIONS////
    //Whether a timer has run out
    public boolean IsTimeUp(double startTime, double runTime) { return opMode.getRuntime()<startTime+runTime ; }

    //Wait for a period of time (seconds)
    public void Wait(double time) {
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)){}
    }

    //Drive for a period of time (speed on scale of 1 and time in seconds)
    public void DriveForTime(double angle, double speed, double turnOffset, double time){
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)) {
            //chassis.RawDrive(angle,speed,turnOffset);
            autoDriveHeading = angle ;
            autoDriveSpeed = speed ;
            autoTurnOffset = turnOffset ;
        }
        //chassis.RawDrive(0,0,0);
        autoDriveHeading = 0 ;
        autoDriveSpeed = 0 ;
        autoTurnOffset = 0 ;
    }

    //Drive for a period of time
    public void DriveForTimeToAngle(double driveAngle, double speed, double turnAngle, double coefficient, double time){
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)) chassis.RawDriveTurningTowards(driveAngle,speed,turnAngle,coefficient);
        chassis.RawDrive(0,0,0);
    }

    //Wall follows at specified speed, which also determines direction.
    public void WallFollowForTime(double speed, double time){
        double turnOffset = 0.02*speed;
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)) chassis.RawDrive(0,speed,turnOffset);
        chassis.RawDrive(0,0,0);
    }

    //Wall follows until white is detected by colorSensor. Must be called every loop().
    public void WallFollowToWhite(double speed, double angle){
        double turnOffset = 0.02*speed;
        if(angle>0) turnOffset*=-1;
        //while (payload.colorSensor.alpha() < payload.whiteThreshold ) chassis.RawDrive(angle,speed,turnOffset);
        chassis.RawDrive(0,0,0);
    }

    public void WallFollowToWhite(double speed, double angle, double time){
        double startTime = opMode.getRuntime() ;
        autoDriveHeading = angle ;
        autoDriveSpeed = speed ;
        while (colorSensor.alpha() < colorWhiteThreshold & IsTimeUp(startTime,time)) { }
        autoDriveSpeed = 0 ;
    }

    //Turns to an angle
    public void TurnToAngle(double angle, double speed){
        while (!chassis.InWithinRangeOfAngle(angle,5) ) {
            chassis.TurnTowardsAngle(angle, speed, turnCoefficient);
            opMode.telemetry.addData("Robot Angle", chassis.GetImu().GetRobotAngle());
            opMode.telemetry.update();
        }
    }



    //public ErasmusPayload payload(){return payload;}
    //public EncoderActuator Turret(){return payload.turret;}
    //public EncoderActuator Arm(){return payload.arm;}
}
