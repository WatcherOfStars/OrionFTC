package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import android.graphics.Bitmap;

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
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigator;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Camera;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

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
    public static double turnCoefficient = 0.02 ;
    //@Config
    public static double armTop ;
    public DigitalChannel ledRed ;
    public DigitalChannel ledGreen ;
    public enum LIGHTS { GREEN, RED, ORANGE, OFF}

    public static double backupDistance = 1.5 ;
    public static double strafeDistance = 1.3 ;
    //public static int colorWhiteThreshold = 90 ;  // for V3 sensor
    public static int colorWhiteThreshold = 800 ;  // for V2 sensor

    public static double colorBackupSpeed = 0.8 ;
    public static double sharedHubBackupSpeed = 0.6 ;
    public static double sharedHubTurretAngle = 110 ;

    // NEW Servo Turret =====================================
    //public ServoThroughBoreTurret newTurret ;
    // NEW Rev Hex Core Turret =====================================
    public DcEncoderActuator newTurret ;
    public static double turretResolution = 3.22 ;
    public static double turretSpeed = 1 ;
    public static double turretEncoderPoint  = 15.2 ;
    public static double turretManualPower = 1 ;
    public static double turretEncoderPower = 1 ;
    public static double turretOffset = 0.25 ;
    // NEW DC Motor Arm======================================
    public DcEncoderActuator newArm ;
    public static double armPower = 1 ;
    public static double armEncoderPoint  = 6.2 ;
    public static double armManualPower = 1 ;
    public static double armEncoderPower = 1 ;
    public static double armOffset = 0.35 ;
    public static double ARM_TOP = -78 ;
    public static double ARM_MIDDLE = -45 ;
    public static double ARM_BOTTOM = -32 ;
    // NEW Servo Intake =====================================
    //public IntakeServoController newIntake ;
    public IntakeDCMotorController newIntake ;
    // NEW State Process ====================================
    public double targetArmPosition = 0 ;
    public double targetTurretPosition = 0 ;
    public double targetIntakeState = 0 ;
    public double targetRobotAngle  = 0 ;
    // Element Gripper  =====================================
    public Servo elementServo ;
    public static double elementServoMin = 0 ;
    public static double elementServoMid = 0.35 ;
    public static double elementServoMax = 0.62 ;
    public static double capHeight = -78 ;
    // NEW Sensors ==========================================
    ColorSensor colorSensor ;
    TouchSensor intakeTouchSensor ;
    DistanceSensor leftDistance ;
    DistanceSensor rightDistance ;
    DistanceSensor frontDistance ;
    DistanceSensor rearDistance ;
    // Webcams ==============================================
    private Camera camera ;
    public static int max1 = 40 ;
    public static int max2 = 255 ;
    public static int max3 = 255 ;
    public static int min1 = 0 ;
    public static int min2 = 110 ;
    public static int min3 = 130 ;
    // For combining controller input with auto driving =====
    public double autoDriveHeading ;
    public double autoDriveSpeed ;
    public double autoTurnOffset ;
    // =====================================================
    public boolean stopThread = false ;

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
            //sensors
            colorSensor =  opMode.hardwareMap.get(ColorSensor.class, "colorSensor") ;
            intakeTouchSensor = opMode.hardwareMap.touchSensor.get("touchSensor") ;
            rightDistance = opMode.hardwareMap.get(DistanceSensor.class, "rightDistance") ;
            leftDistance = opMode.hardwareMap.get(DistanceSensor.class, "leftDistance") ;
            frontDistance = opMode.hardwareMap.get(DistanceSensor.class, "frontDistance") ;
            rearDistance = opMode.hardwareMap.get(DistanceSensor.class, "rearDistance") ;

            // NEW turret ======================================================
            newTurret = new DcEncoderActuator(opMode, "Turret", turretResolution);
            newTurret.setSpeed( turretSpeed );
            newTurret.useBrake = true ;
            //newTurret = new ServoThroughBoreTurret( opMode, "Turret", "Turret" ) ;
            // NEW Arm =======================================================
            newArm = new DcEncoderActuator(opMode, "Arm", 24*(537.7/360));
            newArm.useBrake = true ;
            // NEW Intake =======================================================
            //newIntake = new IntakeServoController(opMode, "Intake") ;
            newIntake = new IntakeDCMotorController(opMode, "Intake") ;
            // Element Gripper
            elementServo = opMode.hardwareMap.servo.get("elementServo" ) ;
            //sensors
            //DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            //payload = new ErasmusPayload(opMode, new _ArmProfile(armMotor), intakeMotor, intakeDist, false);
            //payload.arm.ResetToZero();

            // Lights
            ledGreen = opMode.hardwareMap.get(DigitalChannel.class, "green") ;
            ledRed = opMode.hardwareMap.get(DigitalChannel.class, "red") ;
            ledRed.setMode(DigitalChannel.Mode.INPUT) ;
            ledGreen.setMode(DigitalChannel.Mode.INPUT) ;
            // Camera
            camera = new Camera(opMode,"Webcam 1");

        }
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
        newArm.encoderPoint = armEncoderPoint ;
        newArm.encoderPower = armEncoderPower ;
        newArm.manualPower = armManualPower ;
        newArm.setSpeed(armPower) ;
        newArm.speedOffset = armOffset ;
        //newArm.setTarget(targetArmPosition) ;
        // Move turret to position
        newTurret.encoderPoint = turretEncoderPoint ;
        newTurret.encoderPower = turretEncoderPower ;
        newTurret.manualPower = turretManualPower ;
        newTurret.setSpeed(turretSpeed) ;
        newTurret.speedOffset = turretOffset ;
        //newTurret.setTarget(targetTurretPosition) ;
        setInterlock() ;
        // Driving based on controller and automation
        chassis.combinedDrive( controllerInput1, driveSpeed, turnSpeed, speedMultiplier, autoDriveSpeed, autoDriveHeading, targetRobotAngle ) ;
        // Manage intake state
        newIntake.runIntake(targetIntakeState);
    }

    public void setInterlockOld() {
        double currentTurretPosition = newTurret.getPosition() ;
        double currentArmPosition = newArm.getPosition() ;
        // Send TURRET target
        //if (currentArmPosition<-31 | (targetTurretPosition<3 & targetTurretPosition>-3)) {
        if (currentArmPosition<-31 ) {
            newTurret.setTarget(targetTurretPosition) ;
        }
        //else if (currentTurretPosition<3 & currentTurretPosition>-3) newTurret.setTarget(0) ;
        else if (currentTurretPosition<4 & currentTurretPosition>-4) newTurret.setTarget(0) ;

        else newTurret.setTarget(targetTurretPosition) ;

        // Send ARM target
        if ((currentTurretPosition>3 | currentTurretPosition<-3) & targetArmPosition>-31 & currentArmPosition < -25) {
            newArm.setTarget(-31) ; // Do Nothing
        }
        else  { newArm.setTarget(targetArmPosition) ; }
    }




    public void setInterlock() {
        double currentTurretPosition = newTurret.getPosition() ;
        double currentArmPosition = newArm.getPosition() ;
        // Intake Zone
        if ((currentTurretPosition<3.5 & currentTurretPosition>-3.5)| currentArmPosition>=targetArmPosition) {
            newArm.setTarget(targetArmPosition) ;
            if (currentArmPosition<-31 ) {
                newTurret.setTarget(targetTurretPosition) ;
            }
            else newTurret.setTarget(0) ;
        }
        // Side Zones
        else {
            if (targetArmPosition>-32 & targetArmPosition> currentArmPosition) {
                newArm.setTarget(-33) ;
            }
            else newArm.setTarget(targetArmPosition) ;
            newTurret.setTarget(targetTurretPosition) ;
        }
    }

    public void resetSequence() {
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        targetArmPosition = -34 ;
        Wait(2) ;
        targetArmPosition = 0 ;
        targetTurretPosition = 0 ;
    }

    public void testAutonomous() {
        double cutoffTime = opMode.getRuntime()+23 ; // Use this to decide when to stop and park
        targetArmPosition = -45 ;
        targetTurretPosition = -97 ;
        // Read bar code
        FreightFrenzyNavigator.DuckPos barcodePosition = null ;
        try {barcodePosition = ScanBarcodeOpenCV();} catch(Exception e){ barcodePosition = null ;}
        // Raise / lower arm to bar code position
        double levelDistance = 45 ;
        switch (barcodePosition) {
            case FIRST:
                targetArmPosition = ARM_BOTTOM ;
                levelDistance = 60 ;
                ledSet(LIGHTS.RED) ;
                break ;
            case SECOND:
                targetArmPosition = ARM_MIDDLE ;
                levelDistance = 50 ;
                ledSet(LIGHTS.ORANGE) ;
                break ;
            default:
                targetArmPosition = ARM_TOP ;
                levelDistance = 40 ;
                ledSet(LIGHTS.GREEN) ;
                break ;
        }
        Wait(0.1) ; // Need a little extra to bring the arm up
        // Drive to deliver position
        DriveFromWall(0.9, -65, rightDistance, levelDistance, 2) ;
        Wait(0.2) ;
        // Release freight
        targetIntakeState = -0.5 ;
        newIntake.runIntake(-0.5) ;
        Wait(0.4) ;
        targetTurretPosition = 0 ;
        targetArmPosition = -33 ;
        targetIntakeState = 0 ;
        DriveToWallProportional(0.8, 110, rightDistance, 14, 1.5) ;
        Wait(0.2) ;
        targetArmPosition = 0 ;
        WallFollowToWhite(colorBackupSpeed, 150, 1.0) ;

        // ------------------ Intake/Delivery Loop --------------------------
        int loopCounter = 1 ;
        while (loopCounter > 0 && opMode.getRuntime()<cutoffTime) {
            // ================== INTAKING =================
            targetArmPosition = 0 ;
            targetTurretPosition = 0 ;
            targetIntakeState = 1 ;
            autoDriveSpeed = 0.25 ;
            autoDriveHeading = -155 ;
            int countDown = 1200 ;
            while (!intakeTouchSensor.isPressed() & countDown > 0) { countDown -= 1; } // Wait for intake
            // ================ DELIVERING ====================
            autoDriveSpeed = 0;
            autoDriveHeading = 0;
            targetArmPosition = ARM_TOP;
            targetTurretPosition = -110;

            loopCounter -= 1 ;
        }
        targetIntakeState = 0 ;

    }


    public void redAutonomous() {
        double cutoffTime = opMode.getRuntime()+23 ; // Use this to decide when to stop and park
        // ================= DETECTING ==================
        // Move arm to staging position
        targetArmPosition = -45 ;
        targetTurretPosition = -110 ;
        // Read bar code
        FreightFrenzyNavigator.DuckPos barcodePosition = null ;
        try {barcodePosition = ScanBarcodeOpenCV();} catch(Exception e){ barcodePosition = null ;}
        // Raise / lower arm to bar code position
        switch (barcodePosition) {
            case FIRST:
                targetArmPosition = ARM_BOTTOM ;
                ledSet(LIGHTS.RED) ;
                break ;
            case SECOND:
                targetArmPosition = ARM_MIDDLE ;
                ledSet(LIGHTS.ORANGE) ;
                break ;
            default:
                targetArmPosition = ARM_TOP ;
                ledSet(LIGHTS.GREEN) ;
                break ;
        }
        // Drive to deliver position
        DriveForTime(-65, 1, 0, 0.86) ;  // Drive out to hub
        Wait(0.4) ;
        // Release freight
        targetIntakeState = -0.5 ;
        newIntake.runIntake(-0.5) ;
        Wait(0.3) ;
        targetTurretPosition = 0 ;
        targetArmPosition = 0 ;
        targetIntakeState = 0 ;
        DriveToWall(1, 115, rightDistance, 12, 0.9) ;
        // Drive into warehouse to start loop
        Wait(0.3) ;
        WallFollowToWhite(colorBackupSpeed, 160, 1.5) ;
        while (newArm.getPosition()< -5) {
            try { Thread.sleep(25) ; }
            catch (InterruptedException e) { e.printStackTrace() ; }
        }
        DriveForTime(170, 1, 0, 0.05) ;
        // ======================== Delivery Loop =================================
        int loopCounter = 4 ;
        while (loopCounter > 0 && opMode.getRuntime()<cutoffTime) {
            // ================== INTAKING =================
            // Get payload into position
            targetArmPosition = 0;
            targetTurretPosition = 0;

            // Start intake
            targetIntakeState = 1;
            autoDriveSpeed = 0.35;
            autoDriveHeading = -155 ;
            int countDown = 1200;
            while (!intakeTouchSensor.isPressed() & countDown > 0) {
                countDown -= 1;
                //autoDriveHeading -= 0.1 ;  // Turn a little to get a better intake position
            }
            // ================ DELIVERING ====================
            autoDriveSpeed = 0;
            autoDriveHeading = 0;
            targetArmPosition = ARM_TOP;
            targetTurretPosition = -110;
            // Drive to white line
            DriveForTime(10, 0.8, 0, 0.05) ;
            targetIntakeState = 0;
            //DriveForTime(50, 1, 0, 0.3) ;  // TODO: Change to DriveToWall
            DriveToWall(1, 50, rightDistance, 12, 0.4);
            WallFollowToWhite(colorBackupSpeed, 20, 2.5) ;  // At the line Get past pipes and strafe
            DriveForTime(0, 1, 0, 0.1);  // Get past the line
            //Wait(0.2);
            DriveForTime(-65, 1, 0, 0.75);  // Strafe out to the hub
            Wait(0.3);
            targetIntakeState = -0.5;  // Release
            newIntake.runIntake(-0.5) ;
            Wait(0.3);
            // =================== RETURNING ========================
            targetTurretPosition = 0;
            //DriveForTime(100, 1, 0, 0.3) ;  // TODO: Change ???
            targetIntakeState = 0;
            targetArmPosition = 0;
            //Wait(0.5) ;
            //DriveForTime(115, 1, 0, 0.85);  // TODO: Change to DriveToWall
            DriveToWall(1, 115, rightDistance, 12, 0.9);
            Wait(0.2);
            WallFollowToWhite(colorBackupSpeed-0.1, 160, 1);
            while (newArm.getPosition()< -5) {
                try { Thread.sleep(25); }
                catch (InterruptedException e) { e.printStackTrace(); }
            }
            loopCounter -= 1 ;
        }
        DriveForTime(170, 1, 0, 0.3);
        ledSet(LIGHTS.OFF) ;
    }

    public void redTeamHubDeliver() {
        int loopCounter = 1 ;
        while (loopCounter > 0) {
            // Get payload into position
            targetArmPosition = 0;
            targetTurretPosition = 0;
            // Start intake
            targetIntakeState = 1;
            autoDriveSpeed = 0.2;
            autoDriveHeading = 180;
            int countDown = 1500;
            while (!intakeTouchSensor.isPressed() & countDown > 0) {
                countDown -= 1;
                //autoDriveHeading -= 0.1 ;  // Turn a little to get a better intake position
            }
            autoDriveSpeed = 0;
            autoDriveHeading = 0;
            targetArmPosition = -75;
            targetTurretPosition = -95;
            // >>>>>>>>>>>>>> Intake Complete <<<<<<<<<<<<<<<<
            // Drive to white line
            DriveForTime(10, 0.8, 0, 0.05) ;
            targetIntakeState = 0;
            DriveForTime(40, 1, 0, 0.3) ;
            WallFollowToWhite(colorBackupSpeed, 30, 2.5);
            // At the line Get past pipes and strafe
            DriveForTime(0, 1, 0, 0.2);
            DriveForTime(-95, 1, 0, 0.8);
            targetIntakeState = -0.6;
            Wait(0.4);
            // >>>>>>>>>>>>>>> Deliver Complete <<<<<<<<<<<<<<<<<<<<
            targetIntakeState = 0;
            targetTurretPosition = 0;
            targetArmPosition = 0;
            //Wait(0.5) ;
            DriveForTime(105, 1, 0, 0.9);
            WallFollowToWhite(colorBackupSpeed, -150, 2.5);
            DriveForTime(-170, 0.8, 0, 0.3);
            loopCounter -= 1 ;
        }
    }

    public void redSharedHubDeliver() {
        // =============  Intake until sensed ==============
        int loopCounter = 1 ;
        while (loopCounter > 0 & !stopThread) {
            // Start in warehouse, just across the line, against the wall.
            // Get payload into position
            targetArmPosition = 0;
            targetTurretPosition = 0;
            // Start intake
            targetIntakeState = 1;
            autoDriveSpeed = 0.3;
            autoDriveHeading = 150;
            int countDown = 1500; // Consider eliminating this for teleop (why deliver empty?)
            while (!intakeTouchSensor.isPressed() & countDown > 0) {
                countDown -= 1 ;
                //autoDriveHeading -= 0.1 ; // Turn a little to get a better intake position
            }
            autoDriveSpeed = 0 ;
            autoDriveHeading = 0 ;
            // >>>>>>>>>>>>>> Intake Complete <<<<<<<<<<<<<<<<
            targetArmPosition = -35 ;
            targetTurretPosition = sharedHubTurretAngle ;
            if (stopThread) break ;  // Make sure we stop for kill switch
            // Drive to white line
            DriveForTime(-10, 0.8, 0, 0.05) ;
            targetIntakeState = 0; // Wait just a bit - to make sure we hold it.
            DriveToWall(1, -70, leftDistance, 13, 1.2);
            WallFollowToWhite(sharedHubBackupSpeed, -20, 2.9);
            // At the line Get past pipes and strafe
            Wait(0.5) ;
            /*while (Math.abs(newTurret.getPosition() - targetTurretPosition) > 4) {
                // Wait till the turret is in place
            }*/
            targetIntakeState = -0.7;
            //DriveForTime(-10, 0.5, 0, 0.02);
            Wait(0.4);
            // >>>>>>>>>>>>>>> Deliver Complete <<<<<<<<<<<<<<<<<<<<
            if (stopThread) break ;  // Make sure we stop for kill switch
            targetIntakeState = 0 ;
            targetArmPosition = 0 ;
            targetTurretPosition = 0 ;
            //DriveForTime(-120, 1, 0, 0.1);
            Wait(0.2) ;
            WallFollowToWhite(sharedHubBackupSpeed, -160, 1.1);
            /*while (newArm.getPosition() < -5) {
                // Wait till the arm is down
                if (stopThread) break ;  // Make sure we stop for kill switch
            }*/

            DriveForTime(-170, 0.8, 0, 0.2);
            loopCounter -= 1 ;
            if (stopThread) break ;  // Make sure we stop for kill switch
        }
        targetIntakeState = 0 ;
        autoDriveSpeed = 0.0 ;
        autoDriveHeading = 0 ;
    }

    public void blueTeamHubDeliver() {
        // =============  Intake until sensed ==============
        int loopCounter = 1 ;
        while (loopCounter > 0) {
            // Get payload into position
            targetArmPosition = 0;
            targetTurretPosition = 0;
            // Start intake
            targetIntakeState = 1;
            autoDriveSpeed = 0.3;
            autoDriveHeading = 140;
            int countDown = 1500;
            while (!intakeTouchSensor.isPressed() & countDown > 0) {
                countDown -= 1;
                //autoDriveHeading -= 0.1 ;  // Turn a little to get a better intake position
            }
            autoDriveSpeed = 0;
            autoDriveHeading = 0;
            targetArmPosition = -75;
            targetTurretPosition = 95;
            // >>>>>>>>>>>>>> Intake Complete <<<<<<<<<<<<<<<<
            // Drive to white line
            DriveForTime(-10, 0.8, 0, 0.05) ;
            targetIntakeState = 0;
            DriveForTime(-40, 1, 0, 0.3) ;
            WallFollowToWhite(colorBackupSpeed, -30, 2.5);
            // At the line Get past pipes and strafe
            DriveForTime(0, 1, 0, 0.2);
            DriveForTime(85, 1, 0, 0.9);
            targetIntakeState = -0.6;
            Wait(0.4);
            // >>>>>>>>>>>>>>> Deliver Complete <<<<<<<<<<<<<<<<<<<<
            targetIntakeState = 0;
            targetTurretPosition = 0;
            targetArmPosition = 0;
            //Wait(0.5) ;
            DriveForTime(-105, 1, 0, 0.9);
            WallFollowToWhite(colorBackupSpeed, -150, 2.5);
            DriveForTime(170, 0.8, 0, 0.3);
            loopCounter -= 1 ;
        }
    }


    public void testSequence2() {
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        targetArmPosition = 0 ;
        Wait(2) ;
        targetArmPosition = ARM_TOP ;
        WallFollowToWhite(colorBackupSpeed, -30, 1.5);
        DriveForTime(-10, 0.8, 0, 0.3);
        targetTurretPosition = 97 ;
        Wait(5) ;
        targetArmPosition = 0 ;
        targetTurretPosition = 0 ;
        WallFollowToWhite(colorBackupSpeed, -150, 1.5);
        DriveForTime(-170, 0.8, 0, 0.3);
    }

    public void jerkTestSequence1() {
        ledSet(LIGHTS.RED) ;
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        targetArmPosition = -35 ;
        Wait(2) ;
        targetTurretPosition = 97 ;
        targetArmPosition = ARM_TOP ;
        Wait(4) ;
        targetTurretPosition  = 0 ;
        targetArmPosition = 0 ;
        ledSet(LIGHTS.OFF) ;
    }

    public void jerkTestSequence2() {
        ledSet(LIGHTS.GREEN) ;
        targetIntakeState = 0 ;
        targetTurretPosition = 0 ;
        targetArmPosition = 0 ;
        Wait(2) ;
        targetArmPosition = -72 ;
        targetTurretPosition = 0 ;
        Wait(3) ;
        targetTurretPosition  = 0 ;
        targetArmPosition = 0 ;
        ledSet(LIGHTS.OFF) ;
    }

    public void servoMin() {
        elementServo.setPosition(elementServoMin) ;
    }

    public void servoMid() {
        elementServo.setPosition(elementServoMid) ;
    }

    public void servoMax() {
        elementServo.setPosition(elementServoMax) ;
    }

    public void capSequence() {
        // Grabber is closed on capping element
        targetArmPosition = capHeight ;
        targetTurretPosition = -90 ;
        Wait(2) ;
        //elementServo.setPosition(elementServoMid) ;
    }


    public void distanceTest() {
        DriveToWall(0.7, -90, leftDistance, 10, 2.5);
    }

    public void lineTest() {
        WallFollowToWhite(colorBackupSpeed, -20, 2) ;
        DriveForTime(0, 1, 0, 0.3) ;
        DriveForTime(80, 1, 0, 0.8) ;
    }

    public void intakeTest() {
        autoDriveSpeed = 0;
        autoDriveHeading = 0;
        targetIntakeState = 1;
        //autoDriveSpeed = 0.4;
        //autoDriveHeading = 180;
        int countDown = 1500;
        while (!intakeTouchSensor.isPressed() & countDown > 0) {
            countDown -= 1;
        }
        targetArmPosition = -75;
        Wait(2) ;
        targetIntakeState = -1 ;
        Wait(2) ;
        targetIntakeState = 0 ;
        targetArmPosition = 0 ;
    }

    public void ledSequence1() {
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

    public void ledSequence2() {
        ledSet(LIGHTS.RED) ;
        Wait(2) ;
        ledSet(LIGHTS.GREEN) ;
        Wait(2 ) ;
        ledSet(LIGHTS.ORANGE) ;
        Wait(2 ) ;
        ledSet(LIGHTS.OFF) ;
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
        while (IsTimeUp(startTime,time)){
            try {
                Thread.sleep(25);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    //Drive for a period of time (speed on scale of 1 and time in seconds)
    public void DriveForTime(double angle, double speed, double turnOffset, double time){
        double startTime = opMode.getRuntime();
        autoDriveHeading = angle ;
        autoDriveSpeed = speed ;
        autoTurnOffset = turnOffset ;
        while (IsTimeUp(startTime,time)) {

            try {
                Thread.sleep(20);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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
        while (colorSensor.alpha() < colorWhiteThreshold & IsTimeUp(startTime,time)) {
            try {
                Thread.sleep(25);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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

    public void DriveToWall(double speed, double angle, DistanceSensor distSensor, double distance, double time)  {
        double startTime = opMode.getRuntime() ;
        autoDriveHeading = angle ;
        autoDriveSpeed = speed ;
        while (distSensor.getDistance(DistanceUnit.CM)>distance & IsTimeUp(startTime,time)) {
            try {
                Thread.sleep(20);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        autoDriveSpeed = 0 ;
    }

    public void DriveToWallProportional(double speed, double angle, DistanceSensor distSensor, double distance, double time)  {
        double startTime = opMode.getRuntime() ;
        autoDriveHeading = angle ;
        autoDriveSpeed = speed ;
        while (distSensor.getDistance(DistanceUnit.CM)>distance & IsTimeUp(startTime,time)) {
            double error = distance - distSensor.getDistance(DistanceUnit.CM) ;
            if (error > 30) { this.autoDriveSpeed = speed ; }
            else { this.autoDriveSpeed = error/30 + 0.2 ; }
            try {
                Thread.sleep(20);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        autoDriveSpeed = 0 ;
    }


    public void DriveFromWall(double speed, double angle, DistanceSensor distSensor, double distance, double time)  {
        double startTime = opMode.getRuntime() ;
        autoDriveHeading = angle ;
        autoDriveSpeed = speed ;
        while (distSensor.getDistance(DistanceUnit.CM)<distance & IsTimeUp(startTime, time)) {
            // Proportional speed adjustment
            double error = distance - distSensor.getDistance(DistanceUnit.CM) ;
            this.autoDriveSpeed = error/distance + 0.2 ;
            try {
                Thread.sleep(20);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        autoDriveSpeed = 0 ;
    }

    public void ledSet(LIGHTS color) {
        switch (color) {
            case RED:
                ledRed.setMode(DigitalChannel.Mode.OUTPUT) ;
                ledGreen.setMode(DigitalChannel.Mode.OUTPUT) ;
                //ledGreen.setState(false) ;
                ledRed.setState(true) ;
                break ;
            case GREEN:
                ledRed.setMode(DigitalChannel.Mode.OUTPUT) ;
                ledGreen.setMode(DigitalChannel.Mode.OUTPUT) ;
                ledGreen.setState(true) ;
                ledRed.setState(false) ;
                break ;
            case ORANGE:
                ledRed.setMode(DigitalChannel.Mode.OUTPUT) ;
                ledGreen.setMode(DigitalChannel.Mode.OUTPUT) ;
                ledGreen.setState(false) ;
                ledRed.setState(false) ;
                break ;
            default:
                ledRed.setMode(DigitalChannel.Mode.INPUT) ;
                ledGreen.setMode(DigitalChannel.Mode.INPUT) ;
                ledGreen.setState(false) ;
                ledRed.setState(false) ;
        }
}


    public FreightFrenzyNavigator.DuckPos ScanBarcodeOpenCV() throws InterruptedException {
        //get camera input and convert to mat
        //divide image into three sections
        //find section with most yellow
        FreightFrenzyNavigator.DuckPos pos= FreightFrenzyNavigator.DuckPos.NULL;
        opMode.telemetry.addData("Started scan", opMode.getRuntime());
        Bitmap in = camera.GetImage();
        in = camera.ShrinkBitmap(in,in.getWidth()/3,in.getHeight()/3);
        Mat img = camera.convertBitmapToMat(in);

        Rect firstRect = new Rect(0,0,img.width()/3,img.height());
        Rect secondRect = new Rect(img.width()/3,0,img.width()/3,img.height());
        Rect thirdRect = new Rect(2*img.width()/3,0,img.width()/3,img.height());

        Mat firstMat = new Mat(img,firstRect);
        Mat secondMat = new Mat(img,secondRect);
        Mat thirdMat = new Mat(img,thirdRect);
        //bgr lime(0,255,102)
        Scalar max = new Scalar(max1,max2,max3);
        Scalar min = new Scalar(min1,min2,min3);
        firstMat = camera.isolateColor(firstMat,max,min);
        secondMat = camera.isolateColor(secondMat,max,min);
        thirdMat = camera.isolateColor(thirdMat,max,min);
        Bitmap BigBit = camera.convertMatToBitMap(
                camera.isolateColor(
                        img
                        ,max,min)
        );


        Bitmap first = camera.convertMatToBitMap(firstMat);
        Bitmap second = camera.convertMatToBitMap(secondMat);
        Bitmap third = camera.convertMatToBitMap(thirdMat);

        //FtcDashboard.getInstance().sendImage(first);
        //FtcDashboard.getInstance().sendImage(second);
        //FtcDashboard.getInstance().sendImage(third);
        FtcDashboard.getInstance().sendImage(BigBit);

        double pixelsFirst = camera.countPixels(first);
        double pixelsSecond = camera.countPixels(second);
        double pixelsThird = camera.countPixels(third);

        opMode.telemetry.addData("Pixel Count 1",pixelsFirst);
        opMode.telemetry.addData("Pixel Count 2",pixelsSecond);
        opMode.telemetry.addData("Pixel Count 3",pixelsThird);


        if(pixelsFirst>pixelsSecond&&pixelsFirst>pixelsThird){
            pos= FreightFrenzyNavigator.DuckPos.FIRST;
            opMode.telemetry.addData("Element in position","1");
        }
        else if(pixelsSecond>pixelsFirst&&pixelsSecond>pixelsThird){
            pos= FreightFrenzyNavigator.DuckPos.SECOND;
            opMode.telemetry.addData("Element in position","2");
        }
        else if(pixelsThird>pixelsSecond&&pixelsThird>pixelsFirst){
            pos= FreightFrenzyNavigator.DuckPos.THIRD;
            opMode.telemetry.addData("Element in position","3");
        }
        else if(pos== FreightFrenzyNavigator.DuckPos.NULL) {
            opMode.telemetry.addData("Element in position", "null");
        }
        opMode.telemetry.addData("Scan Done!", opMode.getRuntime());
        // TODO: Remove after playing around with this!!!!
        //opMode.telemetry.update();
        return pos;
    }

    //public ErasmusPayload payload(){return payload;}
    //public EncoderActuator Turret(){return payload.turret;}
    //public EncoderActuator Arm(){return payload.arm;}
}
