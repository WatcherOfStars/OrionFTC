package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.FreightFrenzy.FreightFrenzyNavigator;

class Demobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    BlinkinController blinkinController;

    //Misc
    FtcDashboard dashboard;

    public Demobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {
            //initialize the chassis
        }

        if(USE_PAYLOAD){
            //motors
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("Turret");
            DcMotor duckMotor = opMode.hardwareMap.dcMotor.get("Duck");
            Servo intakeMotor = opMode.hardwareMap.servo.get("Intake");

            //sensors
            DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
            DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");


            blinkinController = new BlinkinController(opMode);

        }

        if(USE_NAVIGATOR){
            //sensors
            DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
            DistanceSensor portDist = opMode.hardwareMap.get(DistanceSensor.class, "portDist");
            DistanceSensor starboardDist = opMode.hardwareMap.get(DistanceSensor.class, "starboardDist");
            DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");
            ColorSensor colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            //initialize navigator

        }
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        //chassis.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){

        //chassis.StartCoreRobotModules();
        //if(USE_NAVIGATOR) navigator.NavigatorOn();
    }

    public void Update(){
        if(USE_PAYLOAD){
        }
    }

    //TODO make sure to stop everything
    public void Stop(){
        if(USE_PAYLOAD) {

        }
    }


    public BlinkinController Lights(){return blinkinController;}

}
