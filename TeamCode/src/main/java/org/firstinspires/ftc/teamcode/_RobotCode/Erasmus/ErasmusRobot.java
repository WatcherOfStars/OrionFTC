package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.FieldState.Pose;

class ErasmusRobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    public MecanumChassis chassis;
    public ErasmusTurretArm turretArm;
    //TODO: add navigator and refactor duck spinner
    DuckSpinner duckSpinner;
    BlinkinController blinkinController;

    //Misc
    FtcDashboard dashboard;

    public ErasmusRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        //initialize the chassis
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Erasmus", 200, setOpMode), this);

        //sensors
        DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
        DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");
        DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");

        dashboard = FtcDashboard.getInstance();


        if(USE_PAYLOAD){
            //motors
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            //DcMotor tapeMotor = opMode.hardwareMap.dcMotor.get("Tape");
            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("Turret");
            DcMotor duckMotor = opMode.hardwareMap.dcMotor.get("Duck");
            DcMotor intakeMotor = opMode.hardwareMap.dcMotor.get("Intake");
            MotorArray intake = new MotorArray(new DcMotor[]{intakeMotor},new double[]{1},false);

            blinkinController = new BlinkinController(opMode);

            turretArm = new ErasmusTurretArm(opMode, this, blinkinController, new _ArmProfile(armMotor), new _TurretProfile(turretMotor),
                    intake, intakeDist, armLevelDist,false);
            turretArm.arm.ResetToZero();

            duckSpinner = new DuckSpinner(duckMotor, 1);

            //tapeMeasureCapper = new EncoderActuator(opMode,new _TapeMeasureProfile(tapeMotor));
        }

        if(useNavigator){
            /*DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
            DistanceSensor portDist = opMode.hardwareMap.get(DistanceSensor.class, "portDist");
            DistanceSensor starboardDist = opMode.hardwareMap.get(DistanceSensor.class, "starboardDist");
            ColorSensor colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            navigation = new FreightFrenzyNavigation(opMode, this, turretArm, duckSpinner, duckDist, intakeDist, armResetDist, portDist, starboardDist, colorSensor, blinkinController, FreightFrenzyNavigation.AllianceSide.BLUE);
            navigation.SetThread(new Thread(navigation));*/
        }
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        chassis.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){

        chassis.StartCoreRobotModules();
        //navigation.NavigatorOn();
    }

    public void Update(){
        if(USE_PAYLOAD){
        }
    }

    public void Stop(){
        //navigation.StopNavigator();
        turretArm.StopArmThread();
    }

    public ErasmusTurretArm TurretArm(){return turretArm;}
    public EncoderActuator Turret(){return turretArm.turret;}
    public EncoderActuator Arm(){return turretArm.arm;}

    public DuckSpinner DuckSpinner(){return duckSpinner;}

    //public FreightFrenzyNavigation Navigation(){return navigation;}

    public BlinkinController Lights(){return blinkinController;}

}
