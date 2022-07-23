package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput.Button;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;


@Autonomous(name = "*ERASMUS AUTONOMOUS*", group = "Erasmus")
@Config
public class ErasmusAutonomous extends LinearOpMode
{
    ////Dependencies////
    private ErasmusFreightFrenzy robot;
    private Thread myTestThread ;
    private ControllerInputDummy controllerInputDummy ;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1 ; //used to change how fast robot drives
    public static double turnSpeed = 0.5 ; //used to change how fast robot turns

    public static double turnP = 0.005;  // 0.005
    public static double turnI = 0.0;
    public static double turnD = 0.001;  // 0.01

    private double speedMultiplier = 1 ;
    public static double controllerTurnCoefficient = -2 ;

    // Temporary for testing
    //PIDFCoefficients pidOrig ;
    //public static int payloadControllerNumber = 1 ;
    //MultipleTelemetry dashboardTelemetry ;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ErasmusFreightFrenzy(this, true, true, false);
        controllerInputDummy = new ControllerInputDummy(gamepad1, 1);
        //controllerInputDummy.addListener(this);
        robot.Init() ;
        waitForStart() ;
        // Start button pressed - Autonomous period begins...
        robot.Start() ;
        robot.chassis.ResetGyro();
        robot.chassis.SetHeadlessMode(true);  // TODO: Should be able to drop this
        robot.chassis.SetHeadingPID(turnP, turnI, turnD); // TODO - Move this to init??
        // Execute actions below:
        Runnable runnable = () -> { robot.redAutonomous(); } ;
        myTestThread = new Thread(runnable) ;
        myTestThread.start() ;
        while (!isStopRequested()) {
            robot.updateState(controllerInputDummy, 1, 1, 1);
        }
        myTestThread.interrupt() ;
        robot.stopThread = true ;
        robot.autoDriveSpeed = 0 ;
        robot.updateState(controllerInputDummy, driveSpeed, turnSpeed, speedMultiplier);
    }

}
