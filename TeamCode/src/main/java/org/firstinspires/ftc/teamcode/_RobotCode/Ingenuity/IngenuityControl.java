package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

@Config
public class IngenuityControl extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components

    ////Variables////
    //Calibration
    private double levelPitchThreshold = 5;


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public IngenuityControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new IngenuityNavProfile(), new HermesLog("Ingenuity", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        super.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

    public boolean IsRobotLevel(){
        double pitch = imu.GetRawAngles().secondAngle;
        opMode.telemetry.addData("Robot pitch: ", pitch);

        return !(Math.abs(pitch) > 5);
    }
}
