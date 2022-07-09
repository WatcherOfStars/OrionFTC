package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeDCMotorController {
    public DcMotor intakeMotor;
    public double motorDirection = -1 ;  // 1 or -1 = intake
    //public enum IntakeDirection { IN, OUT, STOP }


    public IntakeDCMotorController(OpMode opMode, String motorName) {
        intakeMotor = opMode.hardwareMap.dcMotor.get(motorName) ;
    }

    public void setIn() {
        intakeMotor.setPower(checkMotorSpeed(motorDirection)) ;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
    }
    public void setOut() {
        intakeMotor.setPower(checkMotorSpeed(-motorDirection)) ;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
    }
    public void stop() {
        intakeMotor.setPower( 0 ) ;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
    }
    public void runIntake( double speed ) {
        intakeMotor.setPower( speed*motorDirection ) ;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
    }

    private double checkMotorSpeed(double inputSpeed) {
        // Input scale = -1 to 1. Stop = 0
        // Output scale = 0 to 1. Stop = 0.5
        double outputSpeed = inputSpeed ;
        if (outputSpeed > 1) outputSpeed = 1 ;
        if (outputSpeed < -1) outputSpeed = -1 ;
        return outputSpeed ;
    }
}
