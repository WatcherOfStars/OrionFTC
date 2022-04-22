package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServoController {
    public Servo intakeServo ;
    public double servoDirection = -1 ;  // 1 or -1 = intake
    //public enum IntakeDirection { IN, OUT, STOP }


    public IntakeServoController(OpMode opMode, String servoName) {
        intakeServo = opMode.hardwareMap.servo.get(servoName) ;
    }

    public void setIn() { intakeServo.setPosition( convertServoSpeed(servoDirection) ) ; }
    public void setOut() { intakeServo.setPosition ( convertServoSpeed(-servoDirection) ) ; }
    public void stop() { intakeServo.setPosition( 0.5 ) ; }
    public void runIntake( double speed ) {
        intakeServo.setPosition(convertServoSpeed(speed*servoDirection ) );
    }

    private double convertServoSpeed(double inputSpeed) {
        // Input scale = -1 to 1. Stop = 0
        // Output scale = 0 to 1. Stop = 0.5
        double outputSpeed = inputSpeed/2 + 0.5 ;
        if (outputSpeed > 1) outputSpeed = 1 ;
        if (outputSpeed < 0) outputSpeed = 0 ;
        return outputSpeed ;
    }
}
