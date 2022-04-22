package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math ;

// This class is for a turret that uses the following:
//      - Servo to power the turret  (use small (servo) to large (turret) gear ratio.)
//      - Rev Through-Bore Encoder  (plug into DC Motor port. If no DC motor, setup a dummy one.)
// Calibration:
//  - encoderDirection > This is 1 or -1 to make the direction of the encoder match the servo.
//  - encoderResolution > This scales the encoder ticks to match a consistent position based on degrees.
//      - straight (start) = 0, starboard = 90, port = -90, back = -180 & 180
//      - Calculate with:  encoderResolution = (ticks at back starboard)/(180)   or (ticks at x degrees)/x
//              Make sure this is the delta of ticks (account for a non-zero start position, 0 degrees).
//              Dividing the number of ticks on the encoder then give you the position in degrees.
//  - encoderStartPosition = the raw start position of the encoder.
//  *** All calculations are done with respect to degrees. ***
public class ServoThroughBoreTurret {
    //OpMode opMode ;
    public Servo turretServo ;
    public DcMotor turretEncoder ;
    public double encoderStartPosition ;

    // Configurable settings
    public int encoderDirection = 1 ;  // Merge this into encoder resolution
    public double encoderResolution = 65.0 ; // Maps between encoder ticks and degrees
    private double proportionalCoefficient = 0.03 ; // Coefficient to adjust speed based on error

    public ServoThroughBoreTurret(OpMode opMode, String servoName, String encoderName) {
        turretEncoder = opMode.hardwareMap.dcMotor.get(encoderName) ;
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        turretServo = opMode.hardwareMap.servo.get(servoName) ;
        encoderStartPosition = getPosition() ;  // Calibrate the encoder to zero the position
    }

    public void rawTurn(double turnSpeed) {
        // Scale is -1 to 1. Stop = 0
        turretServo.setPosition(convertServoSpeed(turnSpeed)) ;
    }

    public double getPosition() { return ticksToDegrees(turretEncoder.getCurrentPosition()) ; }  // Position in degrees

    public void setTarget( double targetDegrees ) {
        if ( Math.abs(targetDegrees - getPosition()) > 2 ) {
            rawTurn((targetDegrees - getPosition()) * proportionalCoefficient ) ;
        }
        else {
            stop() ;
        }
    }

    public void setPosition( double newPositionDegrees ) {
        while (Math.abs(newPositionDegrees - getPosition()) > 1) {
            rawTurn((newPositionDegrees - getPosition()) * proportionalCoefficient ) ;
        }
        stop() ;
    }

    public void stop() { rawTurn(0) ; }

    private double ticksToDegrees(double measuredTicks) { return (measuredTicks - encoderStartPosition)/encoderResolution ; }

    private double degreesToTicks(double rawDegrees) { return (rawDegrees*encoderResolution) + encoderStartPosition ; }

    private double convertServoSpeed(double inputSpeed) {
        // Input scale = -1 to 1. Stop = 0
        // Output scale = 0 to 1. Stop = 0.5
        double outputSpeed = inputSpeed/2 + 0.5 ;
        if (outputSpeed > 1) outputSpeed = 1 ;
        if (outputSpeed < 0) outputSpeed = 0 ;
        return outputSpeed ;
    }

}
