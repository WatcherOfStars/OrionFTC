package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// This class is for an DC moto-based actuator that uses the following:
//      - Calculate with:  encoderResolution = (ticks at back starboard)/(180)   or (ticks at x degrees)/x
//              Make sure this is the delta of ticks (account for a non-zero start position, 0 degrees).
//              Dividing the number of ticks on the encoder then give you the position in degrees.
//  - encoderStartPosition = the raw start position of the encoder.
//  *** All calculations are done with respect to degrees. ***
//  GoBilda Worm Gear has 24:1 gear ratio
public class DcEncoderActuator {
    //OpMode opMode ;
    public DcMotorEx motor ;
    //public DcMotor motor ;
    public double encoderStartPosition = 0 ;

    // Configurable settings
    public double encoderResolution = 1 ; // Maps between encoder ticks and degrees
    public double motorPower = 1 ;
    public double pointP = 15 ; // The degrees when proportional kicks in


    public DcEncoderActuator( OpMode opMode, String motorName ) {
        motor = opMode.hardwareMap.get( DcMotorEx.class, motorName ) ;
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        encoderStartPosition = motor.getCurrentPosition() ;  // Calibrate the encoder to zero the position
        //motor.setTargetPosition( (int)encoderStartPosition ) ;
        //motor.setMode( DcMotor.RunMode.RUN_TO_POSITION ) ;
    }
    public DcEncoderActuator( OpMode opMode, String motorName, double resolution ) {
        motor = opMode.hardwareMap.get( DcMotorEx.class, motorName ) ;
        encoderResolution = resolution ;
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        encoderStartPosition = motor.getCurrentPosition() ;  // Calibrate the encoder to zero the position
        //motor.setTargetPosition( (int)encoderStartPosition ) ;
        //motor.setMode( DcMotor.RunMode.RUN_TO_POSITION ) ;
    }

    public void rawTurn( double rawSpeed ) {
        // Scale is -1 to 1. Stop = 0
        motor.setPower( rawSpeed ) ;
        motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER ) ;
    }

    public double getPosition() { return ticksToDegrees( motor.getCurrentPosition() ) ; }  // Position in degrees

    public void setTargetOld( double targetDegrees ) {
        if (Math.abs(targetDegrees-getPosition()) > 30) { motor.setPower(motorPower); }
        else { motor.setPower(motorPower*0.5) ; }
        motor.setTargetPosition( (int)( degreesToTicks( targetDegrees ))) ;
        motor.setMode( DcMotor.RunMode.RUN_TO_POSITION ) ;
    }

    public void setTarget( double targetDegrees ) {
        double error = targetDegrees - getPosition() ;
        if ( Math.abs(error) > 2) {
            if (Math.abs(error) > pointP) motor.setPower(Math.signum(error) * motorPower) ;
            else motor.setPower(Math.signum(error)*0.1 + motorPower*error/pointP) ;
        }
        else motor.setPower( 0 ) ;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
    }

    public void setSpeed( double speed ) { motorPower = speed ; }

    public void setPosition( double newPositionDegrees ) {
        setTarget( newPositionDegrees ) ;
    }

    public void stop() { rawTurn(0) ; }

    private double ticksToDegrees(double measuredTicks) { return ( measuredTicks - encoderStartPosition )/encoderResolution ; }

    private double degreesToTicks(double rawDegrees) { return ( rawDegrees*encoderResolution ) + encoderStartPosition ; }

}
