package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// This class is for an DC moto-based actuator that uses the following:
//      - Calculate with:  encoderResolution = (ticks at back starboard)/(180)   or (ticks at x degrees)/x
//              Make sure this is the delta of ticks (account for a non-zero start position, 0 degrees).
//              Dividing the number of ticks on the encoder then gives you the position in degrees.
//  - encoderStartPosition = the raw start position of the encoder.
//  *** All calculations are done with respect to degrees. ***
//  GoBilda Worm Gear has 24:1 gear ratio
public class DcEncoderActuator {
    OpMode opMode ;
    //public DcMotorEx motor ;
    public DcMotor motor ;
    public int encoderStartPosition = 0 ;

    // Configurable settings
    public double encoderResolution = 1 ; // Maps between encoder ticks and degrees
    public double motorPower = 1 ;
    public double encoderPoint = 20 ; // The degrees when proportional kicks in
    public double encoderPower = 0.1 ;
    public double manualPower = 1 ;
    public double speedOffset = 0.1 ;
    public boolean useBrake = true ;
    private int targetTicks = 0 ;
    private double previousTargetDegrees = 1 ;
    public double distance = 1 ;
    public double error = 0 ;
    public double targetSpeed ;

    public DcEncoderActuator( OpMode setOpMode, String motorName ) {
        opMode = setOpMode ;
        motor = opMode.hardwareMap.get( DcMotorEx.class, motorName ) ;
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        encoderStartPosition = motor.getCurrentPosition() ;  // Calibrate the encoder to zero the position
        //motor.setTargetPosition( (int)encoderStartPosition ) ;
        //motor.setMode( DcMotor.RunMode.RUN_TO_POSITION ) ;
    }
    public DcEncoderActuator( OpMode setOpMode, String motorName, double resolution ) {
        opMode = setOpMode ;
        motor = opMode.hardwareMap.get( DcMotorEx.class, motorName ) ;
        encoderResolution = resolution ;
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //encoderStartPosition = motor.getCurrentPosition() ;  // Calibrate the encoder to zero the position
        motor.setTargetPosition( (int)encoderStartPosition ) ;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0) ;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setMode( DcMotor.RunMode.RUN_TO_POSITION ) ;
    }

    public void rawTurn( double rawSpeed ) {
        // Scale is -1 to 1. Stop = 0
        motor.setPower( rawSpeed ) ;
        motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER ) ;
    }



    // Use simple RUN_TO_POSITION to use encoder to turn
    // Very significant OVERSHOOT = unusable. Tried MANY times :(
    public void setTargetEncoder( double targetDegrees ) {
        if (!useBrake) {
            error = targetDegrees - getPosition() ;
            //motor.setPower(clamp((Math.signum(error)*manualPower* motorPower),-1,1));
            motor.setPower(clamp((manualPower* motorPower*error/10),-1,1));
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (motor.getMode()!=DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition((int) (degreesToTicks(targetDegrees))) ;
            motor.setPower(motorPower*encoderPower) ;
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        }
        else {
            if (motor.getTargetPosition()!=(int) (degreesToTicks(targetDegrees)) ) {
                motor.setTargetPosition((int) (degreesToTicks(targetDegrees))) ;
            }
            motor.setPower(motorPower*encoderPower) ;
        }
    }

    //  ============ No proportional - full power ============================================================
    // WAY TO MUCH shaking after releasing manual power
    public void setTargetStraight( double targetDegrees ) {
        error = targetDegrees - getPosition() ;
        if (Math.abs(error)<encoderPoint) {
            motor.setPower(0) ;
        }
        // ======================= Manual input =======================
        else if (!useBrake) {
            //motor.setPower(clamp((Math.signum(error)*manualPower* motorPower),-1,1));
            motor.setPower(clamp((manualPower* motorPower*error/10),-1,1));
            //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //previousTargetDegrees = targetDegrees ;  // TODO: Need this???
        }
        // ================ If not the above, we have to ramp up and down =========================
        else {
            motor.setPower(Math.signum(error)*motorPower) ;
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
    }


    //  ==============================================================================
    public void setTargetSin( double targetDegrees ) {
        error = targetDegrees - getPosition() ;

        // ======================= Manual input =======================
        if (!useBrake) {
            //motor.setPower(clamp((Math.signum(error)*manualPower* motorPower),-1,1));
            motor.setPower(clamp((manualPower* motorPower*error/10),-1,1));
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            previousTargetDegrees = targetDegrees ;  // TODO: Need this???
        }
        else if (Math.abs(error) < encoderPoint) {
            motor.setPower(0) ;
        }// ================ If not the above, we have to ramp up and down =========================
        else {
            if (previousTargetDegrees != targetDegrees) {
                //distance = Math.abs(getPosition()-targetDegrees) ;  // TODO: this is new way of calculating distance
                distance = Math.abs(error) ;  // TODO: This should be the same
                previousTargetDegrees = targetDegrees ;
                motor.setPower(0) ;  // TODO: Consider using this
            }
            motor.setPower(clamp((Math.signum(error) * speedOffset) + (Math.sin(clamp(error * 2.8 / distance, -2.8, 2.8) ))* motorPower, -1, 1)) ;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //  ==============================================================================
    // Complex method, but seems to work well
    public void setTarget( double targetDegrees ) {
        error = targetDegrees - getPosition() ;

        // ======================= Manual input =======================
        if (!useBrake) {
            //motor.setPower(clamp((Math.signum(error)*manualPower* motorPower),-1,1));
            motor.setPower(clamp((manualPower* motorPower*error/10),-1,1));
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            previousTargetDegrees = targetDegrees ;  // TODO: Need this???
        }
        // ================= If we're close and we're braking (no manual input) ======================
        else if (Math.abs(error)<encoderPoint & useBrake) {
            if (!(motor.getMode()==DcMotor.RunMode.RUN_TO_POSITION & motor.getTargetPosition()==(int) (degreesToTicks(targetDegrees)))) {
                motor.setPower(0) ;
                try {
                    Thread.sleep(25);
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                }
                motor.setTargetPosition((int) (degreesToTicks(targetDegrees))) ;
                /*if (motor.getTargetPosition()!=degreesToTicks(targetDegrees)) {
                    motor.setTargetPosition((int) (degreesToTicks(targetDegrees)));
                }
                 */
                motor.setPower(motorPower*encoderPower) ;  // TODO: Try straight power.
                // motor.setPower(motorPower*encoderPower*error/encoderPoint) ;  // TODO: Try adding P to correction. May smooth the transition.
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
                //motor.setPower(motorPower) ;
                //previousTargetDegrees = targetDegrees ;  // TODO: Need this???
            }
            //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
            //else motor.setPower(motorPower*encoderPower*error/encoderPoint) ;  // TODO: Try adding P to correction. May smooth the transition.
            //else motor.setPower(motorPower*encoderPower) ;  // TODO: Try straight power.
        }
        // ================ If not the above, we have to ramp up and down =========================
        else {
            if (previousTargetDegrees != targetDegrees) {
                distance = Math.abs(getPosition()-targetDegrees) ;  // TODO: this is new way of calculating distance
                previousTargetDegrees = targetDegrees ;
                //motor.setPower(0) ;  // TODO: Consider using this
            }
            motor.setPower(clamp((Math.signum(error) * speedOffset) + (Math.sin(clamp(error * 3.0 / distance, -3.0, 3.0) * motorPower)), -1, 1)) ;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //========================================================================



    public void setSpeed( double speed ) { motorPower = speed ; }

    public double getPosition() { return ticksToDegrees( motor.getCurrentPosition() ) ; }  // Position in degrees

    public void setPosition( double newPositionDegrees ) {
        setTarget( newPositionDegrees ) ;
    }

    public void stop() { rawTurn(0) ; }

    private double ticksToDegrees(int measuredTicks) { return measuredTicks/encoderResolution ; }

    private int degreesToTicks(double rawDegrees) { return (int)( rawDegrees*encoderResolution ); }

    private double clamp(double input, double low, double high) {
        if (input>high) return high ;
        else if (input<low) return low ;
        else return input ;
    }
}
