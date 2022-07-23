package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

import java.util.ArrayList;
import java.util.List;

public class ControllerInputDummy extends ControllerInput
{
    //ENUMS
    public enum Button {X,Y,A,B,RT,LT,RB,LB,DUP,DDOWN,DLEFT,DRIGHT,LJS,RJS}

    //VARIABLES
    //misc.
    private double TriggerThreshold = 0.1;

    //buttons
    private boolean ADown = false;
    private boolean BDown = false;
    private boolean XDown = false;
    private boolean YDown = false;
    private boolean LBDown = false;
    private boolean RBDown = false;
    private boolean LTDown = false;
    private boolean RTDown = false;
    private boolean DUpDown = false;
    private boolean DDownDown = false;
    private boolean DLeftDown = false;
    private boolean DRightDown = false;
    private boolean RJSDown = false;
    private boolean LJSDown = false;

    public ControllerInputDummy(Gamepad setGamepad, int setID){
        super(setGamepad, setID) ;  // We do this to satisfy inheritance
    }

    //GETTERS
    public double GetLJSX()  { return 0 ; }
    public double GetLJSY()  { return 0 ; }
    public double GetRJSX()  { return 0 ; }
    public double GetRJSY()  { return 0 ; }
    public double CalculateLJSAngle()  { return 0 ; }
    public double CalculateLJSMag()  { return 0 ; }

    public void Loop(){
        // Do Nothing
    }
}
