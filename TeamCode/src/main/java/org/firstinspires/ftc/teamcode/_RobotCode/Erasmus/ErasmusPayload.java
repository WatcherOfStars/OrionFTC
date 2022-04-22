package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

/*
Class for controlling an arm, turret, and roller intake
Sensors used:
   -Distance sensor on the intake to detect collection
   -Distance sensor facing downwards to level arm and detect ground
Motors used:
   -DC motors w/ encoders for turret and arm
   -DC motor for intake
*/

@Config
public class ErasmusPayload
{
   //References
   protected OpMode opMode;

   //Motors
   EncoderActuatorProfile armProfile;
   public EncoderActuator arm;
   //EncoderActuatorProfile turretProfile;
   //public EncoderActuator turret ;
   Servo intake ;
   //Servo turretServo ;

   //Sensors
   public DistanceSensor intakeSensor;
   public ColorSensor colorSensor ;
   // EA TODO - Add touch sensor

   //Configuration
   public static double intakeMultiplier = 1;
   public static double armIntakeDist = 4;  // TODO - Axe ??
   public static double whiteThreshold = 100 ;

   //arm positions for each of the levels
   public static double armIntakePos = 0;
   public static double armBottomPos = 0.1;
   public static double armMiddlePos = 0.2;
   public static double armTopPos = 0.34;
   public static double armCapPos = 0.34;

   //turret positions
   // ** Consider renaming these **
   double turretSharedPlacementPos = -0.4; //shared hub
   double turretTeamPlacementPos = -0.4; //team hub

   //States
   public int intakeState = 0;

   // Chopping Block ====================================================
   //levelling sensor
   double armSlowDistanceCM = 8;
   double armResetDistanceCM = 2;
   double armStorageLocation = 0.02;

   public enum Tier {COLLECT, BOTTOM, MIDDLE, TOP, CAP}
   public Tier currentAutoTier = Tier.MIDDLE;
   public Tier currentTier = Tier.COLLECT;

   public ErasmusPayload(OpMode setOpMode, EncoderActuatorProfile setArmProfile,
                          Servo setIntake,
                         DistanceSensor setIntakeDetector, boolean reverseIntake)
   {
      opMode = setOpMode;

      armProfile = setArmProfile;
      //turretProfile = setTurretProfile; // Not using DC motor right now, but using the encoder
      intake = setIntake;

      arm = new EncoderActuator(opMode, armProfile); // Positive position is up. Max around 0.25??
      //  Go over bar = 0.07 ??
      //  Intake = 0, 1 = 0.x, 2 = 0.y, 3 = 0.z, 4 (cap) = 0.c
      //turret = new EncoderActuator(opMode, turretProfile);
      //
      intakeSensor = setIntakeDetector;
      if(reverseIntake) intakeMultiplier = -1;
      else intakeMultiplier = 1;
   }

   // My proposed payload methods ===========================================
   // Consider passing these onto the composed objects
   public void armMoveRaw(double armSpeed) {
      arm.SetPowerClamped(armSpeed) ;
   }
   public void armMovePosition(int armPosition) {}
   public int armGetPosition() { return 0 ; }
   public void armSetZero() {}
   public void armStop() {
      arm.SetPowerRaw(0) ;
   }

   public void intakeIn() {
      intake.setPosition(0) ;
   }
   public void intakeOut() {
      intake.setPosition(1) ;
   }
   public boolean intakeGetFull() { return false ; }
   public void intakeStop() { intake.setPosition(0.5) ; }

   public void turretMovePosition() {}
   public void turretSetZero() {}
   public int turretGetPosition() { return 0 ; }
   public void turretStop() {}

   // Methods to save or graft ?? ===========================================================

   //MANAGE COLLECTION
   //what to do when the intake had collected a block
   protected void IntakeCollectAction() {}

   //sets the speed of the intake within a -1 to 1 range
   public void SetIntakeSpeed(double speed){
      double clampedSpeed = clamp(speed, -1,1);
      double servoSpeed = (clampedSpeed * 0.5) + 0.5;
      intake.setPosition(servoSpeed);
   }
   //Turns the intake on
   public void StartIntake(){
      SetIntakeSpeed(intakeMultiplier);
      intakeState = 1;
   }
   //Turns intake off
   public void StopIntake(){
      SetIntakeSpeed(0);
      intakeState = 2;
   }

   //Cycles between intake states: off, intaking, off, outaking
   public void CycleIntakeState(double intakeSpeed){
      opMode.telemetry.addLine("CYCLING INTAKE");
      if(intakeState == 0) SetIntakeSpeed(intakeSpeed);
      else if (intakeState == 1) SetIntakeSpeed(0);
      else if (intakeState == 2) SetIntakeSpeed(-intakeSpeed);
      else if (intakeState == 3) SetIntakeSpeed(0);
      intakeState ++;
      if(intakeState > 3) intakeState = 0;
   }

   //Returns a double of the arm's current tier rotation
   public double GetTierValue(Tier tier){
      if(tier == Tier.BOTTOM) return armBottomPos;
      else if(tier == Tier.MIDDLE) return armMiddlePos;
      else if(tier == Tier.TOP) return armTopPos;
      else if(tier == Tier.CAP) return armCapPos;
      else return armIntakePos;
   }

   //TIER MANAGEMENT
   //Moves the arm to specified tier
   public void GoToTier(Tier tier){
      //armThread.StopThread();
      arm.GoToPosition(GetTierValue(tier));
      currentTier = tier;
   }

   //UTILITY

   //Clamps a value between a max and a min value
   public static double clamp(double val, double min, double max) {
      return Math.max(min, Math.min(max, val));
   }


   // Chopping Block :) =============================================

   //Moves to current auto-intake tier
   public void GoToAutoTier(){
      GoToTier(currentAutoTier);
   }

   //Returns the arm to the base position and starts intaking without doing any reset or levelling
   public void ReturnToHomeAndIntake(){
      StartIntake();
      GoToTier(Tier.COLLECT);
      //turret.GoToPosition(0);
   }

   //Resets the arm and turns the intake on
   public void ResetArmAndIntake(){
      StartIntake();
      //armThread.StartResetArm();
   }

   //Sets the arm's auto position up a tier
   public void AutoIntakeTierUp(){
      if(currentAutoTier == Tier.BOTTOM) currentAutoTier = Tier.MIDDLE;
      else if(currentAutoTier == Tier.MIDDLE) currentAutoTier = Tier.TOP;
      else if(currentAutoTier == Tier.TOP) currentAutoTier = Tier.CAP;
   }

   //Sets the arm auto position down a tier
   public void AutoIntakeTierDown(){
      if(currentAutoTier == Tier.MIDDLE) currentAutoTier = Tier.BOTTOM;
      else if(currentAutoTier == Tier.TOP) currentAutoTier = Tier.MIDDLE;
      else if(currentAutoTier == Tier.CAP) currentAutoTier = Tier.TOP;
   }

   //Halts the thread of the arm
   public void StopArmThread(){
      //armThread.StopThread();
   }

   //Returns the value in CMs of the intake sensor
   public double GetIntakeDistanceCM() {return intakeSensor.getDistance(DistanceUnit.CM);}
}
