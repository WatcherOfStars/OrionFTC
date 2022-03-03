package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput_old;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener_old;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;


@TeleOp(name = "*ERASMUS TELEOP OLD*", group = "Erasmus")
@Config
@Disabled
public class ErasmusTeleop_old extends OpMode implements ControllerInputListener_old
{
   ////Dependencies////
   private ErasmusRobot robot;
   private ControllerInput_old controllerInput1;
   private ControllerInput_old controllerInput2;

   ////Variables////
   //Tweaking Vars
   public static double driveSpeed = 1;//used to change how fast robot drives
   public static double turnSpeed = -1;//used to change how fast robot turns

   public static double turnP = 0.005;
   public static double turnI = 0.0;
   public static double turnD = 0.01;

   private double speedMultiplier = 1;

   public static int payloadControllerNumber = 1;



   @Override
   public void init() {
      robot = new ErasmusRobot(this, true, true, false);
      robot.Init();

      controllerInput1 = new ControllerInput_old(gamepad1, 1);
      controllerInput1.addListener(this);
      controllerInput2 = new ControllerInput_old(gamepad2, 2);
      controllerInput2.addListener(this);

      //hardwareMap.dcMotor.get("FR").setDirection(DcMotorSimple.Direction.REVERSE);
      //hardwareMap.dcMotor.get("FL").setDirection(DcMotorSimple.Direction.REVERSE);

      telemetry.addData("Speed Multiplier", speedMultiplier);
      telemetry.update();


      msStuckDetectLoop = 15000;


      //set roadrunner speed modifiers
      if(robot.USE_NAVIGATOR){
      }
   }

   @Override
   public void start(){
      robot.Start();
      robot.chassis.ResetGyro();
      //if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
      //else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red
      robot.chassis.SetHeadlessMode(true);
   }

   @Override
   public void loop() {
      controllerInput1.Loop();
      controllerInput2.Loop();

      //KILL SWITCH FOR NAVIGATOR
      if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
         //robot.navigation.StopNavigator();
         if(robot.USE_PAYLOAD) robot.TurretArm().StopArmThread();
         if(robot.USE_PAYLOAD) robot.blinkinController.Lime();
      }

      //if(robot.navigation.IsThreadRunning()) return;

      if(robot.fieldSide == BaseRobot.FieldSide.BLUE){
         telemetry.addData("Alliance Side", "BLUE");
         if(robot.USE_PAYLOAD) robot.blinkinController.Blue();
      }
      else {
         telemetry.addData("Alliance Side", "RED");
         if(robot.USE_PAYLOAD) robot.blinkinController.Red();
      }

      robot.Update();

      //robot.navigation.PrintSensorTelemetry();

      //Manage driving
      robot.chassis.SetHeadingPID(turnP, turnI, turnD);
      //robot.chassis.DriveWithGamepad(controllerInput1, driveSpeed, turnSpeed, speedMultiplier);

      //print telemetry
      if(robot.USE_NAVIGATOR) {
         //control.GetOrion().PrintVuforiaTelemetry(0);
         //control.GetOrion().PrintTensorflowTelemetry();
      }

      telemetry.update();
   }

   @Override
   public void stop(){
      robot.Stop();
   }

   ////INPUT MAPPING////

   @Override
   public void APressed(double controllerNumber) {
      if(controllerNumber == 1) {
         if (speedMultiplier == 1) speedMultiplier = 0.5;
         else speedMultiplier = 1;
      }
      else if(controllerNumber == 2){
         //robot.navigation.StartCollecting(robot.turretArm.GetCurrentAutoTierRotation());
      }
   }

   @Override
   public void BPressed(double controllerNumber) {

      if(controllerNumber == 1) {
         //robot.navigation.StarGoToCollect();
      }

   }

   @Override
   public void XPressed(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.TurretArm().ReturnToHomeAndIntake(); //TODO: revert to resetAndIntake()
      }
   }

   @Override
   public void YPressed(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         //control.navigation.PlaceFreightLinear();
         //robot.navigation.StartGoToPlace();
      }
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.TurretArm().CycleIntakeState(1);
      }
   }

   @Override
   public void AHeld(double controllerNumber) {

   }

   @Override
   public void BHeld(double controllerNumber) {

   }

   @Override
   public void XHeld(double controllerNumber) {
   }

   @Override
   public void YHeld(double controllerNumber) {
   }

   @Override
   public void AReleased(double controllerNumber) {

   }

   @Override
   public void BReleased(double controllerNumber)  {
   }

   @Override
   public void XReleased(double controllerNumber) {
   }

   @Override
   public void YReleased(double controllerNumber) {

   }

   @Override
   public void LBPressed(double controllerNumber) {

   }

   @Override
   public void RBPressed(double controllerNumber) {

      //toggle between duck spinner states
        /*if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
            robot.navigation.SpinDucks(0.5,1);
        }
        else if(controllerNumber == 2 && robot.USE_PAYLOAD) robot.navigation.StartSpinDucks(10);*/
   }

   @Override
   public void LTPressed(double controllerNumber) {
   }

   @Override
   public void RTPressed(double controllerNumber) {
   }

   @Override
   public void LBHeld(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.Turret().SetPowerClamped(-1);
      }
   }

   @Override
   public void RBHeld(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.Turret().SetPowerClamped(1);
      }
   }

   @Override
   public void LTHeld(double controllerNumber) {
      if(controllerNumber == 1 && robot.USE_PAYLOAD){
         robot.Arm().SetPowerClamped(1);
      }
      if(controllerNumber == 2 && robot.USE_PAYLOAD){
         robot.Arm().SetPowerRaw(1);
      }
   }

   @Override
   public void RTHeld(double controllerNumber) {
      if(controllerNumber == 1 && robot.USE_PAYLOAD){
         robot.Arm().SetPowerClamped(-1);
      }
      if(controllerNumber == 2 && robot.USE_PAYLOAD){
         robot.Arm().SetPowerRaw(-1);
      }
   }

   @Override
   public void LBReleased(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.Turret().SetPowerClamped(0);
      }
   }

   @Override
   public void RBReleased(double controllerNumber) {
      if(controllerNumber == payloadControllerNumber && robot.USE_PAYLOAD){
         robot.Turret().SetPowerClamped(0);
      }
   }

   @Override
   public void LTReleased(double controllerNumber) {
      if(robot.USE_PAYLOAD){
         robot.Arm().SetPowerRaw(0);
      }
   }

   @Override
   public void RTReleased(double controllerNumber) {
      if(robot.USE_PAYLOAD){
         robot.Arm().SetPowerRaw(0);
      }
   }

   @Override
   public void DUpPressed(double controllerNumber) {
      if(robot.USE_PAYLOAD) {
         robot.TurretArm().AutoIntakeTierUp();
         robot.TurretArm().GoToAutoTier();
      }
   }

   @Override
   public void DDownPressed(double controllerNumber) {
      if(robot.USE_PAYLOAD) {
         robot.TurretArm().AutoIntakeTierDown();
         robot.TurretArm().GoToAutoTier();
      }
   }

   @Override
   public void DLeftPressed(double controllerNumber) {

   }

   @Override
   public void DRightPressed(double controllerNumber) {

   }

   @Override
   public void DUpHeld(double controllerNumber) {

   }

   @Override
   public void DDownHeld(double controllerNumber) {

   }

   @Override
   public void DLeftHeld(double controllerNumber) {

   }

   @Override
   public void DRightHeld(double controllerNumber) {

   }

   @Override
   public void DUpReleased(double controllerNumber) {

   }

   @Override
   public void DDownReleased(double controllerNumber) {

   }

   @Override
   public void DLeftReleased(double controllerNumber) {

   }

   @Override
   public void DRightReleased(double controllerNumber) {

   }

   @Override
   public void LJSPressed(double controllerNumber) {
      if(controllerNumber == 1) {
            /*robot.navigation.ToggleAllianceSide();
            if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.BLUE) robot.SetInputOffset(90); //90 is blue, -90 is red
            else if(robot.navigation.side == FreightFrenzyNavigation.AllianceSide.RED) robot.SetInputOffset(-90); //90 is blue, -90 is red*/
      }
      //if(controllerNumber == 2 && control.isUSE_PAYLOAD()) control.TurretArm().StartResetArm();
   }

   @Override
   public void RJSPressed(double controllerNumber) {
      if(controllerNumber == 1) {
         robot.chassis.ResetGyro();
      }
      if(controllerNumber == 2 && robot.USE_PAYLOAD) robot.Arm().ResetToZero();
   }

   @Override
   public void LJSHeld(double controllerNumber) {

   }

   @Override
   public void RJSHeld(double controllerNumber) {

   }

   @Override
   public void LJSReleased(double controllerNumber) {

   }

   @Override
   public void RJSReleased(double controllerNumber) {

   }
}
