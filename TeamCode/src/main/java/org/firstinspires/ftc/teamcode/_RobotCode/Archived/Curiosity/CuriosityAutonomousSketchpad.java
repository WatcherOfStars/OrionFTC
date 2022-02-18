package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;

@Autonomous(name = "Curiosity Auto Sketchpad", group = "Curiosity")
@Config
@Disabled

public class CuriosityAutonomousSketchpad extends LinearOpMode
{
   CuriosityRobot robot;
   FreightFrenzyNavigation nav;
   protected FreightFrenzyNavigation.AllianceSide side = FreightFrenzyNavigation.AllianceSide.RED;

   @Override
   public void runOpMode() throws InterruptedException {
      robot = new CuriosityRobot(this, true, true, true);
      nav = robot.navigation;
      robot.Init();

      waitForStart();
      robot.Start();
      nav.side = side;

      robot.TurretArm().ReturnToHomeAndIntakeWithSensor();
      while (!isStopRequested()){
         robot.Update();
         telemetry.update();
      }
   }
}
