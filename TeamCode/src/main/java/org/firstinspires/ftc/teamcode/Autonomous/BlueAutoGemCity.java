package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="AutonomousGemCity", group="Motion")
public class BlueAutoGemCity extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
       // robot.lowerGripper(100);
        robot.gripperOpen();
        robot.gripperOpen();
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

    //    robot.wait(500L,this);
        robot.raiseGripper(670,0.7);    //power was at -1 for first 2-tournaments.  It sometimes sprang out too fast at init. rest of program should be power=1  was 730 for 2nd tourney.

        robot.gripperClose();
        robot.wait(500L,this);
        robot.raiseGripper(350, 1);

        //robot.shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Set shooting speed for the rings our robot starts with, drive to the starter stack, and shoot.
        robot.flywheel(true, 0.77);  //;robot.shooting.setPower(0.77); // was 0.75
        robot.goDistanceAcceleration(65, 0.8, false, 5, 50, this);
        robot.storage(true, this);
        robot.wait(300, this);

            // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinderDistance(this);
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, try one more time
            char tryAgain = robot.ringFinderDistance(this);  // If this fails it will take C path
            if (tryAgain == 'E') {
                ringAt = 'C';
            }
            else {
                ringAt = tryAgain;
            }
        }

        // Gets us to the target zone
        if (ringAt == 'A') {
            // Gets us to the target zone
            robot.flywheel(true, 0.80);
            //robot.storage(true, this);
            robot.goDistanceAcceleration(62,0.8,false,5,50,this); // back ramp was 25 2-20-21
                for (int i = 0; i<3; i++) {                             // added extra shot in case last ring is stuck.
                    robot.pushRing(0.4, this);
                    robot.flywheel(true, 0.8); // increase power for second and third shot
                    robot.wait(400, this); // timeout was 1200 fir first tourney
                }
            robot.flywheel(false, 0.0);

            robot.GoDistanceCM2(32, .7, false, this); //distance was 29 2-22-21 added extra
            robot.strafeDistanceCM2(-21, .7,false, this); // distance was -26 for 2nd tourney is -21 for gem city
            // drop wobble stick here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(400L,this);
            robot.GoDistanceCM2(-7,0.3,false,this);
            robot.lowerGripper(950);
            robot.RotateDEG(149, 0.7, this);        // rotation was 146 at 0.7 power is 149  2/22/21 150 for GEM city
            robot.goDistanceAcceleration(92, 0.9, false, 5, 70, this);// distance was 103 is 101 2-22-21 is 95 for gem city with angle 0f 149 and strafe -26
            // robot.wobbleFind(35, 0.2, 40, this);
            robot.wobbleFind2(45, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            int travelDist = ProgrammingFrame.wobble.travelDist;
                if (success) {
                    robot.gripperOpen();
                    robot.raiseGripper(750,1);
                    robot.GoDistanceCM2(travelDist, .15, false, this);
                    robot.gripperClose();
                    robot.wait(400, this);
                    robot.raiseGripper(400,1);
                }
            robot.RotateDEG((180 - wobbleDegrees) + 13, .7, this);
            robot.goDistanceAcceleration(85, .9, false, 5, 75, this);  // distance was 90 2-22/21  was 90 for second tourney
            robot.lowerGripper(100);
            robot.gripperOpen();
            robot.wait(400, this);
            //robot.GoDistanceCM2(-20, 30, false, this);
            robot.lowerGripper(200);
            //  robot.gripperClose();
            robot.GoDistanceCM2(-10,0.5,false,this);
            robot.RotateDEG(50,0.7,this);
            robot.GoDistanceCM2(35, .8,false, this);  // navigate to white line
            robot.lowerGripper(800);
            robot.storage(false,this);

            robot.gripperClose();
            robot.wait(200L,this);  //allow time for servo to finish closing grip before terminating
        }   // end drop zone A

        else if (ringAt == 'B') {

            robot.flywheel(true, 0.80);
            //robot.storage(true, this);
            robot.goDistanceAcceleration(62,0.8,false,5,50,this); // back ramp was 25 2-20-21
                for (int i = 0; i<3; i++) {                             // added extra shot in case last ring is stuck.
                    robot.pushRing(0.4, this);
                    //robot.flywheel(true, 0.80); // increase power for second and third shot
                    robot.wait(400, this); // timeout was 1200 fir first tourney
                }
            robot.flywheel(false, 0.0);

            robot.goDistanceAcceleration(125, 0.8, false,8, 75, this);   // was 127 2-24-21, added extra 20 cm to get ring out of the way.  Wobble was landing on top of ring 1-27-2021
            robot.GoDistanceCM2(-29, -0.4, false, this);  //  back up and drop wobble away from ring
            robot.strafeDistanceCM2(29, 0.5,false, this);
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(500L,this);
            robot.lowerGripper(950);
            robot.GoDistanceCM2(-10, .3, false, this);
            robot.RotateDEG(175, 0.6, this); // was 171
            robot.goDistanceAcceleration(130, 0.9, false, 5, 75, this);  //distance was 135 2-22-21
            // robot.wobbleFind(35, 0.2, 40, this);
            robot.wobbleFind2(45, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            int travelDist = ProgrammingFrame.wobble.travelDist;
                if (success) {
                    robot.gripperOpen();
                    robot.raiseGripper(750,1);
                    robot.GoDistanceCM2(travelDist, .15, false, this);
                    robot.gripperClose();
                    robot.wait(500, this);
                    robot.raiseGripper(400,1);
                }
           // robot.gripperOpen();
           // robot.raiseGripper(750,1);
            //robot.GoDistanceCM2(9, .2, false, this);
            //robot.gripperClose();
           // robot.wait(500, this);
           // robot.raiseGripper(400,1);
            //robot.GoDistanceCM2(-30, -.7, false,this);
            robot.RotateDEG((180 - wobbleDegrees) + 5 , 0.6, this);  // degrees 4.5 would be best  2-22-21
            robot.goDistanceAcceleration(130, .9, false, 5, 75, this);

            robot.lowerGripper(100);
            robot.gripperOpen();    //drop wobble
            robot.wait(400,this);
           // robot.lowerGripper(1000);
            robot.lowerGripper(900);
            robot.storage(false,this);

            robot.gripperClose();
            robot.wait(200L,this);  //allow time for servo to finish closing grip before terminating
        }// end drop zone B

        else {   // ring at C
                for (int i = 1; i <= 3; i++) {
                    robot.pushRing(0.4, this);
                    //robot.shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.flywheel(true, 0.76);//robot.shooting.setPower(0.76);
                    //robot.flywheel(true, 0.8); // increase power for second and third shot
                    robot.wait(1000, this); // timeout was 1200 fir first tourney
                }
            robot.flywheel(false, 0.0);;

            //Strafe to align with the starter stack and pick up the first two starter stack rings.
            robot.storageServo.setPosition(1);  //Lower storage bin
            robot.strafeDistanceCM2(23, 0.2, false, this);
            robot.intake(ProgrammingFrame.States.Forwards);
            robot.GoDistanceCM2(20, 0.3, false, this);
            //sleep(800L); //Wait for the rings to completely intake.

            //Set the shooting speed for the first two starter stack rings, navigate to the shooting position, and shoot.
            robot.flywheel(true ,0.79);//robot.shooting.setPower(0.79);  // was 0.8
            robot.strafeDistanceCM2(-30, .2, false, this);
            robot.intake(ProgrammingFrame.States.Off);
            robot.storage(true, this);;
            sleep(200L);
            robot.storage(false, this);;
            sleep(200L);
            robot.storage(true, this);;

                for (int i = 1; i <= 2; i++) {
                    robot.pushRing(0.4, this);
                    //robot.flywheel(true, 0.79); // increase power for second and third shot
                    robot.wait(400, this); // timeout was 1200 fir first tourney
                }
            robot.flywheel(false, 0.0);;
            robot.storage(false, this);;
            //Strafe to pick up the last two starter stack rings and pick them up.
            robot.strafeDistanceCM2(30, 0.2, false, this);
            robot.intake(ProgrammingFrame.States.Forwards);
            robot.GoDistanceCM2(35, 0.2, false, this);
            //sleep(2000L);
            //robot.intake(ProgrammingFrame.States.Off);
            //Set the shooting power for the last two starter stack rings, navigate to the shooting position, and shoot.
            robot.shooting.setPower(0.8);
            //robot.storageServo.setPosition(0);
            robot.strafeDistanceCM2(-30, .2, false, this);
            robot.storage(true, this);;
            sleep(200L);
            robot.storage(false, this);;
            sleep(200L);
            robot.storage(true, this);;
            sleep(300);
            robot.intake(ProgrammingFrame.States.Off);
                for (int i = 1; i <= 3; i++) {
                    robot.pushRing(0.4, this);
                    //robot.flywheel(true, 0.8); // increase power for second and third shot
                    robot.wait(400, this); // timeout was 1200 fir first tourney
                }
            robot.shooting.setPower(0);

            robot.goDistanceAcceleration(138, 0.8, false, 8, 80, this); //distance was 149 3-23-21
            robot.strafeDistanceCM2(-5
                    , 0.6, false, this); //distance was -11 3-23-21
            //robot.GoDistanceCM2(-28, -0.4, false, this);  //  back up and drop wobble away from ring

            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(300L, this);
            robot.storageServo.setPosition(1);
            robot.goDistanceAcceleration(-70, 0.8, false, 8, 90, this);
            robot.gripperClose();
            robot.lowerGripper(950);
        }// end drop zone c

       // while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

    }       // end runOpmode

}
