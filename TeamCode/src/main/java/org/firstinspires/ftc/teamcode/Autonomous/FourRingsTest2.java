package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="FourRingsTest2", group="Motion")
public class FourRingsTest2 extends LinearOpMode {
    ProgrammingFrame robot = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;
    char ringAt;
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        //robot.lowerGripper(100);
        robot.gripperOpen();
        robot.gripperOpen();

        waitForStart();
        robot.raiseGripper(730,0.8);
        robot.gripperClose();
        robot.wait(500L, this);
        robot.raiseGripper(350,1);
        robot.shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Set shooting speed for the rings our robot starts with, drive to the starter stack, and shoot.
        robot.shooting.setPower(0.77); // was 0.75
        robot.goDistanceAcceleration(65, 0.8, false, 5, 50, this);
        robot.storageServo.setPosition(0);
        robot.wait(300, this);
        ringAt = robot.ringFinderDistance(this);
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, try one more time
            char tryAgain = robot.ringFinderDistance(this);  // If this fails it will take C path
            if (tryAgain == 'E') {
                ringAt = 'C';
            } else {
                ringAt = tryAgain;
            }
        }

        if (ringAt == 'C') {

        for (int i = 1; i <= 3; i++) {
            robot.pushRing(0.4, this);
            robot.shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.shooting.setPower(0.76);
            //robot.flywheel(true, 0.8); // increase power for second and third shot
            robot.wait(1000, this); // timeout was 1200 fir first tourney
        }
        robot.shooting.setPower(0);

        //Strafe to align with the starter stack and pick up the first two starter stack rings.
        robot.storageServo.setPosition(1);  //Lower storage bin
        robot.strafeDistanceCM2(23, 0.2, false, this);
        robot.intake(ProgrammingFrame.States.Forwards);
        robot.GoDistanceCM2(20, 0.3, false, this);
        //sleep(800L); //Wait for the rings to completely intake.

        //Set the shooting speed for the first two starter stack rings, navigate to the shooting position, and shoot.
        robot.shooting.setPower(0.79);  // was 0.8
        robot.strafeDistanceCM2(-30, .2, false, this);
        robot.intake(ProgrammingFrame.States.Off);
        robot.storageServo.setPosition(0);
        sleep(200L);
        robot.storageServo.setPosition(1);
        sleep(200L);
        robot.storageServo.setPosition(0);

        for (int i = 1; i <= 2; i++) {
            robot.pushRing(0.4, this);
            //robot.flywheel(true, 0.79); // increase power for second and third shot
            robot.wait(400, this); // timeout was 1200 fir first tourney
        }
        robot.shooting.setPower(0);
        robot.storageServo.setPosition(1);
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
        robot.storageServo.setPosition(0);
        sleep(200L);
        robot.storageServo.setPosition(1);
        sleep(200L);
        robot.storageServo.setPosition(0);
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
        robot.goDistanceAcceleration(-75, 0.8, false, 8, 90, this);
        robot.gripperClose();
        robot.lowerGripper(950);
         }  // end iF Ring at "C"
    }   // end Run opmode
}   // end of Autonmous
