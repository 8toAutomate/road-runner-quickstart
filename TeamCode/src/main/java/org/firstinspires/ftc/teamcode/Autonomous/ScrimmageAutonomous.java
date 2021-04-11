package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="ScrimmageAutonomous", group="Motion")
@Disabled
public class ScrimmageAutonomous extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.

    ProgrammingFrame robot = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;
    enum States {
        Forwards, Backwards, Off, On
    }

    States ringPusher = States.Backwards;
    States flywheel = States.Off;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.GoDistanceCM2(135, 0.3, false, this);
        robot.storageServo.setPosition(0);
        robot.shooting.setPower(.8);
        robot.ringPusher.scaleRange(0, 1.0);
        initialSH = getRuntime();
        while (getRuntime() - initialSH < 4.0) {}

         for (int i=1; i<=4; i++) {
             initialSH = getRuntime();
             robot.ringPusher.setPosition(1.0);
            while (getRuntime() - initialSH < 1) {}
            robot.ringPusher.setPosition(0);
            initialSH = getRuntime();
            while (getRuntime() - initialSH < 1.5) {}
        }
         robot.shooting.setPower(0);
         robot.GoDistanceCM2(25, .7, false, this);
    }

}