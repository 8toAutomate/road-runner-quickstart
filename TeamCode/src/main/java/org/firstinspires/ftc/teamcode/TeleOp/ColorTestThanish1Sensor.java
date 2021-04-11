/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;


// @TeleOp(name = "ColorTestThanish1Sensor", group = "Sensor")
@Disabled
public class ColorTestThanish1Sensor extends OpMode {

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  ProgrammingFrame robot   = new ProgrammingFrame();

  float gain = 2;

  final float[] hsvValues = new float[3];

  @Override public void init() {

    robot.init(hardwareMap, this);
    if (robot.colorSensor1 instanceof SwitchableLight)
    {
      ((SwitchableLight)robot.colorSensor1).enableLight(true);
    }

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");

  }


  public void loop() {

    telemetry.addData("Gain", gain);

    robot.colorSensor1.setGain(gain);


    NormalizedRGBA colors1 = robot.colorSensor1.getNormalizedColors();

    Color.colorToHSV(colors1.toColor(), hsvValues);


    float red1 = colors1.red;
    float green1 = colors1.green;
    float blue1 = colors1.blue;


    telemetry.addLine();
    telemetry.addData("red1", red1);
    telemetry.addData("green1", green1);
    telemetry.addData("blue1", blue1);
    telemetry.addData("Hue: ", hsvValues);

    telemetry.update();

  }
}


