package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

/** Team-editable Blue close field coordinates (inches / degrees). */
public class BlueClosePoses extends RobotPoses {
    public BlueClosePoses() {
        start = xyz(-65, -41.75, 0);
        lookAtOb = xyz(-23, -23, -195);
        openGatePos = xyz(-7, -72 + 6 + 5.25, 135);
        close1Shooting = xyz(-39, -39, -137);
        collect1Pre = xyz(-12, -31, -90);
        collect1Mid = xyz(-12, -22, -90);
        firstSpikeEnd = xyz(-12, -58, -90);
        strafePos = xyz(-17, -36, -90);
        collect2Mid = xyz(12, -25, -90);
        collect2Pre = xyz(12, -31, -90);
        secondSpikeEnd = xyz(12, -64, -90);
        collect3Pre = xyz(36, -31, -90);
        collect3PrePass = xyz(12, -45, -90);
        thirdSpikeEnd = xyz(36, -64, -90);
    }
}
