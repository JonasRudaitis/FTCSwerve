package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

public class VisionSubsystem extends SubsystemBase {
    public final String team;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    double lastDistance = 6.0;
    public VisionSubsystem (final HardwareMap hMap, String team) {
        this.team = team;
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setCameraPose(new Position(
                            DistanceUnit.INCH, 0, 0, 0, 0),
                        new YawPitchRollAngles(
                                AngleUnit.DEGREES,0, -90, 0, 0))
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())

                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    } // end of constructor
    private List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
    // Gets rotation of the red goal in relation to the camera
    public double getGoalYaw(){
        for (AprilTagDetection detection: getDetections()) {
            if (Objects.equals(team, "red")) {
                if (detection.id == 24) {
                    return detection.ftcPose.yaw;
                }
            }
            else {
                if (detection.id == 20) {
                    return detection.ftcPose.yaw;
                }
            }
        }
        return 0;
    }
    public double getRedGoalYaw() {
        for (AprilTagDetection detection: getDetections()) {
            if (detection.id == 24) {
                return detection.ftcPose.yaw;
            }
        }
        return 0;
    } // end of getRedGoalYaw
    // Gets rotation of the blue goal in relation to the camera
    public double getBlueGoalYaw() {
        for (AprilTagDetection detection: getDetections()) {
            if (detection.id == 20) {
                return detection.ftcPose.yaw;
            }
        }
        return 0;
    } // end of getBlueGoalYaw
    // gets distance of red goal in feet
    public double getGoalDistance() {
        for (AprilTagDetection detection: aprilTag.getDetections()) {
            if (Objects.equals(team, "red")) {
                if (detection.id == 24) {
                    lastDistance = Math.sqrt(detection.ftcPose.x * detection.ftcPose.x + detection.ftcPose.y * detection.ftcPose.y);
                    return lastDistance * 3.28084; // in feet
                }
            }
            else {
                if (detection.id == 20) {
                    lastDistance = Math.sqrt(detection.ftcPose.x * detection.ftcPose.x + detection.ftcPose.y * detection.ftcPose.y);
                    return lastDistance * 3.28084; // in feet
                }
            }
        }
        return -1;
    }
    public double getRedGoalDistance() {
        for (AprilTagDetection detection: aprilTag.getDetections()) {
            if (detection.id == 24) {
                double distance = Math.sqrt(detection.ftcPose.x*detection.ftcPose.x + detection.ftcPose.y*detection.ftcPose.y);
                return distance*3.28084; // in feet
            }
        }
        return 0;
    } // end of getRedGoalDistance
    // gets distance of blue goal in feet
    public double getBlueGoalDistance() {
        for (AprilTagDetection detection: aprilTag.getDetections()) {
            if (detection.id == 21) {
                double distance = Math.sqrt(detection.ftcPose.x*detection.ftcPose.x + detection.ftcPose.y*detection.ftcPose.y);
                return distance*3.28084; // in feet
            }
        }
        return 0;
    } // end of getBlueGoalDistance
    // returns string of motif order
    public String readObelisk() {
        for (AprilTagDetection detection: getDetections()) {
            switch (detection.id) {
                case 21:
                    return "GPP";
                case 22:
                    return "PGP";
                case 23:
                    return "PPG";
            }
        }
        return "";
    } // end of readObelisk
} // end of VisionSubsystem
