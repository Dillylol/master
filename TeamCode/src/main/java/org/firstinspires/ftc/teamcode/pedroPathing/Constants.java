package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Drivetrain Physical Characteristics
    public static double MASS = 1.0; // kg, placeholder
    public static double WHEEL_RADIUS = 1.88976; // inches
    public static double GEAR_RATIO = 1.0;
    public static double TRACK_WIDTH = 15.5; // inches

    // Tuning constants
    public static double LATERAL_MULTIPLIER = 1.0;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}