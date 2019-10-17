package Robot;

import javafx.scene.paint.Color;

public class RobotConstants {

    // G values used for A* algorithm
    public static final int MOVE_COST = 1;
    public static final int TURN_COST = 3;
    public static final double INFINITE_COST = 10000000;
    public static final int CHECKSTEPS = 18;
    //Can remove
    public static final int CALIBRATE_AFTER = 4; //Calibrate After number of moves (for align_right)
    public static final int TURN_AND_CALIBRATE = 1000;

    //Maximum number of moves
    public static final int MAX_MOVE = 9;   //9

    // Best exploration timing for leaderboard in seconds
    public static final int BEST_EXP_TIMING = 1000;

    // To be adjusted
    public static final int MOVE_STEPS = 1;
    public static final long WAIT_TIME = 1000;     //Time waiting before retransmitting in milliseconds
    public static final int STEP_PER_SECOND = 30;  // Default step per second to avoid any delay

    // Sensors default range (In grids)
    public static final int SHORT_MIN = 1;
    public static final int SHORT_MAX = 2;

    public static final int LONG_MIN = 3;
    public static final int LONG_MAX = 4;

    // Camera default range (In grids)
    public static final int CAMERA_RANGE = 4;
    public static final int CAMERA_MIN = 1;
    public static final int CAMERA_MAX = SHORT_MAX;     // to synchronize with shortmax


    //Constants to render Robot
    public static final Color ROBOT_BODY = Color.RED;
    public static final Color ROBOT_OUTLINE = Color.BLACK;
    public static final Color ROBOT_DIRECTION = Color.FORESTGREEN;

    // Not to add delay in movement
    public static final String[] SENSOR_ID = {"F1", "F2" , "F3" , "R1" , "R2" , "L1"};
    public static int  NO_OF_SENSORS = SENSOR_ID.length;
}

