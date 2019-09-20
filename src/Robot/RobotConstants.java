package Robot;

import javafx.scene.paint.Color;

public class RobotConstants {

    // G values used for A* algorithm
    public static final int MOVE_COST = 1;
    public static final int TURN_COST = 3;
    public static final double INFINITE_COST = 10000000;
    public static final int CHECKSTEPS = 18;    // 4
    static final int CALIBRATE_AFTER = 2; //Calibrate After number of moves (for align_right)
    static final int TURN_AND_CALIBRATE = 2;

    //Maximum number of moves
    public static final int MAX_MOVE = 9;   //9

    //Target coverage
    public static final int TARGETED_COVERAGE = 97;

    // Best exploration timing for leaderboard in seconds
    public static final int BEST_EXP_TIMING = 999;

    // To be adjusted
    public static final int MOVE_STEPS = 1;
    static final long WAIT_TIME = 1000;    //Time waiting before retransmitting in milliseconds
    static final short CAMERA_RANGE = 4;
    public static final int STEP_PER_SECOND = 30;  // 30; // default large step per second to avoid any delay

    // Sensors default range (In grids)
    static final int SHORT_MIN = 1;
    static final int SHORT_MAX = 2;

    static final int LONG_MIN = 1;
    static final int LONG_MAX = 4;

    // Camera default range (In grids)
    static final int CAMERA_MIN = 1;
    static final int CAMERA_MAX = SHORT_MAX;     // to synchronize with shortmax

    static final double RIGHT_THRES = 0.5; //Threshold value or right sensor will calibrate once exceeded
    static final double RIGHT_DIS_THRES_CLOSE = 1.0;
    static final double RIGHT_DIS_THRES_FAR = 3.8;
    //Constants to render Robot
    //public static final Color ROBOT_BODY = Color.rgb(59, 89, 152, 1);
    public static final Color ROBOT_BODY = Color.RED;
    public static final Color ROBOT_OUTLINE = Color.BLACK;
    public static final Color ROBOT_DIRECTION = Color.FORESTGREEN;

    // Not to add delay in movement
    static final int MAX_STEPSPERSECOND = 1000;
    static final String[] SENSOR_ID = {"F1", "F2" , "F3" , "R1" , "R2" , "L1"};
    static int  NO_OF_SENSORS = SENSOR_ID.length;
}
