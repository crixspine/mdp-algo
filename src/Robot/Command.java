package Robot;

public enum Command {

    FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, SEND_SENSORS, TAKE_IMG,  ALIGN_FRONT, ALIGN_RIGHT, INITIAL_CALIBRATE, ALIGN_FRONT1;
//
////    public enum AndroidMove {
//        forward, back, left, right
//    }

    public enum ArduinoMove {
        W, S, A, D, K, I, O, P, T, F;
    }
}
