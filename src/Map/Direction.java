package Map;

public enum Direction {

    // Anti-clockwise
    UP, LEFT, DOWN, RIGHT;

    //Get the anti-clockwise direction of robot's current direction
    public static Direction getAntiClockwise(Direction curDirection) {
        return values()[(curDirection.ordinal() + 1) % values().length];
    }

    //Get the clockwise direction of robot's current direction
    public static Direction getClockwise(Direction curDirection) {
        return values()[(curDirection.ordinal() + values().length - 1) % values().length];
    }

    //Get opposite direction of robot's current direction
    public static Direction getOpposite(Direction curDirection) {
        return values()[(curDirection.ordinal() + 2) % values().length];
    }

}
