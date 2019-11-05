package Robot;

import java.awt.*;

import Map.*;

public class Sensor {
    /**
     * ID as an identifier to each sensor
     * MinRange for minimum range of the sensor to detect obstacle accurately
     * MaxRange for maximum range of the sensor to detect obstacle accurately
     * Pos for position of sensor
     * Direction for direction that sensor is facing
     */
    private String id;
    private int minRange;
    private int maxRange;

    private Point pos;
    private Direction sensorDir;

    Sensor(String id, int minRange, int maxRange, int sensorPosRow, int sensorPosCol, Direction sensorDir) {
        this.id = id;
        this.minRange = minRange;
        this.maxRange = maxRange;
        this.pos = new Point(sensorPosCol, sensorPosRow);
        this.sensorDir = sensorDir;
    }

    //Getters and setters of attributes

    public String getId() {
        return id;
    }

    int getMinRange() {
        return minRange;
    }

    int getMaxRange() {
        return maxRange;
    }

    Point getPos() {
        return pos;
    }

    public int getRow() {
        return pos.y;
    }

    public int getCol() {
        return pos.x;
    }

    void setPos(int row, int col) {
        this.pos.setLocation(col, row);
    }

    Direction getSensorDir() {
        return sensorDir;
    }

    void setSensorDir(Direction sensorDir) {
        this.sensorDir = sensorDir;
    }

    /**
     * Converts sensor id, position and direction to string format
     */
    @Override
    public String toString() {
        return String.format("Sensor %s at %s facing %s\n", id, pos.toString(), sensorDir.toString());
    }


    //Sensor detect if obstacle is a cell in simulation
     int detect(Map map) {

        for (int cur = minRange; cur <= maxRange; cur++) {

            switch (sensorDir) {
                case UP:
                    //Return value if sensor detects border or obstacle
                    if(this.getId()== "L1"){
                        if(pos.y + cur > MapConstants.MAP_HEIGHT){
                            break;
                        }
                    }
                    if (pos.y + cur == MapConstants.MAP_HEIGHT)
                        return cur;
                    else if (map.getCell(pos.y + cur, pos.x).isObstacle())
                        return cur;
                    break;
                case RIGHT:
                    if(this.getId()== "L1"){
                        if(pos.x + cur > MapConstants.MAP_WIDTH){
                            break;
                        }
                    }
                    if (pos.x + cur == MapConstants.MAP_WIDTH)
                        return cur;
                    else if (map.getCell(pos.y, pos.x + cur).isObstacle())
                        return cur;
                    break;
                case DOWN:
                    if(this.getId()== "L1"){
                        if(pos.y - cur < -1){
                            break;
                        }
                    }
                    if (pos.y - cur == -1)
                        return cur;
                    else if (map.getCell(pos.y - cur, pos.x).isObstacle())
                        return cur;
                    break;
                case LEFT:
                    if(this.getId()== "L1"){
                        if(pos.x - cur < -1){
                            break;
                        }
                    }
                    if (pos.x - cur == -1)
                        return cur;
                    else if (map.getCell(pos.y, pos.x - cur).isObstacle())
                        return cur;
                    break;
            }
        }
        return -1;
    }
}