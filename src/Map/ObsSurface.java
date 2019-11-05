package Map;

import java.awt.*;
import java.util.Collection;
import java.util.Collections;

public class ObsSurface {

    private Point pos;
    private Point targetPos;

    private Direction targetDir;
    private Direction surface;

    public ObsSurface(Point pos, Direction surface) {
        this.pos = pos;
        this.targetPos = pos;
        this.surface = surface;
        this.targetDir = surface;
    }

    public ObsSurface(Point pos, Point targetPos, Direction surface, Direction targetDir) {
        this.pos = pos;
        this.targetPos = targetPos;
        this.surface = surface;
        this.targetDir = targetDir;
    }

    @Override
    public String toString() {
        return String.format("%d|%d|%s", this.pos.y, this.pos.x, this.surface.toString());   // row|col|surface
    }

    public Point getPos() {
        return pos;
    }

    public Direction getSurface() {
        return surface;
    }

    public Point getTargetPos() {
        return targetPos;
    }

    public Direction getTargetDir() {
        return targetDir;
    }

}
