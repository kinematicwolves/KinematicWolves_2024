package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TagLocation extends Translation2d {
    private int id;
    private boolean rotate180;

    public TagLocation(int id, double x, double y) {
        super(x, y);
        setId(id);
    }

    public TagLocation(int id, double d, Rotation2d angle) {
        super(d, angle);
        setId(id);
    }

    public TagLocation(int id, Translation2d translation) {
        this(id, translation.getX() , translation.getY());
    }
    
    public TagLocation plusTag(Translation2d other) {
        Translation2d res =  ((Translation2d) this).plus((Translation2d) other);
        return new TagLocation(this.id, res);
    }

    public TagLocation setId(int id) {
        this.id = Math.max(Math.min(id, 8), 1);
        this.rotate180 = this.id <= 4;
        return this;
    }
}