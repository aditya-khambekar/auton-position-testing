package frc.lib.fieldviewergui;

import javax.imageio.ImageIO;
import javax.swing.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.stream.IntStream;

public class Field extends JPanel implements MouseMotionListener {
    private final double imperialWidth = 317; //26 ft 5 inches wide
    private final double imperialLength = 690.875;// 57 fft 6 7/8in wide
    private final double inchesToMeters = 0.0254;
    //314 x689

    BufferedImage fieldImage = getFieldImage();
    //double mousex, mousey = 0;
    int mousex, mousey = 0;
    double fieldMinimumX = 0.13377321603 * getWidth();
    double fieldMaximumX = (1 - 0.13377321603) * getWidth();
    double fieldMinimumY = 0.14634888438 * getHeight();
    double fieldMaximumY = (1 - 0.14634888438) * getHeight();

    HashSet<Pose2d> poses = new HashSet<>();

    private static BufferedImage getFieldImage() {
        try {
            return ImageIO.read(new File("src\\main\\java\\frc\\lib\\fieldviewergui\\2025 REEFSCAPE Transparent Background.png"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public Field(){
        super();
        addMouseMotionListener(this);
        setPreferredSize(new Dimension(943, 448));

        Arrays.stream(FieldConstants.Reef.centerFaces).forEach(this::addPose);
        addPose(FieldConstants.Processor.centerFace);
        addPose(FieldConstants.Reef.center);
        addPose(FieldConstants.Barge.closeCage);
        addPose(FieldConstants.Barge.middleCage);
        addPose(FieldConstants.Barge.farCage);
        addPose(FieldConstants.CoralStation.leftCenterFace);
        addPose(FieldConstants.CoralStation.rightCenterFace);
    }

    @Override
    public void paint(Graphics g){
        super.paint(g);
        this.update();
        g.setColor(Color.DARK_GRAY);
        g.fillRect(0, 0, getWidth(), getHeight());
        g.setColor(Color.GRAY);
        g.fillRect((int) fieldMinimumX, (int) fieldMinimumY, (int) (fieldMaximumX-fieldMinimumX), (int) (fieldMaximumY-fieldMinimumY));
        g.drawImage(fieldImage, 0, 0, getWidth(), getHeight(), this);
        g.setColor(Color.BLACK);
        g.drawRect((int) fieldMinimumX, (int) fieldMinimumY, (int) (fieldMaximumX-fieldMinimumX), (int) (fieldMaximumY-fieldMinimumY));
        g.setColor(new Color(255, 215, 0));
        if (fieldMinimumX < mousex
            && mousex < fieldMaximumX
        && fieldMinimumY < mousey
            && mousey < fieldMaximumY
        ) {
            g.drawLine(mousex, 0, mousex, getHeight());
            g.drawLine(0, mousey, getWidth(), mousey);
            String s = "(" +
                    getXValue() +
                    ", " +
                    getYValue() +
                    ")";
            g.drawString(s, mousex + 10, mousey - 10);
        }
        //maybe this will work
        g.drawRect(mousex-15,mousey-15,30,30);

        g.setColor(Color.RED);
        for(Pose2d pose:poses){
            var sides = generateRotatedSquare(Units.metersToInches(pose.getX()), Units.metersToInches(pose.getY()), 30, pose.getRotation().getDegrees());
            for(Point[] side: sides){
                var p0 = toGuiCoordinate((side[0]));
                var p1 = toGuiCoordinate((side[1]));
                g.drawLine(p0.x, p0.y, p1.x, p1.y);
            }
            var forwardLine = sides[1];
            var backwardLine = sides[3];
            Point midPoint = toGuiCoordinate(new Point((int)((forwardLine[0].x + forwardLine[1].x))/2,
                    (int)((forwardLine[0].y + forwardLine[1].y)/2)));
            Point point1 = toGuiCoordinate(backwardLine[0]);
            Point point2 = toGuiCoordinate(backwardLine[1]);
            g.drawLine(point1.x, point1.y, midPoint.x, midPoint.y);
            g.drawLine(point2.x, point2.y, midPoint.x, midPoint.y);
        }
    }

    public static Point[][] generateRotatedSquare(double centerX, double centerY, double sideLength, double rotationAngle) {
        double angleRadians = Math.toRadians(rotationAngle);

        Point[] corners = new Point[4];

        corners[0] = new Point((int) (-sideLength/2), (int) (sideLength/2));
        corners[1] = new Point((int) (sideLength/2), (int) (sideLength/2));
        corners[2] = new Point((int) (sideLength/2), (int) (-sideLength/2));
        corners[3] = new Point((int) (-sideLength/2), (int) (-sideLength/2));

        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        IntStream.range(0, 4).forEach(p -> {
            corners[p] = new Point((int) (corners[p].x * cos - corners[p].y * sin), (int) (corners[p].x * sin + corners[p].y * cos));
            corners[p] = new Point((int) (corners[p].x + centerX), (int) (corners[p].y + centerY));
        });
        var ret = new Point[4][2];
        ret[0] = new Point[]{corners[0], corners[1]};
        ret[1] = new Point[]{corners[1], corners[2]};
        ret[2] = new Point[]{corners[2], corners[3]};
        ret[3] = new Point[]{corners[3], corners[0]};

        return ret;
    }

    private double getYValue() {
        return toFieldCoordinate(new Point(0, mousey)).y;
    }

    private double getXValue() {
        return toFieldCoordinate(new Point(mousex, 0)).x;
    }

    @Override
    public void mouseDragged(MouseEvent e) {

    }

    @Override
    public void mouseMoved(MouseEvent e) {
        mousex = e.getX();
        mousey = e.getY();
        repaint();
    }

    void update(){
        fieldMinimumX = 0.13377321603 * getWidth();
        fieldMaximumX = (1 - 0.13377321603) * getWidth();
        fieldMinimumY = 0.14634888438 * getHeight();
        fieldMaximumY = (1 - 0.14634888438) * getHeight();
        App.getInstance().update();
    }

    Point toFieldCoordinate(Point guiCoordinate){
        var x = guiCoordinate.x;
        var y = guiCoordinate.y;

        x = (int) ((x - fieldMinimumX) * 957.065/App.getInstance().getWidth());
        y = (int) ((y - fieldMinimumY) * 484.34/App.getInstance().getHeight());
        y = 317 - y;
        return new Point(x, y);
    }

    Point toGuiCoordinate(Point fieldCoordinate) {
        var x = fieldCoordinate.x;
        var y = fieldCoordinate.y;

        y = 317 - y;

        x = (int) (((x / 957.065) * App.getInstance().getWidth()) + fieldMinimumX);
        y = (int) (((y / 484.34) * App.getInstance().getHeight()) + fieldMinimumY);

        return new Point(x, y);
    }

    Point MetersToInches(Point p){
        var x = Units.metersToInches(p.x);
        var y = Units.metersToInches(p.y);
        return new Point((int)x, (int)y);
    }

    Point InchesToMeters(Point p){
        var x = Units.inchesToMeters(p.x);
        var y = Units.inchesToMeters(p.y);
        return new Point((int)x, (int)y);
    }

    void addPose(Pose2d pose){
        poses.add(pose);
    }

    void addPose(Translation2d translation){
        poses.add(new Pose2d(translation, Rotation2d.kZero));
    }
}

