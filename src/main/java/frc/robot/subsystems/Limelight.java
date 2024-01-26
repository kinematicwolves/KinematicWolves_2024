// package frc.robot.subsystems;

// import java.lang.annotation.Target;
// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.Iterator;
// import java.util.List;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
// import org.json.simple.parser.JSONParser;
// import org.json.simple.parser.ParseException;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.Camera.TargetType;

// public class Limelight extends SubsystemBase {

//     List<Target> targets;

//     private NetworkTable limelightTable;
//     private JSONParser jsonParser;
//     private final String DETECTOR_DEFAULT_JSON = "{\"Results\":{\"Classifier\":[],\"Detector\":[],\"Fiducial\":[],\"Retro\":[],\"pID\":0.0,\"tl\":0.0,\"ts\":0.0,\"v\":1}}";
    
//     private static Limelight instance = null;

//     private Field2d field;

//     public Limelight() {
//         limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//         jsonParser = new JSONParser();
//         targets = Arrays.asList();

//     }

//     public static Limelight getInstance() {
//         if (instance == null)
//             instance = new Limelight();
//         return instance;
//     }

//     public Transform2d getCameraToRobot() {
//         // 17.75 from front, 15.25 from left
//         // Translation2d disp = new Translation2d(
//         //     (Robot.bot.fullRobotLength/2 - Units.inchesToMeters(17.75)),
//         //     (Robot.bot.fullRobotWidth/2 - Units.inchesToMeters(15.25))
//         // );
//         return new Transform2d(new Translation2d(), new Rotation2d());
//     }

//     public boolean isTargetVisible() {
//         return getAllTargets().size() > 0;
//     }

//     public List<Target> getAllTargets() {
//         return this.targets;
//     }

//     public Target getPrimaryTarget() {
//         return isTargetVisible() ? getAllTargets().get(0) : null;
//     }


//     public void outputTelemetry() {
//         SmartDashboard.putNumber("Number of Detected AprilTags", getAllTargets().size());

//         }

//      @Override
//      public void periodic() {
//         getCameraToRobot();
//         isTargetVisible();
//         getAllTargets();
//         getPrimaryTarget();
//         outputTelemetry();
//         String jsonString = limelightTable.getEntry("json").getString(DETECTOR_DEFAULT_JSON);
//         List<Target> detectedTargets = new ArrayList<Target>();

//         try {
//             JSONObject llResults =  (JSONObject) ((JSONObject) jsonParser.parse(jsonString)).get("Results");
//             JSONArray aprilTags = (JSONArray) llResults.get("Fiducial");
            
//             if (!aprilTags.isEmpty()) {
//                 Iterator<JSONObject> iterator = aprilTags.iterator();
//                 while(iterator.hasNext()) {
//                     JSONObject tag = (JSONObject) iterator.next();
                   
//                     int tagID = Math.toIntExact((Long) tag.get("fID"));

//                     if (tagID > 8)
//                         continue;
                    
//                     JSONArray coords = (JSONArray) tag.get("t6t_cs");

//                     if (!coords.isEmpty()) {
//                         // camera space
//                         double x = (double) coords.get(0);
//                         double y = (double) coords.get(1);
//                         double z = (double) coords.get(2);
//                         double ry = (double) coords.get(4);
                      
//                         // robot space
//                         Pose3d targetPose = new Pose3d(z, -x, -y, new Rotation3d(0, 0, -ry));
//                         Pose2d robotRelativeTargetPose = targetPose.toPose2d().plus(getCameraToRobot());


                        
//                     }
//                 }
//             }

//         } catch (ParseException e) {
//             System.out.println("PARSING NETWORKTABLES JSON FAILED");
//             e.printStackTrace();
//         }

//         this.targets = detectedTargets;
//     }
// }
