package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import org.json.simple.parser.ParseException;

public class AutoCreator {
    private enum StartEndPoint {
        TRENCH_START_LEFT("Trench Start Left"),
        TRENCH_START_RIGHT("Trench Start Right"),
        TRENCH_RIGHT("Trench Right"),
        TRENCH_LEFT("Trench Left"),
        CLIMB_RIGHT("Climb Right"),
        CLIMB_LEFT("Climb Left"),
        DUMP_RIGHT("Dump Right"),
        DUMP_LEFT("Dump Left"),
        BUMP_START_LEFT("Bump Start Left"),
        BUMP_START_RIGHT("Bump Start Right"),
        DEPOT("Depot"),
        OUTPOST("Outpost");

        private String name;

        private StartEndPoint(String name) {
            this.name = name;
        }

        private static StartEndPoint fromString(String s) {
            for (StartEndPoint point : values()) {
                if (point.name.equalsIgnoreCase(s)) {
                    return point;
                }
            }
            throw new IllegalArgumentException("Invalid start/end point: " + s);
        }
    }

    private static record AutoPath(StartEndPoint start, StartEndPoint end, String name, boolean collect) {
        private static AutoPath fromString(String s) {
            if (s.startsWith("Collect")) {
                return fromCollectString(s);
            }

            String[] parts = s.split(" to ");
            if (parts.length != 2) {
                throw new IllegalArgumentException("Invalid auto path: " + s);
            }
            StartEndPoint start = StartEndPoint.fromString(parts[0]);
            StartEndPoint end = StartEndPoint.fromString(parts[1]);
            return new AutoPath(start, end, s, false);
        }

        private static AutoPath fromCollectString(String s) {
            String[] parts = s.split(" ");
            if (parts.length != 4) {
                throw new IllegalArgumentException("Invalid collect auto path: " + s);
            }

            boolean collect = false;

            if (s.endsWith("!")) {
                collect = true;
                s = s.substring(0, s.length() - 1);
            }

            StartEndPoint startEnd =
                    parts[3].equalsIgnoreCase("Left") ? StartEndPoint.TRENCH_LEFT : StartEndPoint.TRENCH_RIGHT;

            return new AutoPath(startEnd, startEnd, s, collect);
        }

        private PathPlannerPath getPathPlannerPath() {
            try {
                return PathPlannerPath.fromPathFile(name);
            } catch (FileVersionException | IOException | ParseException e) {
                throw new RuntimeException("Failed to load path: " + name, e);
            }
        }

        private List<PathPlannerTrajectoryState> getTrajectoryStates() {
            return getPathPlannerPath()
                    .getIdealTrajectory(AutoConstants.PP_CONFIG)
                    .orElse(new PathPlannerTrajectory(new ArrayList<>()))
                    .getStates();
        }

        private double getTotalTime() {
            return getPathPlannerPath()
                    .getIdealTrajectory(AutoConstants.PP_CONFIG)
                    .orElse(new PathPlannerTrajectory(new ArrayList<>()))
                    .getTotalTimeSeconds();
        }
    }

    private EnumMap<StartEndPoint, List<AutoPath>> autoPathsByStartPoint = new EnumMap<>(StartEndPoint.class);
    private ArrayList<AutoPath> selectedAutoPaths = new ArrayList<>();
    private String lastAutoSelection = "";
    private ArrayList<PathPlannerTrajectoryState> trajStates = new ArrayList<>();
    private final StringArrayPublisher autoOptionsPub = AutoConstants.AUTO_OPTIONS.publish();
    private final DoubleArrayPublisher autoOptionTimesPub = AutoConstants.AUTO_OPTION_TIMES.publish();
    private final StringArrayPublisher startOptionsPub = AutoConstants.START_OPTIONS.publish();
    private final StructArrayPublisher<Translation2d> trajPub = AutoConstants.TRAJECTORY.publish();
    private final DoubleArrayPublisher trajTimePub = AutoConstants.TRAJECTORY_TIMESTAMPS.publish();
    private final DoublePublisher timestampPub = AutoConstants.TIMESTAMP.publish();

    public void loadPathplannerPaths() {
        File pathplannerDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        for (String s : pathplannerDir.list()) System.out.println(s);
        if (!pathplannerDir.exists()) {
            throw new RuntimeException("Pathplanner directory does not exist");
        }

        File[] pathFiles = pathplannerDir.listFiles((dir, name) -> name.endsWith(".path"));
        if (pathFiles == null) {
            throw new RuntimeException("Failed to list path files");
        }

        ArrayList<AutoPath> autoPaths = new ArrayList<>(pathFiles.length);

        for (File pathFile : pathFiles) {
            String fileName = pathFile.getName();
            String nameWithoutExtension = fileName.substring(0, fileName.length() - 5);
            autoPaths.add(AutoPath.fromString(nameWithoutExtension));
        }

        for (StartEndPoint point : StartEndPoint.values()) {
            autoPathsByStartPoint.put(
                    point,
                    autoPaths.stream().filter(path -> path.start() == point).toList());
        }
        return;
    }

    public AutoCreator() {
        loadPathplannerPaths();
        autoOptionsPub.set(new String[0]);
        autoOptionTimesPub.set(new double[0]);

        startOptionsPub.set(new String[] {
            StartEndPoint.TRENCH_START_LEFT.name,
            StartEndPoint.TRENCH_START_RIGHT.name,
            StartEndPoint.BUMP_START_LEFT.name,
            StartEndPoint.BUMP_START_RIGHT.name
        });
        timestampPub.set(Timer.getFPGATimestamp());
    }

    private static String[] autoPathsToStringArray(List<AutoPath> autoPaths) {
        String[] arr = new String[autoPaths.size()];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = autoPaths.get(i).name;
        }

        return arr;
    }

    public void updateNT() {
        String selected = AutoConstants.AUTO_SELECTION.get();
        if (selected == lastAutoSelection) {
            return;
        }

        String[] parts = selected.split(";");
        if (parts.length == 1 && parts[0].isEmpty()) {
            selectedAutoPaths.clear();
            return;
        }

        try {
            StartEndPoint selectedStart = StartEndPoint.fromString(parts[0].trim());

            if (parts.length == 1) {
                List<AutoPath> autoPaths = autoPathsByStartPoint.get(selectedStart);
                autoOptionsPub.set(autoPathsToStringArray(autoPaths));
                autoOptionTimesPub.set(autoPaths.stream()
                        .mapToDouble((stream) -> stream.getTotalTime())
                        .toArray());
                timestampPub.set(Timer.getFPGATimestamp());
                selectedAutoPaths.clear();
                return;
            }

            selectedAutoPaths.clear();
            for (int i = 1; i < parts.length; i++) {
                selectedAutoPaths.add(AutoPath.fromString(parts[i].trim()));
            }
        } catch (IllegalArgumentException e) {
            System.out.println(e.getMessage());
            return;
        }

        try {
            updateCurrentTrajectoryStates();
        } catch (RuntimeException e) {
            System.out.println(e.getMessage());
            return;
        }

        List<AutoPath> autoPaths = autoPathsByStartPoint.get(selectedAutoPaths.get(selectedAutoPaths.size() - 1).end);
        autoOptionsPub.set(autoPathsToStringArray(autoPaths));
        autoOptionTimesPub.set(autoPaths.stream()
                .mapToDouble((stream) -> stream.getTotalTime())
                .toArray());
        trajPub.set(getCurrentTrajectory());
        trajTimePub.set(getCurrentTrajectoryTimestamps());
        timestampPub.set(Timer.getFPGATimestamp());
    }

    private void updateCurrentTrajectoryStates() {
        trajStates.clear();
        for (AutoPath path : selectedAutoPaths) {
            trajStates.addAll(path.getTrajectoryStates());
        }
    }

    public Translation2d[] getCurrentTrajectory() {
        return trajStates.stream().map((state) -> state.pose.getTranslation()).toArray(Translation2d[]::new);
    }

    private double[] getCurrentTrajectoryTimestamps() {
        double[] timestamps = new double[trajStates.size()];
        double totalTime = 0;
        timestamps[0] = 0;
        for (int i = 1; i < timestamps.length; i++) {
            if (trajStates.get(i).timeSeconds < trajStates.get(i - 1).timeSeconds) {
                totalTime += trajStates.get(i - 1).timeSeconds;
            }
            timestamps[i] = trajStates.get(i).timeSeconds + totalTime;
        }
        return timestamps;
    }

    public Command buildAuto(Superstructure superstructure) {
        ArrayList<Command> commands = new ArrayList<>();
        for (AutoPath path : selectedAutoPaths) {
            Command toAdd = AutoBuilder.followPath(path.getPathPlannerPath());
            if (path.collect) {
                toAdd = toAdd.beforeStarting(superstructure.startCollecting()).andThen(superstructure.stopCollecting());
            }
            commands.add(toAdd);
        }

        return Commands.sequence(commands.toArray(Command[]::new));
    }
}
