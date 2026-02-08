package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoCreator {
    private enum StartEndPoint {
        TRENCH_START_LEFT("Trench Start Left"),
        TRENCH_START_RIGHT("Trench Start Right"),
        TRENCH_RIGHT("Trench Right"),
        TRENCH_LEFT("Trench Left"),
        CLIMB_RIGHT("Climb Right"),
        CLIMB_LEFT("Climb Left"),
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

    private static record AutoPath(StartEndPoint start, StartEndPoint end, String name) {
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
            return new AutoPath(start, end, s);
        }

        private static AutoPath fromCollectString(String s) {
            String[] parts = s.split(" ");
            if (parts.length != 4) {
                throw new IllegalArgumentException("Invalid collect auto path: " + s);
            }
            StartEndPoint startEnd =
                    parts[3].equalsIgnoreCase("Left") ? StartEndPoint.TRENCH_LEFT : StartEndPoint.TRENCH_RIGHT;

            return new AutoPath(startEnd, startEnd, s);
        }

        private PathPlannerPath getPathPlannerPath() {
            try {
                return PathPlannerPath.fromPathFile(name);
            } catch (FileVersionException | IOException | ParseException e) {
                throw new RuntimeException("Failed to load path: " + name, e);
            }
        }
    }

    private EnumMap<StartEndPoint, List<AutoPath>> autoPathsByStartPoint = new EnumMap<>(StartEndPoint.class);
    private ArrayList<AutoPath> selectedAutoPaths = new ArrayList<>();
    private String lastAutoSelection = "";

    public void loadPathplannerPaths() {
        File pathplannerDir = new File(Filesystem.getDeployDirectory(), "pathplanner\\paths");
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
        Logger.recordOutput("Autos/Auto Options", new AutoPath[0]);
        Logger.recordOutput("Autos/Start Options", new StartEndPoint[] {
            StartEndPoint.TRENCH_START_LEFT,
            StartEndPoint.TRENCH_START_RIGHT,
            StartEndPoint.BUMP_START_LEFT,
            StartEndPoint.BUMP_START_RIGHT
        });
        NetworkTableInstance.getDefault()
                .getStringTopic("Auto/Selection")
                .publish()
                .set("");
    }

    public void updateNT() {
        String selected = AutoConstants.AUTO_SELECTION.get();
        if (selected == lastAutoSelection) {
            return;
        }

        String[] parts = selected.split(":");
        if (parts.length == 1 && parts[0].isEmpty()) {
            selectedAutoPaths.clear();
            return;
        }

        StartEndPoint selectedStart = StartEndPoint.fromString(parts[0].trim());

        if (parts.length == 1) {
            Logger.recordOutput(
                    "Autos/Auto Options",
                    autoPathsByStartPoint.get(selectedStart).toArray(AutoPath[]::new));
            selectedAutoPaths.clear();
            return;
        }

        selectedAutoPaths.clear();
        for (int i = 1; i < parts.length; i++) {
            selectedAutoPaths.add(AutoPath.fromString(parts[i].trim()));
        }

        Logger.recordOutput(
                "Autos/Auto Options",
                autoPathsByStartPoint
                        .get(selectedAutoPaths.get(selectedAutoPaths.size() - 1).end)
                        .toArray(AutoPath[]::new));

        Logger.recordOutput("Autos/Trajectory", getCurrentTrajectory());
    }

    public Translation2d[] getCurrentTrajectory() {
        ArrayList<Translation2d> points = new ArrayList<>();
        for (AutoPath path : selectedAutoPaths) {
            points.addAll(path
                    .getPathPlannerPath()
                    .getIdealTrajectory(AutoConstants.PP_CONFIG)
                    .orElse(new PathPlannerTrajectory(new ArrayList<>()))
                    .getStates()
                    .stream()
                    .map((state) -> state.pose.getTranslation())
                    .toList());
        }

        return points.toArray(Translation2d[]::new);
    }

    public Command buildAuto() {
        ArrayList<Command> commands = new ArrayList<>();
        for (AutoPath path : selectedAutoPaths) {
            commands.add(AutoBuilder.followPath(path.getPathPlannerPath()));
        }

        return Commands.sequence(commands.toArray(Command[]::new));
    }
}
