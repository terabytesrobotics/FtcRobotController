package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.data.Node;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

public class AStarPathfinder {
    // these are viewed in 2d space, so height is the x axis of the robot and width is the y
    private double robotWidth;
    private double robotHeight;

    // 6 feet is 1828.8mm, so min, min is theoretically -1828.8, -1828.8, but 2000mm
    // allows for an extra buffer in case odometry is offset(preventing locking us out of an edge
    private double maxMM = 2000;

    public AStarPathfinder(double robotWidth, double robotHeight) {
        this.robotWidth = robotWidth;
        this.robotHeight = robotHeight;
    }

    private boolean isInBounds(int x, int y) {
        return x >= -maxMM && y >= -maxMM
                && x + robotWidth <= maxMM && y + robotHeight <= maxMM;
    }

    private int remainingSteps(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    private boolean canMoveTo(int x, int y, Set<Node> obstacles) {
        if (!isInBounds(x, y)) {
            return false;
        }

        for (int dx = 0; dx < robotWidth; dx++) {
            for (int dy = 0; dy < robotHeight; dy++) {
                if (obstacles.contains(new Node(x + dx, y + dy))) {
                    return false;
                }
            }
        }

        return true;
    }

    private List<Node> reconstructPath(Node node) {
        List<Node> path = new ArrayList<>();
        Node current = node;

        while (current != null) {
            path.add(current);
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }

    public List<Node> eval(Node start, Node goal, Set<Node> obstacles) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(Node::getF));
        Set<Node> closedSet = new HashSet<>();

        Node startNode = new Node(start.x, start.y);
        startNode.h = remainingSteps(startNode, goal);

        if (!canMoveTo(start.x, start.y, obstacles)) return null;
        if (!canMoveTo(goal.x, goal.y, obstacles)) return null;

        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.x == goal.x && current.y == goal.y) {
                return reconstructPath(current);
            }

            closedSet.add(current);

            List<Node> neighbours = Arrays.asList(
                    new Node(current.x + 1, current.y),
                    new Node(current.x, current.y + 1),
                    new Node(current.x - 1, current.y),
                    new Node(current.x, current.y - 1)
            );

            for (Node neighbour : neighbours) {
                if (!canMoveTo(neighbour.x, neighbour.y, obstacles)) continue;
                if (closedSet.contains(neighbour)) continue;

                int tentativeG = current.g + 1;

                Node existing = openSet.stream()
                        .filter(n -> n.x == neighbour.x && n.y == neighbour.y)
                        .findFirst()
                        .orElse(null);

                if (existing == null) {
                    neighbour.g = tentativeG;
                    neighbour.h = remainingSteps(neighbour, goal);
                    neighbour.parent = current;
                    openSet.add(neighbour);
                } else if (tentativeG < existing.g) {
                    openSet.remove(existing);
                    existing.g = tentativeG;
                    existing.parent = current;
                    openSet.add(existing);
                }
            }
        }

        return null;
    }
}
