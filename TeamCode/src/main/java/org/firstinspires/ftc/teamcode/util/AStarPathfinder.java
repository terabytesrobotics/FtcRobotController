
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
    private double robotWidthMM;
    private double robotHeightMM;
    private double sampleSizeMM;

    // 6 feet is 1828.8mm, so min, min is theoretically -1828.8, -1828.8, but 2000mm
    // allows for an extra buffer in case odometry is offset (preventing locking us out of an edge)
    private double maxMM = 2000;

    private Set<Node> obstacles;

    public AStarPathfinder(double robotWidthMM, double robotHeightMM,
                           double sampleSizeMM, Set<Node> obstacles) {
        this.robotWidthMM = robotWidthMM;
        this.robotHeightMM = robotHeightMM;
        this.sampleSizeMM = sampleSizeMM;
        this.obstacles = obstacles;
    }

    private boolean isInBounds(double x, double y) {
        return x >= -maxMM && y >= -maxMM
                && x + robotWidthMM <= maxMM && y + robotHeightMM <= maxMM;
    }

    private double remainingSteps(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    private boolean canMoveTo(double x, double y, Set<Node> obstacles) {
        if (!isInBounds(x, y)) {
            return false;
        }

        for (Node obstacle : obstacles) {
            if (Math.abs(x - obstacle.x) <= sampleSizeMM / 2.0
                    && Math.abs(y - obstacle.y) <= sampleSizeMM / 2.0) {
                return false;
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

    public List<Node> eval(Node start, Node goal) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(Node::getF));

        Set<Node> closedSet = new HashSet<>();

        Node startNode = new Node(start.x, start.y);
        startNode.h = remainingSteps(startNode, goal);

        if (!canMoveTo(start.x, start.y, obstacles)) {
            return Collections.emptyList();
        }

        Node closestNode = startNode;
        double closestDistance = Math.hypot(startNode.x - goal.x, startNode.y - goal.y);

        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            double distanceToGoal = Math.hypot(current.x - goal.x, current.y - goal.y);

            if (distanceToGoal < closestDistance) {
                closestDistance = distanceToGoal;
                closestNode = current;
            }

            if (distanceToGoal <= sampleSizeMM) {
                return reconstructPath(current);
            }

            closedSet.add(current);

            List<Node> neighbors = Arrays.asList(
                    new Node(current.x + sampleSizeMM, current.y),
                    new Node(current.x, current.y + sampleSizeMM),
                    new Node(current.x - sampleSizeMM, current.y),
                    new Node(current.x, current.y - sampleSizeMM)
            );

            for (Node neighbor : neighbors) {
                if (!canMoveTo(neighbor.x, neighbor.y, obstacles)) {
                    continue;
                }

                if (closedSet.contains(neighbor)) {
                    continue;
                }

                double tentativeG = current.g + sampleSizeMM;

                Node existing = openSet.stream()
                        .filter(n -> n.x == neighbor.x && n.y == neighbor.y)
                        .findFirst()
                        .orElse(null);

                if (existing == null) {
                    neighbor.g = tentativeG;
                    neighbor.h = remainingSteps(neighbor, goal);
                    neighbor.parent = current;
                    openSet.add(neighbor);
                } else if (tentativeG < existing.g) {
                    openSet.remove(existing);
                    existing.g = tentativeG;
                    existing.parent = current;
                    openSet.add(existing);
                }
            }
        }

        return reconstructPath(closestNode);
    }
}
