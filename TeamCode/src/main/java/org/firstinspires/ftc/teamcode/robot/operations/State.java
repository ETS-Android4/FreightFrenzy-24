package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;

/**
 * State is a class consisting of primary, secondary and tertiary operations. One can specify if
 * it is considered reached upon finishing all or just the primary operations.
 */
public class State {

    public enum CompletionBasedUpon {
        PRIMARY_OPERATIONS, ALL_OPERATIONS
    }
    String title;
    CompletionBasedUpon completionBasedUpon = CompletionBasedUpon.ALL_OPERATIONS;
    ArrayList<Operation> primaryOperations = new ArrayList<>();
    ArrayList<Operation> secondaryOperations = new ArrayList<>();
    ArrayList<Operation> tertiaryOperations = new ArrayList<>();
    boolean hasBeenQueued, hasBeenReached;

    public State(String title) {
        this.title = title;
        this.hasBeenQueued = false;
        this.hasBeenReached = false;
    }

    public void addPrimaryOperation(Operation operation) {
        this.primaryOperations.add(operation);
    }
    public void addSecondaryOperation(Operation operation) {
        this.secondaryOperations.add(operation);
    }
    public void addTertiaryOperation(Operation operation) {
        this.tertiaryOperations.add(operation);
    }
    public void setCompletionBasedUpon(CompletionBasedUpon completionBasedUpon) {
        this.completionBasedUpon = completionBasedUpon;
    }

    /**
     * Checks to see if a state had been reached.
     * @param robot Robot
     * @return true if state has been reached, false otherwise
     */
    public boolean isReached(Robot robot) {
        if (!hasBeenQueued) {
            //Can't be reached if it hasn't been queued
            return false;
        }
        else if (hasBeenReached) {
            //if we already have established that the state was reached, no need to check again
            return true;
        } else {
            if(completionBasedUpon == CompletionBasedUpon.PRIMARY_OPERATIONS) {
                //check if all primary operations are complete if that's all we need for completion
                if (robot.primaryOperationsCompleted()) {
                    //mark that state has been reached
                    hasBeenReached = true;
                    Match.log(this.title + " completed as all primary operations are done");
                    return true;
                }
            }
            else if (completionBasedUpon == CompletionBasedUpon.ALL_OPERATIONS) {
                //check if all operations are complete if that's what we need for completion
                if (robot.allOperationsCompleted()) {
                    //mark that state has been reached
                    hasBeenReached = true;
                    Match.log(this.title + " completed as all operations are done");
                    return true;
                }
            }
        }
        return false;
    }

    public synchronized boolean isQueued() {
        return hasBeenQueued;
    }

    /**
     * Queue the state by asking the robot to queue primary, secondary and tertiary operations
     * @param robot
     */
    public synchronized void queue(Robot robot) {
        for (Operation operation: primaryOperations) {
            robot.queuePrimaryOperation(operation);
        }
        for (Operation operation: secondaryOperations) {
            robot.queueSecondaryOperation(operation);
        }
        for (Operation operation: tertiaryOperations) {
            robot.queueTertiaryOperation(operation);
        }
        hasBeenQueued = true;
    }

    public String getTitle() {
        return this.title;
    }
}
