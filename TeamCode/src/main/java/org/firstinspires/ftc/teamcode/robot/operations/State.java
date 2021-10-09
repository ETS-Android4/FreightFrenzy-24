package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class State {

    public enum CompletionBasedUpon {
        PRIMARY_OPERATIONS, ALL_OPERATIONS
    }
    String title;
    CompletionBasedUpon completionBasedUpon = CompletionBasedUpon.ALL_OPERATIONS;
    Operation[] primaryOperations;
    Operation[] secondaryOperations;
    Operation[] tertiaryOperations;
    boolean hasBeenQueued, hasBeenReached;

    public State(Operation[] primaryOperations, Operation[] secondaryOperations, Operation[] tertiaryOperations, String title) {
        this.primaryOperations = primaryOperations;
        this.secondaryOperations = secondaryOperations;
        this.tertiaryOperations = tertiaryOperations;
        this.title = title;
    }

    public void setCompletionBasedUpon(CompletionBasedUpon completionBasedUpon) {
        this.completionBasedUpon = completionBasedUpon;
    }

    public boolean isReached(Robot robot) {
        if (!hasBeenQueued) {
            return false;
        }
        else if (hasBeenReached) {
            return true;
        } else {
            if(completionBasedUpon == CompletionBasedUpon.PRIMARY_OPERATIONS) {
                if (robot.primaryOperationsCompleted()) {
                    hasBeenReached = true;
                    return true;
                }
            }
            else if (completionBasedUpon == CompletionBasedUpon.ALL_OPERATIONS) {
                if (robot.allOperationsCompleted()) {
                    hasBeenReached = true;
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isQueued() {
        return hasBeenQueued;
    }

    public void queue(Robot robot) {
        for (Operation operation: primaryOperations) {
            robot.queuePrimaryOperation(operation);
        }
        for (Operation operation: secondaryOperations) {
            robot.queuePrimaryOperation(operation);
        }
        for (Operation operation: tertiaryOperations) {
            robot.queuePrimaryOperation(operation);
        }
        hasBeenQueued = true;
    }

    public String getTitle() {
        return this.title;
    }
}
