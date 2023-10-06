package frc.lib.statemachine;

public interface State {
  String getName();
  String getParent();
  void enter();
  void leave();
  void periodic(RobotStateManager rs);

  default void enter(State enteredFrom) {
    enter();
  }
}