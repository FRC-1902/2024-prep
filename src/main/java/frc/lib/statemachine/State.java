package frc.lib.statemachine;

public interface State {
  String getName();
  String getParent();
  void enter();
  void leave();
  void periodic();

  default void enter(State enteredFrom) {
    enter();
  }
}