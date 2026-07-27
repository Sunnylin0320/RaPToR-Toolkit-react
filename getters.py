# getters.py
import subprocess


def get_all_topics():
    """
    Get all topics in the ROS2 system
    Args:
        None
    Returns:
        list: A list of all topics
    """
    result = subprocess.run("ros2 topic list", shell=True,
                            capture_output=True, text=True, check=True)

    # Check if the command was successful
    if result.returncode != 0:
        print(f"Error: {result.stderr.strip()}")
        return []

    if result.stdout.strip() == "":
        print("No topics found.")
        return []

    topics = result.stdout.strip().split("\n")

    return topics


def get_all_actions():
    """
    Get all actions in the ROS2 system
    Args:
        None
    Returns:
        list: A list of all actions with their types
    """
    result = subprocess.run("ros2 action list -t", shell=True,
                            capture_output=True, text=True, check=False)

    # Check if the command was successful
    if result.returncode != 0:
        print(f"Error: {result.stderr.strip()}")
        return []

    if result.stdout.strip() == "":
        print("No actions found.")
        return []

    # Split the action names and the action types by space
    actions = [action.split(" ")
               for action in result.stdout.strip().split("\n")]

    # Format the action commands
    for action in actions:
        action[1] = f'{action[0]} {action[1].replace("[", "").replace("]", "")}'

    return actions
