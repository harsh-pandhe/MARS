import os
import signal
import subprocess

def kill_by_name(name):
    try:
        # Find PIDs matching the process name
        pids = subprocess.check_output(["pgrep", "-f", name]).decode().strip().split()
        for pid_str in pids:
            pid = int(pid_str)
            if pid != os.getpid():
                print(f"Killing process {name} with PID {pid}")
                os.kill(pid, signal.SIGKILL)
    except subprocess.CalledProcessError:
        pass
    except Exception as e:
        print(f"Error killing {name}: {e}")

if __name__ == "__main__":
    processes_to_kill = [
        "robot_state_publisher",
        "parameter_bridge",
        "gz-sim-templated-host",
        "ruby",
        "async_slam_toolbox_node",
        "rviz2"
    ]
    for proc in processes_to_kill:
        kill_by_name(proc)
    print("Cleanup completed.")
