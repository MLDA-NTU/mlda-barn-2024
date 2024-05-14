import time
import subprocess
import os
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
hard_ids = [250, 114, 48, 58, 163, 254, 259, 278, 282, 85, 138, 176, 214, 241]
filename = "testhard2"

if __name__ == "__main__":
    start_time = time.time()
    print(f"Using algorithm MLDA")
    print(f"Record into CSV file: {filename}.csv")

    if input("Check before proceed. Press Y to continue. ").lower() != "y":
        exit()

    run_file_path = os.path.join(dir_path, "run_rviz_auto.py")

    for idx in hard_ids:
        process = subprocess.run(
            [
                "python",
                run_file_path,
                "--world_idx",
                f"{idx}",
                "--out",
                f"{filename}.csv",
            ]
        )