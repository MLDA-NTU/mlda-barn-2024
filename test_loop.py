import time
import argparse
import subprocess
import os
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


if __name__ == "__main__":
    start_time = time.time()
    parser = argparse.ArgumentParser(description="Test")
    parser.add_argument('--start','-s', type = int, default = 0)
    parser.add_argument('--to', '-t', type = int, default = 0)
    parser.add_argument('--eps', '-e', type = int, default = 1)
    parser.add_argument('--rviz', '-r', action="store_true")
    parser.add_argument('--gui', '-g', action="store_true")
    parser.add_argument('--type', type=str, required=True)
    parser.add_argument('--out', '-o' ,type=str, default = None)
    args = parser.parse_args()

    filename = args.out if args.out != None else args.type
    
    print(f"==== Run {args.eps} times starting from {args.start} to {args.to} ======")
    print(f"With RVIZ {args.rviz}, Using algorithm {args.type}")
    print(f"Record into CSV file: {filename}.csv")
    
    if input("Check before proceed. Press Y to continue. ").lower() != "y":
        exit()
    
    run_file_path = os.path.join(dir_path, "run.py")
    
    for i in range(args.eps):
        for j in range(args.start, args.to +1):
            if args.rviz:
                if args.gui:
                    process = subprocess.run(['python', run_file_path,  "--world_idx", f"{j}", "--out", f"{filename}.csv", "--type", f"{args.type}","--rviz", "--gui"])

                else:
                    process = subprocess.run(['python', run_file_path,  "--world_idx", f"{j}", "--out", f"{filename}.csv", "--type", f"{args.type}","--rviz"])
            else:
                process = subprocess.run(['python', run_file_path,  "--world_idx", f"{j}", "--out", f"{filename}.csv", "--type", f"{args.type}"])

    with open("test_log.txt", "a") as f:
        f.write(f"=============================\n")
        f.write(f"Run {args.eps} times in WORLDS {args.start} to {args.to}\n")
        f.write(f"With RVIZ {args.rviz}, using algorithm: {args.type}\n")
        f.write(f"Record into CSV file: {args.type}.csv\n")
        duration = round(time.time()- start_time,2)
        f.write(f"{(args.to - args.start +1)*args.eps} runs takes {duration} s\n")
        f.write(f"Or {round(duration/60,2)} minutes or {round(duration/3600,4)} hours\n")
        f.write("\n\n")


