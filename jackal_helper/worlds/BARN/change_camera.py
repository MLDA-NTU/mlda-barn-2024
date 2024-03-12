import os
import xml.etree.ElementTree as ET
current_directory = os.path.dirname(os.path.abspath(__file__))

START = 299 
STOP = 300

new_pose = "-2 4 10 0 1.57 3.14"

for i in range(START, STOP):
    filename = "world_{}.world".format(i)
    filepath = os.path.join(current_directory,filename)
    tree = ET.parse(filepath)
    root = tree.getroot()
    world = root[0]
    gui = world[-1]
    camera = gui[0]
    pose = camera[0]
    pose.text = new_pose
    print(tree.getroot()[0][-1][0][0].text)
    print("File overwritten: ", filepath)
    tree.write(filepath)
