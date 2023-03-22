import xml.etree.ElementTree as ET

tree = ET.parse("../worlds/waypoints.world")
root = tree.getroot()

models = []
for model in root.findall(".//world/model"):
    if model.attrib["name"].startswith("unit_sphere_"):
        models.append(model)
    elif model.attrib["name"].startswith("unit_sphere"):
        # First sphere has no number, so not easily sortable
        firstModel = model

models.sort(key=lambda x: int(x.attrib["name"].split("_")[2]))
models.insert(0, firstModel)

with open("waypoints.txt", "w") as f:
    for model in models:
        pose = model.find("pose")
        f.write(pose.text.split(" 0.5")[0] + "\n")
