from roboflow import Roboflow

rf = Roboflow(api_key="WVwS1qsuBAjRcIrfsYmi")
project = rf.workspace("ilovecantoral").project("semanticsegmentation-lamyy")
version = project.version(9)
dataset = version.download("coco-segmentation")