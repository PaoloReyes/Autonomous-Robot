import json
import os
import shutil

def convert_coco_to_yolo_segmentation(json_file):
    # Load the JSON file
    with open(json_file, 'r') as file:
        coco_data = json.load(file)

    # Create a "labels" folder to store YOLO segmentation annotations
    path_name = os.path.dirname(json_file).split('/')[1:]
    path_name[0] += '_yolo'
    path_name = '/'.join(path_name)
    labels_folder = os.path.join(path_name, 'labels')
    image_folder = os.path.join(path_name, 'images')
    os.makedirs(labels_folder, exist_ok=True)
    os.makedirs(image_folder, exist_ok=True)

    # Extract annotations from the COCO JSON data
    annotations = coco_data['annotations']
    for i, annotation in enumerate(annotations):
        image_id = annotation['image_id']
        category_id = annotation['category_id']
        segmentation = annotation['segmentation']
        bbox = annotation['bbox']

        # Find the image filename from the COCO data
        for image in coco_data['images']:
            if image['id'] == image_id:
                image_filename = os.path.basename(image['file_name'])
                image_filename = os.path.splitext(image_filename)[0] # Removing the extension. (In our case, it is the .jpg or .png part.)
                image_width = image['width']
                image_height = image['height']
                break

        # Calculate the normalized center coordinates and width/height
        x_center = (bbox[0] + bbox[2] / 2) / image_width
        y_center = (bbox[1] + bbox[3] / 2) / image_height
        bbox_width = bbox[2] / image_width
        bbox_height = bbox[3] / image_height

        # Convert COCO segmentation to YOLO segmentation format
        yolo_segmentation = [f"{(x) / image_width:.5f} {(y) / image_height:.5f}" for x, y in zip(segmentation[0][::2], segmentation[0][1::2])]
        #yolo_segmentation.append(f"{(segmentation[0][0]) / image_width:.5f} {(segmentation[0][1]) / image_height:.5f}")
        yolo_segmentation = ' '.join(yolo_segmentation)

        # Generate the YOLO segmentation annotation line
        yolo_annotation = f"{category_id} {yolo_segmentation}"

        # Save the YOLO segmentation annotation in a file
        output_filename = os.path.join(labels_folder, f"{image_filename}.txt")
        with open(output_filename, 'a+') as file:
            file.write(yolo_annotation + '\n')

        # Copy the image to the "images" folder
        try:
            image_filename = os.path.basename(coco_data['images'][i]['file_name'])
            shutil.copyfile(os.path.join(os.path.dirname(json_file), image_filename), os.path.join(image_folder, image_filename))
            print(f"Image {image_filename} copied to {image_folder}.")
        except:
            continue

# Convert COCO to YOLO segmentation format
directories = filter(None, [element if '.' not in element else None for element in os.listdir('./')])

FOLDER_NAME = 'semanticsegmentation-3'

if FOLDER_NAME in directories:
    dataset = filter(None, [element if '.' not in element else None for element in os.listdir(FOLDER_NAME)])
    for data in dataset:
        json_file = f"./{FOLDER_NAME}/{data}/_annotations.coco.json" #JSON file
        split = f"./{FOLDER_NAME}/{data}/labels" #Folder
        convert_coco_to_yolo_segmentation(json_file)
        print(f"Converted {json_file} to YOLO segmentation format.")
    with open(json_file, 'r') as file:
        coco_data = json.load(file)
    with open(os.path.join(f'{FOLDER_NAME}_yolo', 'data.yaml'), 'w') as file:
        file.write(f'train: ../train/images/\n')
        file.write(f'val: ../valid/images/\n')
        file.write(f'names:\n')
        for i, category in enumerate(coco_data['categories']):
            if 'Street' in category['name']:
                file.write(f'  {i}: background\n')
            else:
                file.write(f'  {i}: {category["name"]}\n')
        file.write(f'nc: {len(coco_data["categories"])}\n')