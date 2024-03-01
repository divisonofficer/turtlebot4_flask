from flask import Flask, request, jsonify
from PIL import Image
import io
import torch
from torchvision import models, transforms
import torchvision.transforms.functional as F

app = Flask(__name__)

# Load a pre-trained Faster R-CNN model
model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()  # Set the model to evaluation mode


labelToName = [
    "__background__",
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


# Define a transform to convert the input image for the model
def transform_image(image):
    transform = transforms.Compose(
        [
            transforms.ToTensor(),
        ]
    )
    return transform(image).unsqueeze(0)


@app.route("/detect", methods=["POST"])
def detect_objects():
    if request.method == "POST":
        if "image" not in request.files:
            return jsonify({"error": "No image provided"}), 400

        image_file = request.files["image"]
        image_bytes = image_file.read()
        image = Image.open(io.BytesIO(image_bytes))

        # Transform the image for the model
        input_tensor = transform_image(image)

        # Get predictions
        with torch.no_grad():
            predictions = model(input_tensor)[0]

        # Extract details from predictions
        results = []
        for label, score, bbox in zip(
            predictions["labels"], predictions["scores"], predictions["boxes"]
        ):
            # You can add a threshold for scores to filter out low-probability detections
            if score < 0.5:
                continue
            if label.item() >= len(labelToName):
                continue
            results.append(
                {
                    "label": labelToName[label.item()],
                    "score": score.item(),
                    "bbox": bbox.tolist(),  # Convert bbox tensor to list
                }
            )

        return jsonify(results)


if __name__ == "__main__":
    app.run(debug=True, port=5300)
