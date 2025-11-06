# AI活用編：物体認識を使ってみよう
ここでは物体認識モデルであるYOLO V8mを動かし，カメラで捉えた画像中に映っている物体が何かを認識させます．実行にはRaspberryPiに搭載したAI向けプロセッサHailoを用います．

## Jupyter notebookを準備する
まずは，[8. AI活用編](/tutorials/tutorial8.md)に従ってVSCode環境の準備とRaspberryPiへの接続を行います．

VSCodeでRaspberryPiへの接続ができたら，左下の表示が以下のように接続先のRaspberryPiの情報が表示されていることを確認してください．

<img src="/imgs/ai-8-1-1.png" width=80% />

まず `ctrl` + `shift` + `P` を押して，コマンドパレットを開き `create new terminal`と入力し，表示された `Terminal: Create New Terminal in Editor Area`を選択します．するとコマンドを入力するターミナルが立ち上がります．


ここで以下のように自分のプログラムを保存しておくためのフォルダを作成します．自分の名前をアルファベットで記入したフォルダを作成してください．
```
mkdir -p ~/robotproject/<自分の名前をアルファベットで書く>
```

次に，VSCodeで上記で作成したフォルダを開きます．

Jupyter Notebookを作成します．VSCode上で， `ctrl` + `shift` + `P`キーを押してください．コマンドパレットが表示されたら， `Create: New Jupyter Notebook`と入力し，表示された同名のメニューを選んでEnterを押します．すると，プログラムを記入するためのJupyter Notebookが立ち上がります．

Jupyter NotebookにPythonプログラムを記述して実行することができます．ここでは物体認識モデルであるYOLOを実行するためのコードを示します．

```py
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from IPython import display
from io import BytesIO
import time
import cv2
from hailo_platform import VDevice, HailoSchedulingAlgorithm

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", 
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", 
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", 
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", 
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", 
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", 
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", 
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", 
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", 
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", 
    "toothbrush"
]

TIMEOUT_MS = 1000
VIDEO_DEVICE = 0
OBJECT_DETECTION_THRESHOLD = 0.7
IMG_SIZE = 640  # 640x640

def show_image(rgb_array, fmt='jpeg'):
    buf = BytesIO()
    Image.fromarray(rgb_array).save(buf, format=fmt)
    display.display(display.Image(data=buf.getvalue()))

def get_bbox_info(hailo_result, frame_origin):
    bboxes = []
    scores = []
    classes = []

    for i, detection in enumerate(hailo_result):
        if detection.size==0:
            continue
        # j: box number
        for j in range(len(detection)):
            #bbox = np.array(detection)[j][:4]
            #score = np.array(detection)[j][4]
            bbox = detection[j][:4]
            score = detection[j][4]
            if score < OBJECT_DETECTION_THRESHOLD:
                continue
            else:
                bboxes.append(np.round(bbox * IMG_SIZE))
                scores.append(score)
                classes.append(i)
    return bboxes, scores, classes

def draw_bboxes(bboxes, scores, classes, frame_origin, fps = 'N/A'):
    img = Image.fromarray(frame_origin)
    for bbox, score, cls in zip(bboxes, scores, classes):
        # extract bounding box coordinates
        y1, x1, y2, x2 = bbox

        # Draw bounding box and label
        draw = ImageDraw.Draw(img)
        draw.rectangle([(x1, y1), (x2, y2)], outline="red", width=3)
        draw.text((x1+5, y1), f"{COCO_CLASSES[cls]}: {score:.2f}", fill="lightgreen", font_size=20)

        # Put FPS
        draw.text((0, 0), f'FPS: {fps:.2f}', 'blue', font_size=20)
    return np.array(img)

def main():
    timeout_ms = 1000
    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN

    # Open camera
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    assert cap.isOpened(), "Failed to open video device"

    # The vdevice is used as a context manager (”with” statement) to ensure it's released ontime.
    with VDevice(params) as vdevice:
        # Create an infer model from an HEF:
        infer_model = vdevice.create_infer_model('../yolov8m.hef')
        # Configure the infer model and create bindings for it
        with infer_model.configure() as configured_infer_model:
            bindings = configured_infer_model.create_bindings()

            # Prepare output buffer
            buffer = np.empty(infer_model.output().shape, dtype=np.float32)
            bindings.output().set_buffer(buffer)

            try:
                while True:
                    start = time.time()
                    
                    # Get a frame from camera
                    ret, frame = cap.read()

                    if not ret:
                        print("Failed to read frame from video device")
                        break

                    # Pre-process for inference
                    frame = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame_rgb = np.array(frame_rgb).astype(np.uint8)
                    input_img = np.expand_dims(frame_rgb, axis=0)

                    # Input farame
                    bindings.input().set_buffer(input_img)

                    # Inference on Hailo
                    configured_infer_model.run([bindings], TIMEOUT_MS)
                    buffer = bindings.output().get_buffer()

                    # Post processes: extract detected object information
                    bboxes, scores, classes = get_bbox_info(buffer, frame_rgb)

                    # Calculate FPS and draw bounding boxes
                    end = time.time()
                    fps = 1 / (end - start)
                    result_img = draw_bboxes(bboxes, scores, classes, frame_rgb, fps)

                    # Show image (for Jupyter Notebook)
                    show_image(result_img)
                    display.clear_output(wait=True)

            except KeyboardInterrupt:
                pass
            finally:
                cap.release()
```

新しくセルを下に作成して，以下の内容を記述します．

```py
main()
```

この後，使用するPythonカーネルを選択し，プログラムを実行してみましょう．

# 発展課題
- カチャカのカメラ画像をsubscribeしてウェブカメラの代わりに入力してみましょう