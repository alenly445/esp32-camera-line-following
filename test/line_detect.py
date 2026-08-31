"""电脑端黑线识别预览。

只做图像识别，不连接或控制小车。按 q 退出，按 s 保存当前画面。
"""

import argparse
import time

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=None, help="本地摄像头索引")
    parser.add_argument("--url", help="ESP32 视频流，例如 http://192.168.1.23/stream")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--threshold", type=int, default=75, help="灰度黑线阈值")
    parser.add_argument("--roi", type=float, default=0.55, help="ROI 顶部占画面高度比例")
    args = parser.parse_args()

    if args.url:
        source = args.url
        cap = cv2.VideoCapture(source)
    else:
        camera = 0 if args.camera is None else args.camera
        source = f"本地摄像头 {camera}"
        cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"无法打开视频源：{source}，请检查 ESP32 IP、Wi-Fi 和串流地址。")

    previous = time.perf_counter()
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读取画面失败")
            break

        h, w = frame.shape[:2]
        roi_top = max(0, min(h - 1, int(h * args.roi)))
        roi = frame[roi_top:h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, args.threshold)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        moments = cv2.moments(mask)
        found = moments["m00"] > 500
        error = None
        if found:
            center_x = int(moments["m10"] / moments["m00"])
            error = center_x - w // 2
            cv2.line(frame, (center_x, roi_top), (center_x, h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, (roi_top + h) // 2), 6, (0, 255, 0), -1)

        cv2.rectangle(frame, (0, roi_top), (w - 1, h - 1), (255, 180, 0), 2)
        status = f"FOUND error={error:+d}px" if found else "LOST"
        cv2.putText(frame, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.putText(frame, "q: quit  s: save", (12, 58), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1)
        cv2.imshow("black-line detection", frame)
        cv2.imshow("black-line mask", mask)

        now = time.perf_counter()
        if now - previous >= 1:
            print(f"found={found} error={error} fps={1 / max(now - previous, 1e-6):.1f}")
            previous = now
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            cv2.imwrite("line_detection_snapshot.jpg", frame)
            print("已保存 line_detection_snapshot.jpg")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
