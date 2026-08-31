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
    parser.add_argument("--threshold", type=int, default=130, help="灰度黑线阈值")
    parser.add_argument("--roi", type=float, default=0.70, help="ROI 顶部占画面高度比例")
    parser.add_argument("--bottom-ratio", type=float, default=0.25,
                        help="ROI 底部窄条占 ROI 高度比例")
    parser.add_argument("--min-area", type=int, default=500,
                        help="候选黑线最小面积")
    parser.add_argument("--confirm-frames", type=int, default=3,
                        help="连续多少帧确认识别")
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
    found_streak = 0
    lost_streak = 0
    stable_found = False
    last_error = None
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

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_h = mask.shape[0]
        bottom_y = int(roi_h * (1 - args.bottom_ratio))
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            _, y, _, contour_h = cv2.boundingRect(contour)
            touches_bottom = y + contour_h >= roi_h - 3
            if area >= args.min_area and touches_bottom:
                candidates.append((area, contour))

        found = bool(candidates)
        error = None
        candidate_mask = np.zeros_like(mask)
        if found:
            _, target = max(candidates, key=lambda item: item[0])
            cv2.drawContours(candidate_mask, [target], -1, 255, thickness=-1)
            bottom_mask = candidate_mask[bottom_y:roi_h, :]
            moments = cv2.moments(bottom_mask)
            found = moments["m00"] > args.min_area
            if found:
                center_x = int(moments["m10"] / moments["m00"])
                error = center_x - w // 2
                last_error = error
                cv2.drawContours(frame, [target + np.array([[[0, roi_top]]])], -1,
                                 (0, 255, 255), 2)
                cv2.line(frame, (center_x, roi_top + bottom_y),
                         (center_x, h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, h - 8), 6, (0, 255, 0), -1)

        if found:
            found_streak += 1
            lost_streak = 0
            if found_streak >= args.confirm_frames:
                stable_found = True
        else:
            found_streak = 0
            lost_streak += 1
            if lost_streak >= args.confirm_frames:
                stable_found = False

        if stable_found:
            status = f"FOUND error={last_error:+d}px"
            status_color = (0, 255, 0)
        elif found:
            status = f"CONFIRMING {found_streak}/{args.confirm_frames}"
            status_color = (0, 200, 255)
        else:
            status = "LOST -> STOP"
            status_color = (0, 0, 255)

        bottom_line_y = roi_top + bottom_y
        cv2.rectangle(frame, (0, roi_top), (w - 1, h - 1), (255, 180, 0), 2)
        cv2.line(frame, (0, bottom_line_y), (w - 1, bottom_line_y), (255, 0, 255), 2)
        cv2.putText(frame, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, status_color, 2)
        cv2.putText(frame, "q: quit  s: save", (12, 58), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1)
        cv2.imshow("black-line detection", frame)
        cv2.imshow("black-line mask", mask)
        cv2.imshow("bottom-line candidate", candidate_mask)

        now = time.perf_counter()
        if now - previous >= 1:
            print(f"status={status} found={found} error={last_error} "
                  f"elapsed={now - previous:.2f}s")
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
