import cv2
import numpy as np

class TrafficLight:
    def __init__(self):
        self.state = "RED"

    def update(self, traffic_density):
        """
        Determines traffic light state based on traffic density thresholds.
        """
        if traffic_density > 10:
            self.state = "RED"
        elif 5 <= traffic_density <= 10:
            self.state = "YELLOW"
        else:
            self.state = "GREEN"
        return self.state

def detect_vehicles(frame):
    """
    Detects vehicles in the frame using simple contour detection.
    Returns the traffic density (number of vehicles).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    vehicle_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
    return len(vehicle_contours)

def main():
    video_path = input("Enter the path of your traffic video: ")
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Video not found!")
        return

    traffic_light = TrafficLight()

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # End of video

        traffic_density = detect_vehicles(frame)
        light_state = traffic_light.update(traffic_density)

        # Display info on frame
        display_frame = frame.copy()
        cv2.putText(display_frame, f"Traffic Density: {traffic_density}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        color = (0,0,255) if light_state=="RED" else (0,255,255) if light_state=="YELLOW" else (0,255,0)
        cv2.putText(display_frame, f"Traffic Light: {light_state}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.imshow("Traffic Simulation", display_frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

