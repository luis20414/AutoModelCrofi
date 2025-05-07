
class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'
        self.bridge = CvBridge()

        self.value_pub = self.create_publisher(Bool, '/stop', 10)
        self.processed_pub = self.create_publisher(Image, '/red_image', 10)
        self.camera_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        self.go_sub = self.create_subscription(Bool, '/go_state', self.avanzar_callback, 10)
        self.go_on_sub = self.create_subscription(Bool, '/go_on_state', self.continuar_callback, 10)

        self.allow_processing = False  # <-- Bandera de control
        self.frame_count = 0
        self.last_detection_time = 0
        self.get_logger().info("Nodo de detección de señales inicializado")

    def avanzar_callback(self, msg):
        if msg.data:
            self.allow_processing = True
            self.get_logger().info("Procesamiento activado por /go_state")

    def continuar_callback(self, msg):
        if msg.data:
            self.allow_processing = True
            self.get_logger().info("Procesamiento activado por /go_on_state")

    def image_callback(self, msg):
        if not self.allow_processing:
            return

        try:
            if time.time() - self.last_detection_time < 20:
                self.get_logger().info("Esperando 20 segundos antes de permitir una nueva detección.")
                return

            if msg.encoding != 'bgr8':
                self.get_logger().error(f"Formato de imagen no compatible: {msg.encoding}")
                return

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (640, 480))

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            processed_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.processed_pub.publish(processed_msg)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            detected = False

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 500:
                    continue

                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) == 8:
                    x, y, w, h = cv2.boundingRect(contour)
                    roi = frame[y:y+h, x:x+w]
                    if self.frame_count % 10 == 0:
                        img_scene = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        text = pytesseract.image_to_string(img_scene)
                        text = text.strip().upper()
                        if text:
                            self.get_logger().info(f"Señal detectada: {text}")
                            detected = True
                            self.last_detection_time = time.time()
                            break

            value_msg = Bool()
            value_msg.data = detected
            self.get_logger().info(f"Publicando señal detectada: {value_msg.data}")
            self.value_pub.publish(value_msg)

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()