#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import PrintJob
from std_msgs.msg import String
from collections import deque
import time
import threading

class PrinterNode(Node):
    def __init__(self):
        super().__init__('printer_node')
        self.srv = self.create_service(PrintJob, 'print_job', self.handle_print_job)
        self.publisher = self.create_publisher(String, 'print_status', 10)
        self.queue = deque()
        self.processing = False
        self.lock = threading.Lock()

    def handle_print_job(self, request, response):
        self.get_logger().info(f"Received print job request: {request.document_name}")
        with self.lock:
            self.queue.append(request.document_name)
        self.publish_status(f"Job Queued: {request.document_name}")
        response.accepted = True
        if not self.processing:
            threading.Thread(target=self.process_queue).start()
        return response

    def publish_status(self, msg):
        self.publisher.publish(String(data=msg))
        self.get_logger().info(msg)

    def process_queue(self):
        with self.lock:
            self.processing = True
        while True:
            with self.lock:
                if not self.queue:
                    self.processing = False
                    break
                job = self.queue.popleft()
            self.publish_status(f"Started printing: {job}")
            for _ in range(2):
                time.sleep(1)
                self.get_logger().info("Printing...")
            self.publish_status(f"Completed printing: {job}")

def main(args=None):
    rclpy.init(args=args)
    node = PrinterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()