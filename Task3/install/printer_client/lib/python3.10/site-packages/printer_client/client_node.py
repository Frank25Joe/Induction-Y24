#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import PrintJob
from rclpy.parameter import Parameter
import sys

class PrintClient(Node):
    def __init__(self):
        super().__init__('print_client')
        self.cli = self.create_client(PrintJob, 'print_job')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = PrintJob.Request()

    def send_request(self, doc_name):
        self.req.document_name = doc_name
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = PrintClient()
    doc_name = client.declare_parameter('document_name', 'default.pdf').get_parameter_value().string_value
    response = client.send_request(doc_name)
    client.get_logger().info(f'Response: accepted={response.accepted}')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
