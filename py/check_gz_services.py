#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.client import Client

class ServiceChecker(Node):
    def __init__(self):
        super().__init__('service_checker')
        self.get_logger().info("🔍 检查可用服务...")
        
        # 获取所有服务
        self.timer = self.create_timer(1.0, self.check_services)
        
    def check_services(self):
        services = self.get_service_names_and_types()
        self.get_logger().info("\n📋 可用服务:")
        
        gz_services = []
        for service_name, service_types in services:
            if 'gz' in service_name or 'delete' in service_name or 'spawn' in service_name:
                gz_services.append((service_name, service_types))
                self.get_logger().info(f"  🔹 {service_name}: {service_types}")
        
        if not gz_services:
            self.get_logger().info("  ❌ 没有找到Gazebo相关服务")
        
        self.timer.cancel()
        rclpy.shutdown()

def main():
    rclpy.init()
    checker = ServiceChecker()
    rclpy.spin(checker)

if __name__ == '__main__':
    main()