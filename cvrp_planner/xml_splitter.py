import os
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET


class XmlSplitterNode(Node):
    def __init__(self):
        super().__init__('xml_splitter_node')
        self.get_logger().info('XmlSplitter 节点已启动，0.1 秒后开始拆分 XML')

        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.input_path = os.path.join(current_dir, 'params', 'cvrp_solution.xml')
        self.output_dir = os.path.join(current_dir, 'params')

        self._timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        self._timer.cancel()

        if not os.path.isfile(self.input_path):
            self.get_logger().error(f'找不到输入 XML 文件: {self.input_path}')
            rclpy.shutdown()
            return

        os.makedirs(self.output_dir, exist_ok=True)

        try:
            self.split_xml_by_vehicle(self.input_path, self.output_dir)
            self.get_logger().info(f'所有 Vehicle 拆分完毕，输出到目录：{self.output_dir}')
        except Exception as e:
            self.get_logger().error(f'拆分过程中出现异常：{e}')

        rclpy.shutdown()


    def split_xml_by_vehicle(self, xml_file: str, out_dir: str):
        tree = ET.parse(xml_file)
        root = tree.getroot()

        for vehicle in root.findall('Vehicle'):
            vid = vehicle.get('id')
            if vid is None:
                self.get_logger().warn('发现一个 <Vehicle> 没有 id 属性，跳过该节点')
                continue

            new_root = ET.Element('Maypoints')
            vehicle_str = ET.tostring(vehicle, encoding='utf-8')
            cloned_vehicle = ET.fromstring(vehicle_str)
            new_root.append(cloned_vehicle)

            new_tree = ET.ElementTree(new_root)
            out_path = os.path.join(out_dir, f'cvrp_solution_vehicle_{vid}.xml')
            new_tree.write(out_path, encoding='utf-8', xml_declaration=True)

            self.get_logger().info(f'已输出: {out_path}')


def main(args=None):
    rclpy.init(args=args)
    node = XmlSplitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
