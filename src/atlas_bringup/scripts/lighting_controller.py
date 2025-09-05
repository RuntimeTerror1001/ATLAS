#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetLightProperties
from gazebo_msgs.srv import GetLightProperties
from std_msgs.msg import ColorRGBA


class LightingController(Node):
    def __init__(self):
        super().__init__('lighting_controller')

        # Parameters
        self.declare_parameter('lighting_scenario', 'normal')

        # extra lights in the world
        self.declare_parameter('extra_lights', ['Warehouse_CeilingLight_003', 'warehouse_light_1', 'warehouse_light_2', 'warehouse_light_3', 'warehouse_light_4'])

        self.scenario = self.get_parameter('lighting_scenario').get_parameter_value().string_value
        self.extra_lights = list(self.get_parameter('extra_lights').get_parameter_value().string_array_value)

        # Service clients
        self.set_client = self.create_client(SetLightProperties, '/gazebo/set_light_properties')
        self.get_client = None
        self.get_client = self.create_client(GetLightProperties, '/gazebo/get_light_properties')
        
        self._applied = False
        self.timer = self.create_timer(0.5, self._try_apply) # poll until services are up

        self.get_logger().info(f'Lighting Controller ready, scenario="{self.scenario}"')

    def _try_apply(self):
        if self._applied:
            return

        if not self.set_client.service_is_ready():
            self.get_logger().debug('Waiting for /gazebo/set_light_properties...')
            return
        if not self.get_client.service_is_ready():
            self.get_logger().debug('Waiting for /gazebo/get_light_properties...')
            return
        
        self._applied = True
        self.timer.cancel()
        self.apply_scenario()

    def apply_scenario(self):
        s = self.scenario.lower()
        if s == 'normal':
            self.setup_normal()
        elif s == 'dim':
            self.setup_dim()
        elif s == 'bright':
            self.setup_bright()
        else:
            self.get_logger().warn(f'Unknown lighting_scenario "{self.scenario}", defaulting to normal.')
            self.setup_normal()

    def setup_normal(self):
        self.get_logger().info('Applying NORMAL lighting...')
        self.set_light('sun',
                       diffuse=(0.8, 0.8, 0.8, 1.0), specular=(0.2, 0.2, 0.2, 1.0),
                       k=0.9, l=0.01, q=0.001)
        for name in self._warehouse_lights():
            self.set_light(name,
                           diffuse=(0.7, 0.7, 0.7, 1.0), specular=(0.1, 0.1, 0.1, 1.0),
                           k=0.5, l=0.01, q=0.001)

    def setup_dim(self):
        self.get_logger().info('Applying DIM lighting...')
        self.set_light('sun',
                       diffuse=(0.3, 0.3, 0.3, 1.0), specular=(0.05, 0.05, 0.05, 1.0),
                       k=0.5, l=0.02, q=0.002)
        for name in self._warehouse_lights():
            self.set_light(name,
                           diffuse=(0.3, 0.3, 0.3, 1.0), specular=(0.02, 0.02, 0.02, 1.0),
                           k=0.3, l=0.02, q=0.003)

    def setup_bright(self):
        self.get_logger().info('Applying BRIGHT lighting...')
        self.set_light('sun',
                       diffuse=(1.0, 1.0, 1.0, 1.0), specular=(0.4, 0.4, 0.4, 1.0),
                       k=1.0, l=0.005, q=0.0005)
        for name in self._warehouse_lights():
            self.set_light(name,
                           diffuse=(0.9, 0.9, 0.9, 1.0), specular=(0.2, 0.2, 0.2, 1.0),
                           k=0.7, l=0.005, q=0.0005)

    def _warehouse_lights(self):
        return [f'warehouse_light_{i}' for i in range(1, 5)] + self.extra_lights
    
    def set_light(self, light_name, *, diffuse, specular, k, l, q, timeout=2.0):
        # Optionally verify the light exists first
        if self.get_client:
            try:
                req_g = GetLightProperties.Request()
                req_g.light_name = light_name
                fut_g = self.get_client.call_async(req_g)
                rclpy.spin_until_future_complete(self, fut_g, timeout_sec=timeout)
                ok = fut_g.done() and fut_g.result() is not None and fut_g.result().success
                if not ok:
                    self.get_logger().warn(f'Light "{light_name}" not found (skipping).')
                    return
            except Exception as e:
                self.get_logger().warn(f'Could not query "{light_name}": {e}')

        req = SetLightProperties.Request()
        req.light_name = light_name
        req.diffuse = ColorRGBA(r=float(diffuse[0]), g=float(diffuse[1]), b=float(diffuse[2]), a=float(diffuse[3]))
        req.specular = ColorRGBA(r=float(specular[0]), g=float(specular[1]), b=float(specular[2]), a=float(specular[3]))
        req.attenuation_constant = float(k)
        req.attenuation_linear = float(l)
        req.attenuation_quadratic = float(q)

        fut = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            if fut.result().success:
                self.get_logger().info(f'Updated light "{light_name}".')
            else:
                self.get_logger().error(f'Failed to update "{light_name}": {fut.result().status_message}')
        else:
            self.get_logger().error(f'Timeout updating "{light_name}".')


def main(args=None):
    rclpy.init(args=args)
    node = LightingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
