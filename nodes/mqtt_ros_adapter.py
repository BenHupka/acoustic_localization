#!/usr/bin/env python3
"""
Publishen der Anker- und ROV-Positionen, die über mqtt empfangen wurden
"""
import rclpy
from hippo_msgs.msg import RTK
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json


class mqtt_ros_adapter(Node):

    def __init__(self):
        super().__init__(node_name='mqtt_ros_adapter')

        self.broker_address = "localhost"
        self.port = 1883
        self.topics = [
            "buoy_1/local_info",
            "buoy_1/status",
            "buoy_2/local_info",
            "buoy_2/status",
            "buoy_3/local_info",
            "buoy_3/status",
            "vehicle/local_info",
            "vehicle/status"
        ]

        self.buoy_1_north = 0.0
        self.buoy_1_east = 0.0
        self.buoy_1_down = 0.0
        self.buoy_1_gnssfix = 0
        self.buoy_1_carrsoln = 0
        self.buoy_1_relposval = 0

        self.buoy_2_north = 0.0
        self.buoy_2_east = 0.0
        self.buoy_2_down = 0.0
        self.buoy_2_gnssfix = 0
        self.buoy_2_carrsoln = 0
        self.buoy_2_relposval = 0

        self.buoy_3_north = 0.0
        self.buoy_3_east = 0.0
        self.buoy_3_down = 0.0
        self.buoy_3_gnssfix = 0
        self.buoy_3_carrsoln = 0
        self.buoy_3_relposval = 0

        self.vehicle_north = 0.0
        self.vehicle_east = 0.0
        self.vehicle_down = 0.0
        self.vehicle_gnssfix = 0
        self.vehicle_carrsoln = 0
        self.vehicle_relposval = 0
    

        self.buoy_1_pub = self.create_publisher(msg_type=RTK,
                                                      topic='buoy_1',
                                                      qos_profile=1)
        self.buoy_2_pub = self.create_publisher(msg_type=RTK,
                                                      topic='buoy_2',
                                                      qos_profile=1)
        self.buoy_3_pub = self.create_publisher(msg_type=RTK,
                                                      topic='buoy_3',
                                                      qos_profile=1)
        self.vehicle_pub = self.create_publisher(msg_type=RTK,
                                                      topic='vehicle',
                                                      qos_profile=1)
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(self.broker_address, port=self.port)
        # Blocking loop to keep the client running
        self.client.loop_forever()
    
    # Called when connection to the broker is established
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        # Subscribe to all topics once connected
        for topic in self.topics:
            client.subscribe(topic)

    # Called when a message is received
    def on_message(self, client, userdata, msg):
        # Decode the message's payload and convert from JSON
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == "buoy_1/local_info":
            self.buoy_1_north = payload["north"] * 1e-2
            self.buoy_1_east = payload["east"] * 1e-2
            self.buoy_1_down = - payload["down"] * 1e-2

        if msg.topic == "buoy_1/status":
            self.buoy_1_gnssfix = payload["gnssFixOK"]
            self.buoy_1_carrsoln = payload["carrSoln"]
            self.buoy_1_relposval = payload["relPosValid"]

            buoy_1 = RTK()
            buoy_1.id = 0
            buoy_1.header.stamp = self.get_clock().now().to_msg()
            buoy_1.north = self.buoy_1_north
            buoy_1.east = self.buoy_1_east
            buoy_1.down = self.buoy_1_down
            buoy_1.gnss_fix_ok = self.buoy_1_gnssfix
            buoy_1.carr_soln = self.buoy_1_carrsoln
            buoy_1.rel_pos_valid = self.buoy_1_relposval

            self.buoy_1_pub.publish(buoy_1)

        if msg.topic == "buoy_2/local_info":
            self.buoy_2_north = payload["north"] * 1e-2
            self.buoy_2_east = payload["east"] * 1e-2
            self.buoy_2_down = - payload["down"] * 1e-2

        if msg.topic == "buoy_2/status":
            self.buoy_2_gnssfix = payload["gnssFixOK"]
            self.buoy_2_carrsoln = payload["carrSoln"]
            self.buoy_2_relposval = payload["relPosValid"]

            buoy_2 = RTK()
            buoy_2.id = 1
            buoy_2.header.stamp = self.get_clock().now().to_msg()
            buoy_2.north = self.buoy_2_north
            buoy_2.east = self.buoy_2_east
            buoy_2.down = self.buoy_2_down
            buoy_2.gnss_fix_ok = self.buoy_2_gnssfix
            buoy_2.carr_soln = self.buoy_2_carrsoln
            buoy_2.rel_pos_valid = self.buoy_2_relposval

            self.buoy_2_pub.publish(buoy_2)

        if msg.topic == "buoy_3/local_info":
            self.buoy_3_north = payload["north"] * 1e-2
            self.buoy_3_east = payload["east"] * 1e-2
            self.buoy_3_down = - payload["down"] * 1e-2

        if msg.topic == "buoy_3/status":
            self.buoy_3_gnssfix = payload["gnssFixOK"]
            self.buoy_3_carrsoln = payload["carrSoln"]
            self.buoy_3_relposval = payload["relPosValid"]

            buoy_3 = RTK()
            buoy_3.id = 2
            buoy_3.header.stamp = self.get_clock().now().to_msg()
            buoy_3.north = self.buoy_3_north
            buoy_3.east = self.buoy_3_east
            buoy_3.down = self.buoy_3_down
            buoy_3.gnss_fix_ok = self.buoy_3_gnssfix
            buoy_3.carr_soln = self.buoy_3_carrsoln
            buoy_3.rel_pos_valid = self.buoy_3_relposval

            self.buoy_3_pub.publish(buoy_3)

        if msg.topic == "vehicle/local_info":
            self.vehicle_north = payload["north"] * 1e-2
            self.vehicle_east = payload["east"] * 1e-2
            self.vehicle_down = - payload["down"] * 1e-2

        if msg.topic == "vehicle/status":
            self.vehicle_gnssfix = payload["gnssFixOK"]
            self.vehicle_carrsoln = payload["carrSoln"]
            self.vehicle_relposval = payload["relPosValid"]

            vehicle = RTK()
            vehicle.id = 9
            vehicle.header.stamp = self.get_clock().now().to_msg()
            vehicle.north = self.vehicle_north
            vehicle.east = self.vehicle_east
            vehicle.down = self.vehicle_down
            vehicle.gnss_fix_ok = self.vehicle_gnssfix
            vehicle.carr_soln = self.vehicle_carrsoln
            vehicle.rel_pos_valid = self.vehicle_relposval

            self.vehicle_pub.publish(vehicle)




def main():
    rclpy.init()
    node = mqtt_ros_adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()