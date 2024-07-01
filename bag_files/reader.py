import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class Reader():

    def __init__(self, bag_file: str):
        self.bag_file = bag_file

    def _read_topic(self, selected_topic: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self.bag_file, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"),
        )
        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f'topic {topic_name} not in bag')

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic != selected_topic:
                continue
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        del reader

    def get_data(self, topic: str):
        return [[x[1], x[2]] for x in self._read_topic(topic)]
