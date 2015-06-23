import rospy


class PublisherFactory:
    publishers = {}
 
    @staticmethod
    def get_publisher(topic, data_class, queue_size=None):
        if topic not in PublisherFactory.publishers:
            PublisherFactory.publishers[topic] = rospy.Publisher(topic, data_class, queue_size=queue_size)
        return PublisherFactory.publishers[topic]