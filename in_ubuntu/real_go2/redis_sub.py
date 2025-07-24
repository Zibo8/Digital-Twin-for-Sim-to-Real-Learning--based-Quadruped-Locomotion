# Code originally provided by Hongpeng Cao.
# ------------------------------------------------------------------------------
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
import pickle
import redis

class RedisSub:
    """
    RedisSub class for subscribing to Redis channels and receiving robot states.
    """
    def __init__(self):
        # Initialize a Redis ConnectionPool for efficient connection reuse
        self.pool = redis.ConnectionPool(host="localhost", port=6379, password='ubuntu')
        # Define the Redis channel name for publishing/subscribing car states
        self.redis_channel_states = 'channel_pose'
        # Create a Redis client using the connection pool
        self.conns = redis.Redis(connection_pool=self.pool)
        # Placeholder for the PubSub subscriber; will be initialized when needed
        self.substates = None

    def init_subscriber(self):
        # Create a PubSub object for subscribing to channels
        self.substates = self.conns.pubsub()
        # Subscribe to the car states channel
        self.substates.subscribe(self.redis_channel_states)
        # Wait for subscription confirmation before proceeding
        self.substates.parse_response()

    def subscribe_states(self):
        # Block and wait for the next message; parse_response returns [type, channel, message]
        states_pack = self.substates.parse_response()[2]
        # Deserialize the byte stream back into the original Python object
        states = pickle.loads(states_pack)
        return states
    
class RedisSubDigitalTwin:
    """
    RedisSub class for subscribing to Redis channels and receiving robot states.
    """
    def __init__(self):
        # Initialize a Redis ConnectionPool for efficient connection reuse
        self.pool = redis.ConnectionPool(host="localhost", port=6379, password='ubuntu')
        # Define the Redis channel name for publishing/subscribing car states
        self.redis_channel_states = 'channel_digital_twin'
        # Create a Redis client using the connection pool
        self.conns = redis.Redis(connection_pool=self.pool)
        # Placeholder for the PubSub subscriber; will be initialized when needed
        self.substates = None

    def init_subscriber(self):
        # Create a PubSub object for subscribing to channels
        self.substates = self.conns.pubsub()
        # Subscribe to the car states channel
        self.substates.subscribe(self.redis_channel_states)
        # Wait for subscription confirmation before proceeding
        self.substates.parse_response()

    def subscribe_states(self):
        # Block and wait for the next message; parse_response returns [type, channel, message]
        states_pack = self.substates.parse_response()[2]
        # Deserialize the byte stream back into the original Python object
        states = pickle.loads(states_pack)
        return states
