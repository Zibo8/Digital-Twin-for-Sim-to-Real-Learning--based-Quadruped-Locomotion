# Code originally provided by Hongpeng Cao.
# ------------------------------------------------------------------------------
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
import threading
import time
import numpy as np
from redis_sub import RedisSub

class RedisVel:
    """
    RedisVel class for subscribing to Redis channels, tracking position and computing velocity.
    """
    def __init__(self):
        # Initialize position (meters) and velocity (meters/second)
        # self.pos = [0,0,0.385]
        self.pos = [0,0,0.4]
        self.vel = np.zeros(3)

        # Timestamp of the last update; None until first message arrives
        self._last_time = None

        # Set up Redis subscriber
        self.redis_sub = RedisSub()
        self.redis_sub.init_subscriber()

        # Launch listener thread as daemon so it won’t block program exit
        self._listener = threading.Thread(
            target=self._redis_listener,
            daemon=True
        )
        self._listener.start()

        self.vel=[0,0,0]
        self.pos=[0,0,0]
        self.command=[0,0,0]
        self.quat_vicon=[0,0,0]
        self.raw_pos=[0,0,0]


    def _redis_listener(self):
        while True:
            states = self.redis_sub.subscribe_states()
            self.vel= states[0].get('vel')
            self.command= states[0].get('command')
            self.pos= states[0].get('position')
            self.raw_pos= states[0].get('translation')[0]
            self.quat_vicon= states[0].get('quaternion')[0]
            
def main():
    red = RedisVel()
    try:
        while True:
            # print('vel',red.vel)
            print('pos',red.quat_vicon)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting.")

if __name__ == "__main__":
    print("Start")
    main()
    print("End")
