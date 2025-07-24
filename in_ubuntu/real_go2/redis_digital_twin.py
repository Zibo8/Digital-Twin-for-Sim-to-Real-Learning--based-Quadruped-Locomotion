# Code originally provided by Hongpeng Cao.
# ------------------------------------------------------------------------------
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
import threading
import time
import numpy as np
from redis_sub import RedisSubDigitalTwin as RedisSub

class RedisDigitalTwin:
    """
    RedisVel class for subscribing to Redis channels, tracking position and computing velocity.
    """
    def __init__(self):
        # Set up Redis subscriber
        self.redis_sub = RedisSub()
        self.redis_sub.init_subscriber()

        # Launch listener thread as daemon so it won’t block program exit
        self._listener = threading.Thread(
            target=self._redis_listener,
            daemon=True
        )
        self._listener.start()

        # initial data
        self.joint_pos = np.array([0.35,-0.35,0.5,-0.5,1.36,1.36,1.36,1.36,-2.65,-2.65,-2.65,-2.65])  
        self.quaternion = [1, 0, 0, 0]

    def _redis_listener(self):
        while True:
            states = self.redis_sub.subscribe_states()
            self.joint_pos= states[0].get('joint_positions')
            self.quaternion = states[0].get('quaternion')

  
def main():
    red = RedisDigitalTwin()
    try:
        while True:
            # print('joint_pos',red.joint_pos)
            print('quaternion',red.quaternion)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")

if __name__ == "__main__":
    print("Start")
    main()
    print("End")
