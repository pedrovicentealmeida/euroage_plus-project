#!/usr/bin/env python3

import threading
import rclpy
from game_library import StoryTelling

rclpy.init(args=None)
ST = StoryTelling()


def main():
    
    try:

        while True:
            
            ros_thread = threading.Thread(target=rclpy.spin, args=(ST,))
            ros_thread.start()

            ST.define_parameters("Pedro", 90, "Nulo", "Falar com os netos", "Carpinteiro", "Vitor:Neto", "Passeio pela praia", "Morte da esposa")
            
            while True:
                ST.obtain_response()

                ST.new_message()
    
    finally:
        # Cleanup on exit
        ST.shutdown_hook()
        ST.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
