import asyncio
from mavsdk import System

async def run():
    drone = System()
    # Connect to PX4 SITL running in Gazebo (UDP port 14540)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())