# A script which simulates a drone following a path planned by the pathgen

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import pathgen



async def connect(drone: System):
    '''
    Connects the drone and awaits for gps fix
    '''
    await drone.connect()

    print("Drone is connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone!")
            break

    print("Waiting for position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position state is valid")
            break


def mission_from_rectangles(rectangles: list, altitude, speed):
    """
    Takes a list of Rectangle objects and returns a mission plan taking a photo at each rectangle
    """
    
    mission_items = []

    # iterate over every point
    for r in rectangles:
        mission_items.append(MissionItem(
            r.center[0], r.center[1], altitude, speed, 
            False, # is_fly_through False means it will stop at each point
            0, # 0 gimbal pitch
            r.yaw, # 0 gimbal yaw
            MissionItem.CameraAction.TAKE_PHOTO, # take one photo
            0, # no loiter time
            0, # no cam photo interval
            0.5, # small acceptance radius (m)
            0, # yaw angle 0
            0 # no cam photo distance
        ))

    return MissionPlan(mission_items)



async def run():
    print('running')
    drone = System()

    # connect to the drone and wait for gps fix
    await connect(drone)

    # get the drone's starting position
    async for position in drone.telemetry.position():
        start_pos = (position.latitude_deg, position.longitude_deg)
        print(f"Drone position is ({start_pos[0]}, {start_pos[1]})")
        break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    # segment the workspace to get points for the mission
    perimeter = [
        (37.76966, -119.60218), (37.77075, -119.59952),
        (37.77025, -119.59359), (37.76803, -119.59862),
        (37.76768, -119.59536), (37.76559, -119.59900)
    ]
    workplace = pathgen.Workplace(
        start_pos=(43.679782271987395, -70.2692889874136), 
        fov=(62.2, 48.8),   # the rpi cam 2 FOV 
        altitude=10, 
        perimeter=perimeter
    )

    workplace.print_grid(workplace.potential_field)

    # build the mission
    mission_plan = mission_from_rectangles(
        rectangles=workplace.path,
        altitude=10,
        speed=1
    )

    await drone.mission.set_return_to_launch_after_mission(True)

    # upload the mission
    await drone.mission.upload_mission(mission_plan)

    print("arming!")
    await drone.action.arm()

    print("starting mission")
    await drone.mission.start_mission()

    await termination_task

    print("Mission complete.")



async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())