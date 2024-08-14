import asyncio
import websockets
import json
import numpy as np
import os
import signal
import psutil
import socket
import cv2
import threading
import mujoco
import warnings
import platform
import subprocess

class MujocoARConnector:
    """
    A connector to receive position and rotation data from a connected application.
    """

    def __init__(self, mujoco_model=None, mujoco_data=None, port=8888, debug=False):
        """
        Initialize the connector with the given port and other parameters.

        Args:
            port (int): The port on which the connector listens.
            mujoco_model: The MuJoCo model object.
            mujoco_data: The MuJoCo data object.
            debug (bool): Enable debug mode for verbose output.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        self.ip_address = s.getsockname()[0]
        self.port = port
        self.latest_data = {
            "rotation": None,
            "position": None,
            "button": None,
            "toggle": None,
        }
        self.server = None
        self.ping_interval = 20000000000
        self.ping_timeout = 10000000000
        self.connected_clients = set()
        self.debug = debug
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data
        self.reset_position_values = np.array([0.0, 0.0, 0.0])
        self.get_updates = True
        self.linked_frames = []

    async def _stop_server(self):
        """
        Stop the WebSocket server and close all active connections.
        """
        if self.server is not None:
            # Closing all connected clients
            for websocket in self.connected_clients:
                await websocket.close()
            # Stopping the server
            self.server.close()
            await self.server.wait_closed()
            self.server = None
            print("[INFO] MujocoARConnector Stopped")

    def stop(self):
        """
        Stop the MujocoARConnector.
        """
        self.loop.call_soon_threadsafe(asyncio.create_task, self._stop_server())

            
    def reset_position(self):
        """
        Reset the position to the current position, treating it as (0,0,0).
        """
        if self.latest_data["position"] is not None:
            self.reset_position_values += self.latest_data["position"]
            if self.debug:
                print(f"[INFO] Position reset to: {self.reset_position_values}")
    
    def pause_updates(self):
        """
        Pauses getting updates of data from the connected device.
        """
        self.get_updates = False
    
    def resume_updates(self):
        """
        Resumes getting updates of data from the connected device.
        """
        self.get_updates = True

    def _kill_process_using_port(self, port):
        """
        Kill the process using the given port on Unix-based systems.

        Args:
            port (int): The port to check for existing processes.
        """
        if platform.system() != "Windows":
            try:
                # Use lsof to find the process using the port
                command = f"lsof -t -i:{port}"
                pid = subprocess.check_output(command, shell=True).strip().decode()
                if pid:
                    os.kill(int(pid), signal.SIGKILL)
                    if self.debug:
                        print(f"[INFO] Killed process with PID {pid} using port {port}")
                else:
                    if self.debug:
                        print(f"[INFO] No process found using port {port}")
            except subprocess.CalledProcessError as e:
                if self.debug:
                    print(f"[ERROR] Failed to kill process using port {port}: {e}")
        else:
            os.system(f'taskkill /PID {pid} /F')
            
    async def _handle_connection(self, websocket, path):
        """
        Handle incoming connections and messages from the application.

        Args:
            websocket: The WebSocket connection.
            path: The URL path of the WebSocket connection.
        """
        print("[INFO] Application connected")
        self.connected_clients.add(websocket)
        try:
            async for message in websocket:
                if self.get_updates:
                    data = json.loads(message)
                    if 'rotation' in data and 'position' in data:
                        rotation = np.array(data['rotation'])
                        position = np.array(data['position'])
                        self.latest_data["rotation"] = rotation
                        self.latest_data["position"] = np.array([position[0],position[1],position[2]]).astype(float)
                        if self.reset_position_values is not None and self.latest_data["position"].dtype == self.reset_position_values.dtype:
                            self.latest_data["position"] -= self.reset_position_values
                        self.latest_data["button"] = data.get('button', False)
                        self.latest_data["toggle"] = data.get('toggle', False)
                    
                        for linked_frames in self.linked_frames:
                            linked_frames.update(self.mujoco_model, self.mujoco_data, self.latest_data.copy())

                        if self.debug:
                            print(f"[DATA] Rotation: {self.latest_data['rotation']}, Position: {self.latest_data['position']}, Button: {self.latest_data['button']}, Toggle: {self.latest_data['toggle']}")
        except websockets.ConnectionClosed as e:
            print(f"[INFO] Application disconnected: {e}")
        finally:
            self.connected_clients.remove(websocket)

    def set_mujoco(self, mujoco_model, mujoco_data):
        """
        Set the MuJoCo model and data.

        Args:
            mujoco_model: The MuJoCo model object.
            mujoco_data: The MuJoCo data object.
        """
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data

    async def _start(self):
        """
        Start the connector.
        """
        attempt = 0
        max_attempts = 10
        while attempt < max_attempts:
            self._kill_process_using_port(self.port)
            await asyncio.sleep(0.1)
            try:
                print(f"[INFO] MujocoARConnector Starting on port {self.port}...")
                self.server = await websockets.serve(self._handle_connection, "0.0.0.0", self.port,ping_interval=self.ping_interval,ping_timeout=self.ping_timeout)
                print(f"[INFO] MujocoARConnector Started. Details: \n[INFO] IP Address: {self.ip_address}\n[INFO] Port: {self.port}")
                break  # Exit the loop if server starts successfully
            except OSError as e:
                print(f"[WARNING] Port {self.port} is in use. Trying next port.")
                self.port += 1  # Increment the port number
                attempt += 1
        else:
            raise RuntimeError("Failed to start server on any port. Exceeded maximum attempts.")

        # Start the camera and control loops
        await self.server.wait_closed()
        
    def start(self):
        """
        Start the MujocoARConnector.
        """
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_event_loop).start()

    def _run_event_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._start())


    def get_latest_data(self):
        """
        Get the latest received data.

        Returns:
            dict: The latest rotation, position, and grasp data.
        """
        return self.latest_data
    
    def link_body(self, name, scale=1.0, position_origin=np.zeros(3), rotation_origin=np.identity(3), pose_transform=np.identity(4), toggle_fn=None, button_fn=None, disable_pos=False, disable_rot=False):
        """
        Adds a linked body to be directly controlled by the AR data as opposed to you doing it manually.

        Args:
            body_name (str): The name of the body in MuJoCo.
            scale (float): Scalar number to multiply the positions from AR kit by
            position_origin (numpy array of shape (3)): Translation to be done on the retrived pose (done after scaling if scale is not 1.0)
            rotation_origin (numpy array of shape (3,3)): Rotation to be done on the retrived pose relative to the global frame (done after scaling if scale is not 1.0)
            pose_transform (numpy array of shape (4,4)): Transform the recieved pose by this pose upon recieving it
            toggle_fn (bool): a function to call when the toggle is toggled (no args)
            button_fn (function): a function to call when the button is pressed (no args)
            disable_pos (bool): if true, does not update the position of the geom
            disable_rot (bool): if true, does not update the rotation of the geom

        """

        if self.mujoco_model is None or self.mujoco_data is None:
            raise ValueError("Must set the MuJoCo model and data to have a linked body.")

        body_id =  mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_BODY, name)

        linked_body = LinkedFrame(
            id=body_id,
            frame_type=0,
            scale=scale,
            position_origin=position_origin,
            rotation_origin=rotation_origin,
            pose_transform=pose_transform,
            toggle_fn=toggle_fn,
            button_fn = button_fn,
            disable_pos = disable_pos,
            disable_rot = disable_rot,
        )
        
        self.linked_frames.append(linked_body)

        if self.mujoco_model is None or self.mujoco_data is None:
            warnings.warn("Must set the MuJoCo model and data to have the linked body move.")
            
    def link_site(self, name, scale=1.0, position_origin=np.zeros(3), rotation_origin=np.identity(3), pose_transform=np.identity(4), toggle_fn=None, button_fn=None, disable_pos=False, disable_rot=False):
        """
        Adds a linked site to be directly controlled by the AR data as opposed to you doing it manually.

        Args:
            site_name (str): The name of the site in MuJoCo.
            scale (float): Scalar number to multiply the positions from AR kit by
            position_origin (numpy array of shape (3)): Translation to be done on the retrived pose (done after scaling if scale is not 1.0)
            rotation_origin (numpy array of shape (3,3)): Rotation to be done on the retrived pose relative to the global frame (done after scaling if scale is not 1.0)
            pose_transform (numpy array of shape (4,4)): Transform the recieved pose by this pose upon recieving it
            toggle_fn (bool): a function to call when the toggle is toggled (no args)
            button_fn (function): a function to call when the button is pressed (no args)
            disable_pos (bool): if true, does not update the position of the geom
            disable_rot (bool): if true, does not update the rotation of the geom

        """

        if self.mujoco_model is None or self.mujoco_data is None:
            raise ValueError("Must set the MuJoCo model and data to have a linked site.")

        site_id =  mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_SITE, name)
        linked_site = LinkedFrame(
            id=site_id,
            frame_type = 1,
            scale=scale,
            position_origin=position_origin,
            rotation_origin=rotation_origin,
            pose_transform=pose_transform,
            toggle_fn=toggle_fn,
            button_fn = button_fn,
            disable_pos = disable_pos,
            disable_rot = disable_rot,
        )
        
        self.linked_frames.append(linked_site)

        if self.mujoco_model is None or self.mujoco_data is None:
            warnings.warn("Must set the MuJoCo model and data to have the linked site move.")

    def link_geom(self, name, scale=1.0, position_origin=np.zeros(3), rotation_origin=np.identity(3), pose_transform=np.identity(4), toggle_fn=None, button_fn=None, disable_pos=False, disable_rot=False):
        """
        Adds a linked geom to be directly controlled by the AR data as opposed to you doing it manually.

        Args:
            geom_name (str): The name of the geom in MuJoCo.
            scale (float): Scalar number to multiply the positions from AR kit by
            position_origin (numpy array of shape (3)): Translation to be done on the retrived pose (done after scaling if scale is not 1.0)
            rotation_origin (numpy array of shape (3,3)): Rotation to be done on the retrived pose relative to the global frame (done after scaling if scale is not 1.0)
            pose_transform (numpy array of shape (4,4)): Transform the recieved pose by this pose upon recieving it
            toggle_fn (bool): a function to call when the toggle is toggled (no args)
            button_fn (function): a function to call when the button is pressed (no args)
            disable_pos (bool): if true, does not update the position of the geom
            disable_rot (bool): if true, does not update the rotation of the geom

        """

        if self.mujoco_model is None or self.mujoco_data is None:
            raise ValueError("Must set the MuJoCo model and data to have a linked geom.")

        geom_id =  mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_GEOM, name)
        linked_geom = LinkedFrame(
            id=geom_id,
            frame_type = 2,
            scale=scale,
            position_origin=position_origin,
            rotation_origin=rotation_origin,
            pose_transform=pose_transform,
            toggle_fn=toggle_fn,
            button_fn = button_fn,
            disable_pos = disable_pos,
            disable_rot = disable_rot,
        )
        
        self.linked_frames.append(linked_geom)

        if self.mujoco_model is None or self.mujoco_data is None:
            warnings.warn("Must set the MuJoCo model and data to have the linked geom move.")

class LinkedFrame:
    
    def __init__(self, id, frame_type, scale, position_origin, rotation_origin, pose_transform, toggle_fn, button_fn, disable_pos, disable_rot):
        self.id = id
        self.frame_type = frame_type
        self.scale = scale
        self.position_origin = position_origin
        self.rotation_origin = rotation_origin
        self.pose_transform=pose_transform
        self.button_fn = button_fn
        self.toggle_fn = toggle_fn
        self.disable_pos = disable_pos
        self.disable_rot = disable_rot
        self.last_toggle = False

    def update(self, mujoco_model, mujoco_data, latest_data):

        pose = np.identity(4)
        pose[0:3,0:3] = latest_data["rotation"]
        pose[0:3,3] = latest_data["position"]

        # Applying scale
        pose[0:3,3] = pose[0:3,3]*self.scale
        # Applying translation
        if self.position_origin is not None:
            pose[0:3,3] += self.position_origin
        # Applying rotation
        pose[0:3,0:3] = pose[0:3,0:3].T@self.rotation_origin

        # Applying transformation
        pose = self.pose_transform @ pose

        # Updating Toggle Button Variable 
        if latest_data["toggle"] is not None and self.toggle_fn is not None and self.last_toggle != latest_data["toggle"]:
            self.toggle_fn()
            self.last_toggle = latest_data["toggle"]

        # Calling button function if pressed
        if latest_data["button"] and self.button_fn is not None:
            self.button_fn()
        
        # Converting the rotation matrix to quaternion
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, pose[0:3,0:3].flatten())

        if self.frame_type == 0: # body
            mocap_id = mujoco_model.body(self.id).mocapid[0]
            if mocap_id != -1:
                if not self.disable_rot:
                    mujoco_data.mocap_quat[mocap_id] = quat
                if not self.disable_pos:
                    mujoco_data.mocap_pos[mocap_id] = pose[0:3,3].tolist()
            else:
                if not self.disable_rot:
                    mujoco_model.body_quat[self.id] = quat
                if not self.disable_pos:
                    mujoco_model.body_pos[self.id] = pose[0:3,3].tolist()

        elif self.frame_type == 1: # site
            if not self.disable_rot:
                mujoco_model.site_quat[self.id] = quat
            if not self.disable_pos:
                mujoco_model.site_pos[self.id] = pose[0:3,3].tolist()

        elif self.frame_type == 2: # geom
            if not self.disable_rot:
                mujoco_model.geom_quat[self.id] = quat
            if not self.disable_pos:
                mujoco_model.geom_pos[self.id] = pose[0:3,3].tolist()

