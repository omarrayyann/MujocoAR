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

class MujocoARConnector:
    """
    A connector to receive position and rotation data from a connected application.
    """

    def __init__(self, port=8888, mujoco_model=None, mujoco_data=None, camera_name=None, camera_resolution=(240, 320), controls_frequency=100, camera_frequency=1, debug=False):
        """
        Initialize the connector with the given port and other parameters.

        Args:
            port (int): The port on which the connector listens.
            mujoco_model: The MuJoCo model object.
            mujoco_data: The MuJoCo data object.
            camera_name (str): The name of the camera to use.
            camera_resolution (tuple): The resolution of the camera.
            controls_frequency (int): The frequency of control updates.
            camera_frequency (int): The frequency of camera updates.
            debug (bool): Enable debug mode for verbose output.
        """
        self.ip_address = socket.gethostbyname(socket.gethostname())
        self.port = port
        self.latest_data = {
            "rotation": None,
            "position": None,
            "grasp": None
        }
        self.server = None
        self.connected_clients = set()
        self.debug = debug
        self.mujoco_model = mujoco_model
        self.mujoco_data = mujoco_data
        self.camera_name = camera_name
        self.controls_frequency = controls_frequency
        self.camera_frequency = camera_frequency
        self.camera_resolution = camera_resolution
        self.reset_position_values = np.array([0.0, 0.0, 0.0])
        self.get_updates = True


        if camera_name is not None and (mujoco_model is None or mujoco_data is None):
            warnings.warn("Must set the MuJoCo model and data to have the camera transmission work.")

    def reset_position(self):
        """
        Reset the position to the current position, treating it as (0,0,0).
        """
        self.reset_position_values = self.latest_data["position"]
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
        Kill the process using the given port.

        Args:
            port (int): The port to check for existing processes.
        """
        for proc in psutil.process_iter(['pid', 'name', 'username']):
            try:
                for conns in proc.connections(kind='inet'):
                    if conns.laddr.port == port:
                        os.kill(proc.info['pid'], signal.SIGKILL)
                        if self.debug:
                            print(f"[INFO] Killed process {proc.info['name']} with PID {proc.info['pid']} using port {port}")
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                pass

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
                print(self.get_updates)
                if self.get_updates:
                    data = json.loads(message)
                    if 'rotation' in data and 'position' in data:
                        self.latest_data["rotation"] = np.array(data['rotation']) 
                        self.latest_data["position"] = np.array([-data['position'][2],-data['position'][0],data['position'][1]])
                        if self.reset_position_values is not None:
                            self.latest_data["position"] -= self.reset_position_values
                        self.latest_data["grasp"] = data.get('grasp', False)
                        if self.debug:
                            print(f"[DATA] Rotation: {self.latest_data['rotation']}, Position: {self.latest_data['position']}, Grasp: {self.latest_data['grasp']}")
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

    def get_last_camera_frame(self):
        """
        Get the last camera frame.

        Returns:
            np.ndarray: The last camera frame in BGR format.
        """
        renderer = mujoco.Renderer(self.mujoco_model, height=self.camera_resolution[0], width=self.camera_resolution[1])
        renderer.update_scene(self.mujoco_data, self.camera_name)
        frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        return frame

    def set_camera(self, camera_name):
        """
        Set the camera name.

        Args:
            camera_name (str): The name of the camera.
        """
        if self.mujoco_model is None or self.mujoco_data is None:
            warnings.warn("Must set the MuJoCo model and data to have the camera transmission working.")
        self.camera_name = camera_name    

    async def _send_frame(self):
        """
        Send an image to all connected applications.
        """
        while True:
            if self.connected_clients and self.camera_name and self.mujoco_model and self.mujoco_data:
                image_array = self.get_last_camera_frame()
                image_bytes = cv2.imencode('.jpg', image_array)[1].tobytes()
                await asyncio.wait([client.send(image_bytes) for client in self.connected_clients])
            await asyncio.sleep(1 / self.camera_frequency)

    async def _control_update(self):
        """
        Update controls at the specified frequency.
        """
        while True:
            # Handle control updates here
            await asyncio.sleep(1 / self.controls_frequency)

    async def _start(self):
        """
        Start the connector.
        """
        self._kill_process_using_port(self.port)
        print(f"[INFO] MujocoARConnector Starting...")
        self.server = await websockets.serve(self._handle_connection, "0.0.0.0", self.port)
        print(f"[INFO] MujocoARConnector Started. Details: \n[INFO] IP Address: {self.ip_address}\n[INFO] Port: {self.port}")

        # Start the camera and control loops
        await asyncio.gather(self._send_frame(), self._control_update(), self.server.wait_closed())

    def start(self):
        """
        Start the MujocoARConnector.
        """
        threading.Thread(target=lambda: asyncio.run(self._start())).start()

    def get_latest_data(self):
        """
        Get the latest received data.

        Returns:
            dict: The latest rotation, position, and grasp data.
        """
        return self.latest_data
