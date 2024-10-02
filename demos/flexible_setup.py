from mujoco_ar import MujocoARConnector

connector = MujocoARConnector()
connector.start()

while True:
    data = connector.get_latest_data()
    if data["position"] is not None:
        print(data)