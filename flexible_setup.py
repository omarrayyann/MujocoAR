from mujoco_ar import MujocoARConnector

connector = MujocoARConnector()
connector.start()

while True:
    data = connector.get_latest_data()
    print(data)