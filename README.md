# AMR logistics automation system



### Consumer Order GUI
1. ***Add database config file***
    ```db/config/config.ini```
```
[database] 
host = localhost
user = root
password = your_password
database = your_database
```

2. ***Install Dependencies***
```
pip install websocket-client
pip install PyQt5 websocket-client
```
3. ***Execute***
<br>
3-1) ros2 and websocket <br>
3-2) Main GUI
```
source .install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```
python3 gui/consumer/src/ui_main.py
```


