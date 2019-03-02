# Capstone ROS, 2018-2019
Contains the scripts for the controller subsystem for capstone G03 (that can be publicly shared).

The scripts are kept as the package name "capstone_ros", under the "src" folder. See file headers for descriptions. The scripts consist of 3 ROS nodes, each of which do the following:
- Publishes all received CAN messages from the controller to a ROS topic
- Interprets the CAN messages from the ROS topic, and then publishes diagnostic data to various diagnostic topics
- Subscribes to the diagnostic data and POSTS the data to a web service.

The utils folder contains useful files for testing the ROS nodes.

## Metadata File
In order to use the Controller Application, a metadata file is needed. The format of the metadata file is a JSON file (as shown below):
```
{
    "_comments": "Contains metadata for the capstone controller application.",

    "url": "http://sample/api",
    "vehicle_id": 4,
    "api_key": "abcdef1234568",

    "params": [
        {
            "key": "Param A",
            "ros_path": "oil_level",
            "ros_type": "String",
            "id": 0,
            "param_type": "float",
            "units": "mm"
        },
        ...
    ]
}
```

The absolute location of the metadata file should be in the environment variable 'CONTROLLER_APP_METADATA_PATH'. The environment variable can be set like so on the command line or from a script:
```
export CONTROLLER_APP_METADATA_PATH="/home/user/metadata.json"
```

The metadata file used for the Controller Application can be found in the <package-name>/src folder.