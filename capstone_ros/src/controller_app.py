#!/usr/bin/env python
# ****************************************************************************
# CAPSTONE G03, 2018-2019
#
# License:
#   This software is available under the Apache-2.0 license.
#
# Repository:
#   Github: https://github.com/rahmant3/capstoneROS2018
#
# Description:
#   The main controller application for capstone G03. In charge of subscribing
#   to diagnostic data, and then POSTing to the web service.
#
#   See the readme.md file for more information about setting up the metadata
#   file for this script.
#
# History:
#   2018-11-14 by Tamkin Rahman
#   - Created.
#   2018-12-14 by Tamkin Rahman
#   - Handles metadata and parameter tables through the use of a JSON file.
#   2018-12-21 by Tamkin Rahman
#   - Add additional ROS types handling for Bool and Float64.
# ****************************************************************************

# ****************************************************************************
#   IMPORTS
# ****************************************************************************
import json
import os
import rospy
import urllib
import urllib2

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# ****************************************************************************
#   CONSTANTS
# ****************************************************************************
MAX_QUEUE_SIZE = 10

VERSION_STRING = "0.01"

META_DATA_ENV = 'CONTROLLER_APP_METADATA_PATH'
DEFAULT_META_DATA_FPATH = "controller_app_metadata.json"

# Mapping from string to ROS type.
ROS_TYPES_TABLE = { 'string': String,
                    'bool':   Bool,
                    'float64': Float64
                  }

# ****************************************************************************
#   GLOBALS
# ****************************************************************************
controller_params_mgr = None

# ****************************************************************************
#   CLASSES
# ****************************************************************************
class ControllerParams:

    def __init__(self, vehicle_id, post_url, api_key):
        self.vehicle_id = vehicle_id
        self.post_url = post_url
        self.api_key = api_key

        self.json_data = { 'id': self.vehicle_id,
                           'key': self.api_key,
                           'version': VERSION_STRING,
                           'params': []
                           }
        self.param_dict = {}

    def _post_data(self):
        ''' Private function. Makes a POST request to the Web Service.

            Returns:
                True on success, False on failure.
        '''

        rc = False

        req = urllib2.Request(self.post_url)
        req.add_header('Content-Type', 'application/json')

        try:
            response = urllib2.urlopen(req, json.dumps(self.json_data))
            rc = True

        except urllib2.HTTPError as e:
            print("HTTP Error occurred on POST, with error code: {0}".format(str(e.code)))
        except urllib2.URLError as e:
            print("URL Error occurred on POST, with reason: {0}".format(e.reason))
        except:
            print("Unknown Error occurred on POST.")

        return rc

    def _update_json_data(self):
        ''' Private function. Prepares the internal "json" dictionary so that
            json.dumps can be used to generate the JSON string.
        '''

        self.json_data['params'] = []

        for key in self.param_dict:
            if (self.param_dict[key][1] is not None):
                self.json_data['params'].append({ 'name':      key,
                                                  'id':        self.param_dict[key][0],
                                                  'value':     self.param_dict[key][1],
                                                  'timestamp': self.param_dict[key][2],
                                                  'message':   self.param_dict[key][3],
                                                  'type':      self.param_dict[key][4],
                                                  'units':     self.param_dict[key][5]
                                                  })

    def add_param(self, name, id, value, type, units, timestamp, message=""):
        ''' Add the given parameter to the internal parameter library. A value of
            None will prevent this parameter from being transmitted to the web service.

            Parameters:
                name (string):    The name of parameter.
                id (int):         ID for the parameter.
                value (string):   Value of the parameter.
                type (string):    The type for the parameter.
                units (string):   The units for the parameter.
                timestamp (int):  Timestamp (seconds since epoch) for when the
                                  value was last updated.
                message (string): Optional message for the parameter.
        '''

        self.param_dict[name] = [id, value, timestamp, message, type, units]

    def update_param(self, name, value, timestamp, message=""):
        ''' Updates the given parameter in the internal parameter library.

            Parameters:
                name (string):    The name of the parameter to update.
                value (string):   The value to update with.
                timestamp (int):  Timestamp (seconds since epoch) for when the
                                  value was last updated.
                message (string): Optional message for the parameter.
        '''
        rc = False
        if (self.param_dict[name][1] != value):
            rc = True
            self.param_dict[name][1] = value

        self.param_dict[name][2] = timestamp
        self.param_dict[name][3] = message

        return rc

    def get_param_value(self, name):
        ''' Returns the data associated with the parameter.

            Parameters:
                name (string): The name of the parameter to get.

            Returns:
                The current value for parameter.
        '''

        return self.param_dict[name][1]

    def post_data(self):
        self._update_json_data()

        #print(json.dumps(self.json_data))

        if (len(self.json_data['params']) > 0):
            self._post_data()

# ****************************************************************************
#   FUNCTIONS
# ****************************************************************************
def diagnostic_data_received(data, cb_args=None):
    '''
        Callback for ROS subscriber data. Update the internal tables with the
        new timestamp, and then POST to the web service.

        Parameters:
            data : The Message class containing the data.
            cb_args: The callback arguments, indicating the data type.
    '''
    global controller_params_mgr

    timestamp = int(rospy.get_time())

    # Update param with str(data.data) and timestamp.
    # Post on changes.
    if (controller_params_mgr.update_param(cb_args, str(data.data), timestamp)):
        controller_params_mgr.post_data()

    rospy.loginfo(rospy.get_caller_id() + "I heard %s with arg %s",
                  data.data, cb_args)

# ****************************************************************************
def metadata_init(filename):
    '''
        Sets up the variables needed for initializing the Controller App
        manager.

        Parameters:
            filename: The path to the JSON file containing the metadata.
        Returns:
            A list, containing the following information (in this order):
            0. Vehicle ID (int)
            1. POST url (string)
            2. API key (string)
            3. Params file path (string)
    '''
    data = []

    try:
        f = open(filename, 'rb')
        content = f.read()

        json_content = json.loads(content)

        data.append(json_content.get('vehicle_id', None))
        data[0] = int(data[0]) # If this fails, a Value error will occur.

        data.append(json_content.get('url', None))
        data.append(json_content.get('api_key', None))
        data.append(json_content.get('params', None))

    except IOError:
        print("Failed to open file: \"{0}\"".format(filename))
    except ValueError:
        print("Failed to decode JSON text.")

    return data

# ****************************************************************************
def ros_init(params):
    '''
        Initializes the ROS subscribers and Controller Manager based on the
        given parameters from the metadata file.
        
        Parameters:
            params: List of dictionaries containing the metadata for each
                    parameter.
        Returns:
            True on success, False on failure.
    '''
    
    global controller_params_mgr

    rc = True
    # Reference: http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html
    #            http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    rospy.init_node('controller_app_listener', anonymous=True)

    for entry in params:
        print(entry)
        try:
            controller_params_mgr.add_param(entry['key'],
                                            int(entry['id']),
                                            None,
                                            entry['param_type'],
                                            entry['units'],
                                            None)

            rospy.Subscriber(entry['ros_path'],
                             ROS_TYPES_TABLE[entry['ros_type'].lower()],
                             diagnostic_data_received,
                             entry['key'],
                             MAX_QUEUE_SIZE
                             )

        except KeyError:
            print("Failed to add a parameter due to poorly formed JSON.")
            rc = False
            break

    return rc

# ****************************************************************************
#   ENTRY POINT
# ****************************************************************************
if __name__ == '__main__':
    meta_data_file = os.getenv(META_DATA_ENV, DEFAULT_META_DATA_FPATH)
    data = metadata_init(meta_data_file)

    if ((len(data) == 4) and (None not in data)):
        controller_params_mgr = ControllerParams(data[0], data[1], data[2])

        if (ros_init(data[3])):
            # spin() simply keeps python from exiting until this node is stopped
            rospy.spin()

