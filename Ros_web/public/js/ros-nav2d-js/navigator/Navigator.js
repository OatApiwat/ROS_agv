/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * tfClient (optional) - the TF client
 *   * robot_pose (optional) - the robot topic or TF to listen position
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 */
NAV2D.Navigator = function (options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var tfClient = options.tfClient || null;
  var robot_pose = options.robot_pose || "/robot_pose";
  var serverName = options.serverName || "/move_base";
  var actionName = options.actionName || "move_base_msgs/MoveBaseAction";
  var withOrientation = options.withOrientation || false;
  var use_image = options.image;
  this.rootObject = options.rootObject || new createjs.Container();

  this.goalMarker = null;

  var currentGoal;

  // setup the actionlib client
  var actionClient = new window.ROSLIB.ActionClient({
    ros: ros,
    actionName: actionName,
    serverName: serverName,
  });

  /**
   * Subscribe to the goal position topic and display the goal marker.
   */
  var subscribeToGoalTopic = function () {
    // setup a listener for the goal position
    var goalListener = new window.ROSLIB.Topic({
      ros: ros,
      name: '/move_base/current_goal',
      messageType: 'geometry_msgs/PoseStamped'
    });

    goalListener.subscribe(function (message) {
      
      var goalPose = message.pose;
      var pose = new window.ROSLIB.Pose({
        position: goalPose.position,
        orientation: goalPose.orientation
      });

      // Convert ROS coordinates to global coordinates used by createjs
      var globalCoords = rosToGlobalCoords(pose.position);
      console.log(rosToGlobalCoords.x)
      // Remove old goal marker if exists
      if (that.goalMarker !== null) {
        that.rootObject.removeChild(that.goalMarker);
      }

      // Create new goal marker
      that.goalMarker = createGoalMarker(message.pose.position.x, message.pose.position.y);
      that.rootObject.addChild(that.goalMarker);
    });
  };

  // Subscribe to the goal topic when the navigator is instantiated
  subscribeToGoalTopic();

  /**
   * Create a marker for the goal position.
   *
   * @param x - x-coordinate of the goal position
   * @param y - y-coordinate of the goal position
   * @returns {createjs.Shape} - the goal marker
   */
  var createGoalMarker = function (x, y) {
    var marker;
    if (use_image && window.ROS2D.hasOwnProperty("ImageNavigator")) {
      marker = new window.ROS2D.ImageNavigator({
        size: 2.5,
        image: use_image,
        alpha: 0.7,
        pulse: false,
      });
    } else {
      marker = new window.ROS2D.NavigationArrow({
        size: 15,
        strokeSize: 1,
        fillColor: createjs.Graphics.getRGB(255, 64, 128, 0.66),
        pulse: false,
      });
    }
    marker.x = x;
    marker.y = y;
    marker.scaleX = 1.0 / stage.scaleX;
    marker.scaleY = 1.0 / stage.scaleY;

    return marker;
  };

  /**
   * Convert ROS coordinates to global coordinates used by createjs.
   * Replace this function with your actual conversion logic.
   */
  var rosToGlobalCoords = function (rosCoords) {
    // Implement your own logic to convert ROS coordinates to global coordinates
    // This will depend on how your map and coordinate system are set up.
    // Example:
    var scaleX = 1.0 / stage.scaleX;
    var scaleY = 1.0 / stage.scaleY;
    var globalX = rosCoords.x * scaleX;
    var globalY = -rosCoords.y * scaleY; // Invert Y-axis if needed

    return { x: globalX, y: globalY };
  };

  /**
   * Cancel the currently active goal.
   */
  this.cancelGoal = function () {
    console.log("Cancel Goals");
    if (typeof that.currentGoal !== "undefined") {
      console.log(that.currentGoal);
      that.currentGoal.cancel();
    }

    // Remove goal marker if exists
    if (that.goalMarker !== null) {
      that.rootObject.removeChild(that.goalMarker);
      that.goalMarker = null;
    }
  };

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = null;
  if (use_image && window.ROS2D.hasOwnProperty("ImageNavigator")) {
    robotMarker = new window.ROS2D.ImageNavigator({
      size: 2.5,
      image: use_image,
      pulse: false,
    });
  } else {
    robotMarker = new window.ROS2D.NavigationArrow({
      size: 25,
      strokeSize: 1,
      fillColor: createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse: false,
    });
  }

  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  var updateRobotPosition = function (pose, orientation) {
    // update the robots position on the map
    robotMarker.x = pose.x;
    robotMarker.y = -pose.y;
    console.log(initScaleSet);
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }
    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(orientation);
    // Set visible
    robotMarker.visible = true;
  };

  if (tfClient !== null) {
    tfClient.subscribe(robot_pose, function (tf) {
      updateRobotPosition(tf.translation, tf.rotation);
    });
  } else {
    // setup a listener for the robot pose
    var poseListener = new window.ROSLIB.Topic({
      ros: ros,
      name: robot_pose,
      messageType: "geometry_msgs/Pose",
      throttle_rate: 100,
    });
    poseListener.subscribe(function (pose) {
      updateRobotPosition(pose.position, pose.orientation);
    });
  }

  if (withOrientation === false) {
    // setup a double click listener (no orientation)
    this.rootObject.addEventListener("dblclick", function (event) {
      // convert to ROS coordinates
      var coords = stage.globalToRos(event.stageX, event.stageY);
      var pose = new window.ROSLIB.Pose({
        position: new window.ROSLIB.Vector3(coords),
      });
      // send the goal
      // sendGoal(pose);
    });
  } else {
    // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var positionVec3 = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function (event, mouseState) {
      if (mouseState === "down") {
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new window.ROSLIB.Vector3(position);
        mouseDown = true;
      } else if (mouseState === "move") {
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);

        if (mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new window.ROSLIB.Vector3(currentPos);

          if (use_image && window.ROS2D.hasOwnProperty("ImageNavigator")) {
            orientationMarker = new window.ROS2D.ImageNavigator({
              size: 2.5,
              image: use_image,
              alpha: 0.7,
              pulse: false,
            });
          } else {
            orientationMarker = new ROS2D.NavigationArrow({
              size: 25,
              strokeSize: 1,
              fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.66),
              pulse: false,
            });
          }

          xDelta = currentPosVec3.x - positionVec3.x;
          yDelta = currentPosVec3.y - positionVec3.y;

          thetaRadians = Math.atan2(xDelta, yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);

          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          orientationMarker.x = positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 1.0 / stage.scaleX;
          orientationMarker.scaleY = 1.0 / stage.scaleY;

          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseDown) {
        // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        var goalPos = stage.globalToRos(event.stageX, event.stageY);

        var goalPosVec3 = new window.ROSLIB.Vector3(goalPos);

        xDelta = goalPosVec3.x - positionVec3.x;
        yDelta = goalPosVec3.y - positionVec3.y;

        thetaRadians = Math.atan2(xDelta, yDelta);

        if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
          thetaRadians += (3 * Math.PI) / 2;
        } else {
          thetaRadians -= Math.PI / 2;
        }

        var qz = Math.sin(-thetaRadians / 2.0);
        var qw = Math.cos(-thetaRadians / 2.0);

        var orientation = new window.ROSLIB.Quaternion({ x: 0, y: 0, z: qz, w: qw });

        var pose = new window.ROSLIB.Pose({
          position: positionVec3,
          orientation: orientation,
        });
        // send the goal
        // sendGoal(pose);
      }
    };

    this.rootObject.addEventListener("stagemousedown", function (event) {
      mouseEventHandler(event, "down");
    });

    this.rootObject.addEventListener("stagemousemove", function (event) {
      mouseEventHandler(event, "move");
    });

    this.rootObject.addEventListener("stagemouseup", function (event) {
      mouseEventHandler(event, "up");
    });
  }
};
