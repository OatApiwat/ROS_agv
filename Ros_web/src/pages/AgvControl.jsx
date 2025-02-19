import React, { Component, } from "react";
import { Row, Col, Container, Button } from "react-bootstrap";
import Config from "../config/Config";
import * as Three from 'three';
import Alert from "react-bootstrap/Alert";

export default class AgvControl extends Component {
    constructor(props) {
        super(props);
        this.state = {
            connected: false,
            ros: null,
            mapInitialized: false, // เพิ่ม state นี้เพื่อตรวจสอบว่า ROS2D Viewer ได้ถูกสร้างแล้วหรือยัง
            x: 0,
            y: 0,
            orientation: 0,
            linear_velocity: 0,
            angular_velocity: 0,
            allow: true,
            navClient: [],
            viewer: [],
            robotCommand: "Waiting Command",
            robotState: 0,
            robotGoal: '',
        };
        this.view_map = this.view_map.bind(this);
        this.handleClick = this.handleClick.bind(this);
        this.handleReset = this.handleReset.bind(this);
    }
    componentDidMount() {
        this.initConnection();
    }

    initConnection() {
        const ros = new window.ROSLIB.Ros();
        ros.on('connection', () => {
            console.log('Connection established in RobotState!');
            this.setState({ connected: true });
            console.log(this.state.navClient)
            if (!this.state.mapInitialized && (this.state.navClient.length === 0) && (this.state.viewer.length === 0)) {
                this.view_map(ros);
                this.getRobotState(ros);
                this.getRobotFeedback(ros);
            }
        });

        ros.on('close', () => {
            console.log('Connection is closed!');
            this.setState({ connected: false });
            // ros.close();
            setTimeout(() => {
                try {
                    ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
                } catch (error) {
                    console.log(error);
                }
            }, Config.RECONNECTION_TIMER);
        });

        ros.on('error', error => {
            console.log('Error: ', error);
            this.setState({ connected: false });
        });

        try {
            ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
        } catch (error) {
            console.log(error);
        }

        this.setState({ ros });
    }

    getRobotState(ros) {
        const pose_subscriber = new window.ROSLIB.Topic({
            ros,
            name: Config.POSE_TOPIC,
            messageType: 'geometry_msgs/PoseWithCovarianceStamped',
            queue_length: 10,
        });

        pose_subscriber.subscribe(message => {
            const { position, orientation } = message.pose.pose;
            this.setState({
                x: position.x.toFixed(4),
                y: position.y.toFixed(4),
                orientation: this.getOrientationFromQuaternion(orientation).toFixed(4),
            });
        });
        // create a subscriber for the velocities in the odom topic
        const velocity_subscriber = new window.ROSLIB.Topic({
            ros,
            name: Config.ODOM_TOPIC,
            messageType: 'nav_msgs/Odometry',
            queue_length: 10,
        });

        // callback function for the odom
        velocity_subscriber.subscribe(message => {
            // Use a single setState call to update multiple state properties
            this.setState({
                linear_velocity: message.twist.twist.linear.x.toFixed(4),
                angular_velocity: message.twist.twist.angular.z.toFixed(4),
            });
            // console.log(message)
        });


    }

    getOrientationFromQuaternion(ros_orientation_quaternion) {
        const q = new Three.Quaternion(
            ros_orientation_quaternion.x,
            ros_orientation_quaternion.y,
            ros_orientation_quaternion.z,
            ros_orientation_quaternion.w
        );

        const RPY = new Three.Euler().setFromQuaternion(q);

        return RPY.z * (180 / Math.PI);
    }

    getRobotFeedback(ros) {
        const feedback_subscriber = new window.ROSLIB.Topic({
            ros,
            name: Config.FEEDBACK_TOPIC,
            messageType: "std_msgs/String",
            queue_length: 10,
        });
    
        feedback_subscriber.subscribe(message => {
            const feedback = message.data;
            let updatedCommand = "";
            let updatedState = 0; // Assuming initial state is 0, adjust as needed
            let allow = false
            if (feedback === "OK") {
                updatedCommand = "Success to " + this.state.robotGoal;
                updatedState = 2;
                allow=true
            } else if (feedback === "NG") {
                updatedCommand = "Fail to " + this.state.robotGoal + ". Please contact admin";
                updatedState = 3;
            } else {
                // Handle other cases or unexpected feedback here
                updatedCommand = "Unknown feedback: " + feedback;
                updatedState = -1; // Example state for unknown feedback
            }
    
            this.setState({
                robotCommand: updatedCommand,
                robotState: updatedState,
                allow: allow,
            });
    
            console.log(this.state.robotCommand); // Optional: Logging the received message
        });
    }
    

    handleClick(robot_goal) {
        console.log("Function " + robot_goal + " called!");
        if (!this.state.ros) {
            console.error("ROS connection is not established.");
            return;
        }
        const robot_goal_pub = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: Config.TARGET_TOPIC,
            messageType: "std_msgs/String",
        });
        const rosbotGoalMsg = new window.ROSLIB.Message({
            data: robot_goal
        });
        robot_goal_pub.publish(rosbotGoalMsg);
        this.setState({
            robotCommand: "Go to station " + robot_goal,
            robotState: 1,
            robotGoal: robot_goal,
            allow:false,
        })
    }

    handleReset() {
        console.log("Reset function called!");
        this.setState({
            allow: true,
            robotCommand: "Waiting Command",
            robotState: 0,
            robotGoal: '',
        });
    }

    view_map(ros) {
        const viewer = new window.ROS2D.Viewer({
            divID: 'nav_div',
            width: 500,
            height: 250,
        });

        const navClient = new window.NAV2D.OccupancyGridClientNav({
            ros,
            rootObject: viewer.scene,
            viewer: viewer,
            serverName: '/move_base',
            withOrientation: true,
        });

        this.setState({ navClient: navClient, viewer: viewer, mapInitialized: true })
    }

    render() {
        const { allow, connected } = this.state;
        return (
            <div>
                <Container>
                    <h1 className="text-center mt-10" style={{ fontWeight: "bold" }}>Robot Control Status</h1>
                    <Row>
                        <Col>
                            {/* Connection */}
                            <div>
                                <Alert className='text-center m-3' variant={this.state.connected ? "success" : "danger"} style={{ backgroundColor: this.state.connected ? "#28a745" : "#dc3545", fontWeight: "bold", color: "white" }}>
                                    {this.state.connected ? "Robot Connected" : "Robot Disconnected"}
                                </Alert>
                            </div>
                        </Col>
                    </Row>
                    <Row>
                        <Col>
                            <Row>
                                <h5>Point for Goal</h5>
                                <div >
                                    <Button
                                        variant="outline-primary"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("HOME") : null}
                                        disabled={!allow || !connected}
                                        // style={{ color: "black", padding: "20px",borderColor: "black" }}
                                        style={{ padding: "40px", }}
                                        
                                    >
                                        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Home&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
                                    </Button>{' '}

                                    <Button
                                        variant="outline-success"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("A") : null}
                                        disabled={!allow || !connected}
                                        style={{ padding: "40px",marginLeft: '1mm', }}
                                    >
                                        Position: A
                                    </Button>{' '}
                                    <Button
                                        variant="outline-warning"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("B") : null}
                                        disabled={!allow || !connected}
                                        style={{ padding: "40px",marginLeft: '1mm', }}
                                    >
                                        Position: B
                                    </Button>{' '}
                                </div>
                            </Row>
                            <Row style={{ marginTop: '2mm' }}>
                                <div >
                                    <Button
                                        variant="outline-danger"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("C") : null}
                                        disabled={!allow || !connected}
                                        background-color="#333"
                                        style={{ padding: "40px", }}

                                    >
                                        Position: C
                                    </Button>{' '}
                                    <Button
                                        variant="outline-primary"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("D") : null}
                                        disabled={!allow || !connected}
                                        style={{ padding: "40px",marginLeft: '1mm', }}

                                    >
                                        Position: D
                                    </Button>{' '}
                                    <Button
                                        variant="outline-info"
                                        size="md" // กำหนดขนาดของปุ่มใหญ่ขึ้น
                                        onClick={allow && connected ? () => this.handleClick("E") : null}
                                        disabled={!allow || !connected}
                                        style={{ padding: "40px",marginLeft: '1mm', }}
                                    >
                                        Position: E
                                    </Button>{' '}
                                </div>
                            </Row>
                         {/* <Row style={{ marginTop: '1mm' }}>
                                <h5 className="justify-content-center">Robot status</h5>
                                <Col>
                                    <h5 className="justify-content-left">Position</h5>
                                    <p className=" justify-content-left mt-0">x: {this.state.x} y: {this.state.y} Orientation: {this.state.orientation}</p>
                                </Col>
                                <h5 className="justify-content-left">Velocities</h5>
                                <p className=" justify-content-left mt-0">Linear Velocity: {this.state.linear_velocity} Angular Velocity: {this.state.angular_velocity} </p>
                            </Row>  */}
                            <Row style={{ marginTop: '4mm' }}>
                                <Button
                                    variant="danger"
                                    size="md"
                                    // onClick={this.handleReset}
                                    onClick={connected ? () => this.handleReset() : null}
                                    style={{ marginLeft: '10px' }}
                                    disabled={!connected}
                                >
                                    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Reset&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
                                </Button>
                            </Row>
                        </Col>
                        <Col >
                            <Row>
                                <div id="nav_div">
                                    <h5>Map2D Viewer</h5>
                                </div>
                            </Row>
                            <Row>
                                <h5 style={{ display: "inline", margin: 0 }}>Robot state:&nbsp;
                                    <p style={{
                                        display: "inline",
                                        margin: 0,
                                        backgroundColor: this.state.robotState === 0 ? "yellow" : this.state.robotState === 1 ? "yellowgreen": this.state.robotState === 2 ? "green":this.state.robotState === 3 ?"red":"blue",
                                        fontWeight: "bold"
                                    }}>
                                        {this.state.robotCommand}
                                    </p>
                                </h5>
                            </Row>
                        </Col>
                    </Row>
                </Container>
            </div>
        )
    }
}
