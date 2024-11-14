import React from "react"
import "./css/GUICanvas.css";
import Car from "../resources/images/car-top-view.svg";

const Lasers = (props) => {
    const meter = 73; // 1m = 73px

    const [laser, setLaser] = React.useState([])
    const [maxRange, setMaxRange] = React.useState([])

    React.useEffect(() => {
        const callback = (message) => {
            if(message.data.update.map){
              const map_data = JSON.parse(message.data.update.map);
              console.log(map_data)
              setLaser (map_data.lasers[0])
              setMaxRange (map_data.ranges[0])
              console.log(map_data.ranges)
            }
            // Send the ACK of the msg
            window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
        };
        RoboticsExerciseComponents.commsManager.subscribe(
          [RoboticsExerciseComponents.commsManager.events.UPDATE],
          callback
        );
    
        return () => {
          console.log("TestShowScreen unsubscribing from ['state-changed'] events");
          RoboticsExerciseComponents.commsManager.unsubscribe(
            [RoboticsExerciseComponents.commsManager.events.UPDATE],
            callback
          );
        };
      }, []);

    return (
      <div style={{display: "flex",   width: "100%",
        height: "100%", backgroundColor: "#363233", position:"relative", overflow:"hidden"
      }}>
        <img src={Car} id="car"/>
        {laser.map(element => {
          var ang = -element[1]
          var length = (element[0] / 75) * meter;
          return (
            <hr className="laser-beam" 
              style={{
              rotate: "z "+ ang +"rad",
              width: length + "px",
              position: "absolute",
              background: "repeating-linear-gradient(to right,rgb(255, 112, 112),rgb(255, 112, 112) 73px,rgb(175, 29, 29)  73px,rgb(175, 29, 29) 146px)",
              backgroundSize: "100% 1px",
              bottom: "59%",
              left: "50%",
              transformOrigin: "0% 0%",
              zIndex: "3"}}
            />
        )})
        }
	    </div>
)};

export default Lasers;