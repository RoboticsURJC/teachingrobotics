import * as React from "react";
import PropTypes from "prop-types";

function SpecificFollowPerson(props) {
  const [image, setImage] = React.useState(null);
  React.useEffect(() => {
    const callback = (message) => {
      console.log(message);
      console.log(`** Context: ${context}`);   // BORRAR
      if (message.data.update.image) {
        console.log("New img received");
        const image = JSON.parse(message.data.update.image);
        setImage(`data:image/png;base64,${image.image}`);
      
        // Send the ACK of the img
        window.RoboticsExerciseComponents.commsManager.send("gui", "ack");      
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      console.log("TestShowScreen unsubscribing from ['state-changed'] events");
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  function listen_key(event) {

    // Agregar el event listener
    window.addEventListener("keypress", function (event) {
      console.log(`Key pressed: ${event.code}`);   // BORRAR
      if (['KeyS', 'KeyW', 'KeyA', 'KeyD', 'KeyX'].includes(event.code)) {
        window.RoboticsExerciseComponents.commsManager.send("gui", event.code);
      }
  
    });   
  }

  React.useEffect(() => {
    const callback = (message) => {
      console.log(`** Message: ${message}`); // BORRAR
      console.log(`** Context: ${context}`);   // BORRAR
      if (message.data.state === "application_running") {
        if (context.mapSelected === "follow_person_teleop") {
          listen_key()
        }           
      }
    }

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
        callback
      );
    };
  }, []);

  return (
    <div>
      <img height={"400px"} width={"100%"} src={image} id="gui_canvas" />
    </div>
  );
}

SpecificFollowPerson.propTypes = {
  context: PropTypes.any,
};

export default SpecificFollowPerson;
