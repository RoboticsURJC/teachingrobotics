import * as React from "react";
import PropTypes from "prop-types";
import { drawImage, startStreaming } from "./helpers/showImagesColorFilter";
// The stream & capture
//var stream = document.getElementById('stream');
function decode_utf8(s) {
    return decodeURIComponent(escape(s));
}
function SpecificColorFilter(props) {
    const [image, setImage] = React.useState(null);
    const [imageData, setImageData] = React.useState("");
    React.useEffect(() => {
        console.log("TestShowScreen subscribing to ['update'] events");
        // Start Streaming
        //startStreaming()
        const callback = (message) => {
             console.log(message);

            if (message.data.update.image) {
                // drawImage(message.data.update);
		console.log("recibo showimage");
                let image_data = JSON.parse(message.data.update.image);
                let source = decode_utf8(image_data.image);

                if (source.length > 0)
                    setImageData(`data:image/jpeg;base64,${source}`);
            }

            // Send the ACK of the msg
            window.RoboticsExerciseComponents.commsManager.send("gui", "ack");
        };

        window.RoboticsExerciseComponents.commsManager.subscribe(
            [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
            callback
        );

        return () => {
            console.log(
                "TestShowScreen unsubscribing from ['state-changed'] events"
            );
            window.RoboticsExerciseComponents.commsManager.unsubscribe(
                [window.RoboticsExerciseComponents.commsManager.events.UPDATE],
                callback
            );
        };
    }, []);

    return (
        <div style={{ display: "flex", width: "100%", height: "100%" }}>
            {/* <canvas id='gui_canvas' style={{ display: "none" }}></canvas> */}
            {imageData && <img src={imageData} />}
        </div>
    );
}

SpecificColorFilter.propTypes = {
    circuit: PropTypes.string,
};


export default SpecificColorFilter;
