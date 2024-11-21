import React, { useEffect, useState } from "react";
import LoadingButton from "@mui/lab/LoadingButton";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import PauseIcon from "@mui/icons-material/Pause";

const PlayPause = (props) => {
  const [loading, setLoading] = useState(false);
  const [applicationRunning, setApplicationRunning] = useState(true);
  const [applicationPaused, setApplicationPaused] = useState(false);
  const [disabled, setDisabled] = useState(true);
  const [editorChanged, setEditorChanged] = useState(false);

  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const config = JSON.parse(
    document.getElementById("exercise-config").textContent
  );

  useEffect(() => {
    const callback = (message) => {
      const state = message.data.state;
      setApplicationPaused(state === "paused");
      setApplicationRunning(state === "application_running");
      setDisabled(
        !(
          state === "visualization_ready" ||
          state === "application_running" ||
          state === "paused"
        )
      );
    };

    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  React.useEffect(() => {
    RoboticsReactComponents.CodeEditor.OnEditorCodeChanged(() => {
      setEditorChanged(true);
    });
  }, []);

  const play = async () => {
    setLoading(true);
    let editorCode = "";
    editorCode = RoboticsReactComponents.CodeEditor.getCode();

    if (applicationPaused) {
      if (editorChanged) {
        await resetCode(editorCode);
      }
      commsManager.resume();
    } else {
      runCode(editorCode);
    }
    setLoading(false);
    setEditorChanged(false);
  };

  const resetCode = async (code) => {
    setLoading(true);
    const errorMessage =
      "Syntax or dependency error, check details on the console.\n";
    
    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${config[0].exercise_id}/user_code_zip`;

    try {
      const response = await fetch(requestUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          code: code
        }),
      });

      const zipBlob = await response.blob();

      if (!response.ok) {
        console.error("Error formatting code:", zip.error);
        return 
      }
      var reader = new FileReader();
      reader.readAsDataURL(zipBlob);
      reader.onloadend = async function () {
        // Get the zip in base64
        var base64data = reader.result;
        window.RoboticsExerciseComponents.commsManager
          .run({
            code: base64data
          })
          .then(() => {})
          .catch((response) => {
            let linterMessage = JSON.stringify(response.data.message).split(
              "\\n"
            );
            RoboticsReactComponents.MessageSystem.Alert.showAlert(
              errorMessage,
              "error"
            );
            console.log(`Received linter message ·${linterMessage}`);
          });
      };
    } catch (error) {
      console.log(error);
      return 
    }
  }

  const runCode = async (code) => {
    setLoading(true);
    const errorMessage =
      "Syntax or dependency error, check details on the console.\n";
    
    const serverBase = `${document.location.protocol}//${document.location.hostname}:7164`;
    let requestUrl = `${serverBase}/exercises/exercise/${config[0].exercise_id}/user_code_zip`;

    try {
      const response = await fetch(requestUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          code: code
        }),
      });

      const zipBlob = await response.blob();

      if (!response.ok) {
        console.error("Error formatting code:", zip.error);
        return 
      }
      var reader = new FileReader();
      reader.readAsDataURL(zipBlob);
      reader.onloadend = async function () {
        // Get the zip in base64
        var base64data = reader.result;
        window.RoboticsExerciseComponents.commsManager
          .terminate_application()
          .then(() => {
            window.RoboticsExerciseComponents.commsManager
              .run({
                code: base64data
              })
              .then(() => {})
              .catch((response) => {
                let linterMessage = JSON.stringify(response.data.message).split(
                  "\\n"
                );
                RoboticsReactComponents.MessageSystem.Alert.showAlert(
                  errorMessage,
                  "error"
                );
                console.log(`Received linter message ·${linterMessage}`);
              });
          });
      };
    } catch (error) {
      console.log(error);
      return 
    }
  };

  const pause = () => {
    setLoading(true);
    window.RoboticsExerciseComponents.commsManager
      .pause()
      .then(() => {})
      .catch((response) => console.log(response))
      .finally(() => setLoading(false));
  };

  return (
    <LoadingButton
      disabled={disabled}
      id={"loadIntoRobot"}
      loading={loading}
      color={"secondary"}
      onClick={applicationRunning ? pause : play}
      sx={{ m: 0.5 }}
      variant={"outlined"}
    >
      {applicationRunning ? <PauseIcon /> : <PlayArrowIcon />}
    </LoadingButton>
  );
};

export default PlayPause;
