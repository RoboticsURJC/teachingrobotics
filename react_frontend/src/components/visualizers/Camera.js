import React, { useEffect, useRef, useState } from "react";
import PropTypes from "prop-types";
import { drawImage, startStreaming } from "./showImagesColorFilter";
// The stream & capture
//var stream = document.getElementById('stream');
function decode_utf8(s) {
    return decodeURIComponent(escape(s));
}

function Camera() {
  const commsManager = window.RoboticsExerciseComponents.commsManager;
  const videoRef = useRef(null);
  const streamRef = useRef(null); // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  const [readyCamera, setReadyCamera] = useState(false);
  const [pauseCamera, setPauseCamera] = useState(false);
  const [isRunning, setIsRunning] = useState(true); // Estado para controlar si el intervalo está activo
  const intervalRef = useRef(null); // Referencia para guardar el ID del intervalo
  const [image, setImage] = React.useState(null);
  const [imageData, setImageData] = React.useState("");
    
  // Obtener el stream de la cámara
  useEffect(() => {
    if (!readyCamera) return;

    const startCamera = () => {
      const frameRate = 20;
      const constraints = {
        video: {
          frameRate: { ideal: frameRate, min: frameRate, max: frameRate },
        },
        audio: false,
      };
      if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices
          .getUserMedia({ video: true, audio: false })
          .then((stream) => {
            // Establecer el stream y asignarlo al video
            setStream(stream);
            if (videoRef.current) {
              videoRef.current.srcObject = stream;
              streamRef.current = stream; // Guardamos el stream en la referencia
              captureFrame();
            }
          })
          .catch((err) => console.log(err));
      }
    };

    startCamera();
    
    // Limpiar el stream cuando el componente se desmonte
    return () => {
      if ((streamRef, current)) {
        streamRef.getTracks().forEach((track) => track.stop());
      }
    };
  }, [readyCamera]);

  // Función para capturar un fotograma del video y convertirlo en una matriz CV_8UC4
  const captureFrame = () => {
    const video = videoRef.current;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");

    if (video && canvas && ctx) {
      // Establecer el tamaño del canvas igual al tamaño del video
      canvas.width = 320;
      canvas.height = 240;

      // Dibujar el frame del video en el canvas
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

      // Obtener los datos de la imagen (array de píxeles RGBA)
      const imageDataURL = canvas.toDataURL("image/jpeg");
      // Codificamos en base64
      // Enviar la matriz por WebSocket
      window.RoboticsExerciseComponents.commsManager.send(
        "gui",
        `pick${imageDataURL}`
      );
    }
  };

  // Llamar a captureFrame cada 100 ms para enviar imágenes de la cámara
  useEffect(() => {
    if (isRunning && readyCamera && !pauseCamera) {
      intervalRef.current = setInterval(() => {
        captureFrame();
      }, 0);
      return () => {
        if (intervalRef.current) 
        {
           clearInterval(intervalRef.current);
        }
      };
    }
  }, [isRunning, readyCamera, pauseCamera]);

  useEffect(() => {
    const callback = (message) => {
      if (
        message.data.state === "visualization_ready" ||
        message.data.state === "application_running"
      ) {
        setReadyCamera(true);
        setPauseCamera(false);
      } else if (message.data.state === "paused") {
        setPauseCamera(true);
      }
    };
    commsManager.subscribe([commsManager.events.STATE_CHANGED], callback);

    return () => {
      commsManager.unsubscribe([commsManager.events.STATE_CHANGED], callback);
    };
  }, []);

  // ack (you can get response from update_gui() in GUI.py)
  useEffect(() => {
    const callback = (message) => {
      if (message.data.update.id == "ack" && readyCamera && !pauseCamera)
      {
      	captureFrame();
      }
      if (isRunning && intervalRef.current) 
      {
      	 setIsRunning(false);
         clearInterval(intervalRef.current);
      }
      
      if (message.data.update.image) 
      {
         let image_data = JSON.parse(message.data.update.image);
         let source = decode_utf8(image_data.image);

         if (source.length > 0)
            setImageData(`data:image/jpeg;base64,${source}`);
         
         // Send the ACK of the msg
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
  }, [isRunning, readyCamera, pauseCamera]);

  return (
    <div style={{ display: "flex", width: "100%", height: "100%" }}>
      <div style={{ display: "flex", width: "100%", height: "100%" }}>
        <video ref={videoRef} autoPlay></video>
      </div>
      <div style={{ display: "flex", width: "100%", height: "100%" }}>
        {imageData && <img src={imageData} />}
      </div>
    </div>
  );
}

export default Camera;
