import houseMap from "../../resources/images/mapgrannyannie.png";
// To decode the image string we will receive from server
function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}

let image_right = new Image();
let image_left = new Image();

export function drawImage(data) {
  var canvas = document.getElementById("gui_canvas_right");

  // Request Animation Frame to remove the flickers
  function decode_utf8(s) {
      return decodeURIComponent(escape(s))
  }

  // Parse the Image Data
  var image_data = JSON.parse(data.image_right),
    source = decode_utf8(image_data.image_right),
    shape = image_data.shape_right;

  if (source != "" && shape instanceof Array) {
    canvas.src = "data:image/jpeg;base64," + source;
    canvas.width = shape[1];
    canvas.height = shape[0];
  }
}

// export function drawLeftImage(data) {
//   var canvas = document.getElementById("gui_canvas_left");

//   // Request Animation Frame to remove the flickers
//   function decode_utf8(s) {
//       return decodeURIComponent(escape(s))
//   }

//   // Parse the Image Data
//   var image_data = JSON.parse(data.image_left),
//     source = decode_utf8(image_data.image_left),
//     shape = image_data.shape_left;

//   if (source != "" && shape instanceof Array) {
//     canvas.src = "data:image/jpeg;base64," + source;
//     canvas.width = shape[1];
//     canvas.height = shape[0];
//   }
// }

export function drawLeftImage(data) {
  const canvas = document.getElementById("gui_canvas_left");
  const context = canvas.getContext("2d");

  // Configuración de la imagen de fondo
  const background = new Image();
  background.src = houseMap; // Imagen de fondo
  background.onload = () => {
    // Dibujar fondo cuando esté cargado
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.drawImage(background, 0, 0, canvas.width, canvas.height);

    // Dibujar los círculos según los datos de posición
    const realPose = data.real_pose ? JSON.parse(data.real_pose) : null;
    const noisyPose = data.noisy_pose ? JSON.parse(data.noisy_pose) : null;
    const estimatePose = data.estimate_pose ? JSON.parse(data.estimate_pose) : null;

    // Dibujar círculos si las posiciones están disponibles
    if (realPose) drawCircle(context, realPose, "red");
    if (noisyPose) drawCircle(context, noisyPose, "blue");
    if (estimatePose) drawCircle(context, estimatePose, "green");
  };
}

// Función auxiliar para dibujar un círculo en el contexto 2D
function drawCircle(context, pose, color) {
  const [x, y] = pose; // Asume que pose es un array [x, y]
  context.beginPath();
  context.arc(x, y, 10, 0, 2 * Math.PI); // Radio de 10 píxeles
  context.fillStyle = color;
  context.fill();
}