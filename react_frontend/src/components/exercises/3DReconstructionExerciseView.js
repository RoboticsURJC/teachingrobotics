import { Box, Typography } from "@mui/material";
import * as React from "react";
import ExerciseControl from "./ExerciseControl";
import AceEditorRobot from "./AceEditorRobot";
import VisualizationComponents from "./VisualizationComponents";
import GuiCanvasThree from "./GuiCanvasThree";
import FrequencyMenu from "./FrequencyMenu";
import GazeboViewer from "./GazeboViewer";
import VncConsoleViewer from "./VncConsoleViewer";
import LoadModalView from "./LoadModalView";
import CustomAlert from "./CustomAlert";
import ErrorModalView from "./ErrorModalView";
import PropTypes from "prop-types";
import CanvasThree from "./CanvasThree";

export default function _3DReconstructionExerciseView(props) {
  return (
    <Box id="exercise-view">
      <ExerciseControl context={props.context} />
      <Box
        sx={{
          display: "flex",
          border: "2px solid",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
        <Typography align={"center"} color={"primary"} variant={"h4"}>
          {" "}
          Start Coding !{" "}
        </Typography>
        <AceEditorRobot context={props.context} />
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          Visualization
        </Typography>
        <CanvasThree context={props.context} />
        <VisualizationComponents>
          <GuiCanvasThree context={props.context} />
          <FrequencyMenu context={props.context} />
        </VisualizationComponents>
      </Box>

      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          border: "2px solid",
          p: 1,
          m: 1,
        }}
      >
        <Typography align={"center"} m={2} color={"primary"} variant={"h4"}>
          {" "}
          Simulation and Console !{" "}
        </Typography>
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            justifyContent: "space-around",
            p: 2,
          }}
        >
          <GazeboViewer context={props.context} />
          <VncConsoleViewer context={props.context} />
        </Box>
      </Box>
      <LoadModalView context={props.context} />
      <CustomAlert context={props.context} />
      <ErrorModalView context={props.context} />
    </Box>
  );
}

_3DReconstructionExerciseView.propTypes = {
  context: PropTypes.any,
};
