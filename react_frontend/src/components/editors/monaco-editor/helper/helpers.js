import {
  basic_snippets,
  guiAndHalAutoCompleteObj,
  importSnippetsObj,
} from "./../index";
import {
  pylint_error,
  pylint_warning,
  pylint_convention,
  pylint_refactor,
  pylint_fatal,
} from "../constants";

// post and response code format
export const fetchFormatCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  setMonacoEditorSourceCode,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/format/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRFToken": context.csrf,
      },

      body: JSON.stringify({
        code: monacoEditorSourceCode,
      }),
    });

    const data = await response.json();
    if (response.ok) {
      setMonacoEditorSourceCode(data.formatted_code);
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    console.log("error ", error);
  }
};

// post and response code analysis
export const fetchAnalysisCode = async ({
  baseUrl,
  monacoEditorSourceCode,
  controller,
}) => {
  try {
    const response = await fetch(`${baseUrl}/api/v1/analysis/`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRFToken": context.csrf,
      },
      body: JSON.stringify({
        code: monacoEditorSourceCode,
        disable_errors: [
          ...pylint_error,
          ...pylint_warning,
          ...pylint_convention,
          ...pylint_refactor,
          ...pylint_fatal,
        ],
      }),
      signal: controller.signal,
    });

    const data = await response.json();

    if (response.ok) {
      return data;
    } else {
      console.error("Error formatting code:", data.error);
    }
  } catch (error) {
    if (error.name !== "AbortError") {
      console.log(error);
    }
  }
};

export const getMarkerSeverity = ({ type, monaco }) => {
  switch (type) {
    case "refactor":
    case "convention":
      return monaco.MarkerSeverity.Info;
    case "error":
      return monaco.MarkerSeverity.Error;
    case "warning":
    case "fatal":
      return monaco.MarkerSeverity.Warning;
    default:
      return monaco.MarkerSeverity.Error;
  }
};

// Snippets Builder
export const snippetKind = ({ kind, monaco }) => {
  switch (kind) {
    case "variable":
      return monaco.languages.CompletionItemKind.Variable;
    case "method":
      return monaco.languages.CompletionItemKind.Method;
    case "snippet":
      return monaco.languages.CompletionItemKind.Snippet;
    case "keyword":
      return monaco.languages.CompletionItemKind.Keyword;
    case "function":
      return monaco.languages.CompletionItemKind.Function;
    default:
      return monaco.languages.CompletionItemKind.Variable;
  }
};

// hal & gui auto complete
export const getHalGuiMethods = ({ importName }) => {
  const pathName = window.location.pathname;
  let exerciseName = pathName.split("/").filter(Boolean);
  exerciseName = exerciseName[exerciseName.length - 1];
  exerciseName = `_${exerciseName}`;

  // if no object found by exercise name
  if (!guiAndHalAutoCompleteObj[exerciseName]) {
    return [];
  }

  if (importName === "GUI") {
    return guiAndHalAutoCompleteObj[exerciseName].gui;
  } else if (importName === "HAL") {
    return guiAndHalAutoCompleteObj[exerciseName].hal;
  }

  return [];
};

export const snippetsBuilderV2 = ({
  snippetName,
  monaco,
  range,
  importName,
}) => {
  const snippets = [];
  let importSnippets;

  // basic_snippets
  if (snippetName === "basic_snippets") {
    importSnippets = basic_snippets;
  } else if (snippetName === "hal_gui") {
    // hal_gui
    importSnippets = getHalGuiMethods({ importName });
  } else if (snippetName === "import") {
    // import
    importSnippets = importSnippetsObj[`_${importName}`];
  }

  if (!importSnippets || !importSnippets.length) return [];

  importSnippets.forEach((snippet) => {
    if (snippet.label && snippet.code) {
      snippets.push({
        label: snippet.label,
        kind: snippetKind({ kind: snippet.type, monaco }),
        insertText: snippet.code,
        insertTextRules:
          monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
        range: range,
      });
    }
  });

  return snippets;
};

// local storage data
export const setEditorSettingsWidgetsData = (data) => {
  const data_string = JSON.stringify(data);
  localStorage.setItem("editorSettingsWidgets", data_string);
};

export const getEditorSettingsWidgetsData = () => {
  const data = localStorage.getItem("editorSettingsWidgets");

  if (data) return JSON.parse(data);
  return null;
};
