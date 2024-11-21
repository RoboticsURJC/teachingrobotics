import {
  listed_python_packages,
  snippetsBuilderV2,
  getEditorVariables,
  getEditorFunctions,
  extractClassesAndMembers,
  findClassNameByInstance,
  extractPythonImports,
} from "./../index";

// Main Editor Snippets
export const monacoEditorSnippet = ({ monaco }) => {
  monaco.languages.register({ id: "python" });
  // Register a completion item provider for the new language
  monaco.languages.registerCompletionItemProvider("python", {
    triggerCharacters: ["."],
    provideCompletionItems: async (model, position) => {
      var word = model.getWordUntilPosition(position);
      var range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      // get text until position
      const textUntilPosition = model.getValueInRange({
        startLineNumber: 1,
        startColumn: 1,
        endLineNumber: position.lineNumber,
        endColumn: position.column,
      });
      // get all text data in editor
      const text = model.getValue();

      // import extract
      const allLines = text.split("\n").filter(Boolean);
      const allImports = [];
      allLines.forEach((line) => {
        const importData = extractPythonImports(line);

        if (importData.length) {
          if (listed_python_packages.includes(importData[0].importName)) {
            allImports.push(importData[0]);
          }
        }
      });

      // check valid import
      if (allImports.length) {
        const importMatch = allImports.find((imp) => {
          const match = textUntilPosition.match(
            new RegExp(`(${imp.alias})\\.$`)
          );
          if (match) return imp;
        });

        // match with listed import
        if (importMatch) {
          const { importName } = importMatch;

          // HAL & GUI
          if (importName === "GUI" || importName === "HAL") {
            const suggestions = snippetsBuilderV2({
              snippetName: "hal_gui",
              monaco,
              range,
              importName,
            });

            return { suggestions };
          } else {
            // Other Imports (ex: Numpy, Math)
            const suggestions = snippetsBuilderV2({
              snippetName: "import",
              monaco,
              range,
              importName,
            });

            return { suggestions };
          }
        }
      }

      //* Class & Objects
      const classes = extractClassesAndMembers(text);
      // Match the instance name before the dot (e.g. obj.)
      const classObjMatch = textUntilPosition.match(/(\w+)\.$/);

      if (classObjMatch) {
        // class obj
        const instanceName = classObjMatch[1];
        const className = findClassNameByInstance(text, instanceName);

        if (className && classes[className]) {
          const attributesSet = [...new Set(classes[className].attributes)];
          const methodsSet = [...new Set(classes[className].methods)];

          const suggestions = [
            ...attributesSet.map((attr) => ({
              label: attr,
              kind: monaco.languages.CompletionItemKind.Field,
              insertText: attr,
              documentation: `Attribute of ${className}`,
            })),
            ...methodsSet.map((method) => ({
              label: method,
              kind: monaco.languages.CompletionItemKind.Method,
              insertText: `${method}()`,
              documentation: `Method of ${className}`,
            })),
          ];

          return { suggestions: suggestions };
        }
      } else {
        // custom suggestions

        // import
        const importSet = new Set();
        allImports.forEach((imp) => {
          // importSet.add(imp.importName);
          importSet.add(imp.alias);
        });
        // Class
        const classesSet = new Set();
        Object.keys(classes).forEach((key) => classesSet.add(key));

        // custom suggestions
        const lines = model.getLinesContent();
        // func
        const functions = getEditorFunctions({ lines, monaco, range });

        // variable
        const variables = getEditorVariables({ lines, monaco, range });

        // Get all pre-defined snippets
        const preDefinedSnippets = snippetsBuilderV2({
          snippetName: "basic_snippets",
          monaco,
          range,
          importName: "",
        });

        let suggestions = [
          // import snippet
          // add import
          ...Array.from(importSet).map((imp) => ({
            label: imp,
            kind: monaco.languages.CompletionItemKind.Class,
            insertText: imp,
            range: range,
          })),
          // class-obj
          ...Array.from(classesSet).map((cls) => ({
            label: cls,
            kind: monaco.languages.CompletionItemKind.Class,
            insertText: cls,
            range: range,
          })),
          ...functions,
          ...variables,
          ...preDefinedSnippets,
        ];

        return { suggestions: suggestions };
      }
    },
  });
};
